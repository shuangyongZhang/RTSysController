using cszmcaux;
using MotorControlApp.Configuration;
using MotorControlApp.Waveforms;
using System.Text;

namespace MotorControlApp.Devices;

/// <summary>
/// 正运动 ZMotion 控制卡（PAC 本机 / 独立 PCI 卡 / 网口）。
/// 连接：ZAux_FastOpen(type, target, timeout)
/// 运动：ZAux_Direct_Single_Vmove 连续速度模式（dir=1/-1）
/// 停止：ZAux_Direct_Single_Cancel(imode=2 减速停止)
/// </summary>
public class ZMotionDeviceController : IDeviceController
{
    private readonly AppConfig _cfg;
    private readonly object _lock = new();

    private IntPtr _handle;           // 非 0 表示已连接
    private volatile bool _connected;
    private bool _disposed;
    private CancellationTokenSource? _cts;
    private bool _wasOutOfLimit;      // 限位监控：上一轮是否越界（用于只提示一次）
    private volatile int _lastMoveDir;  // 最后一次用户指令方向：0=停止/未知，+1=前进，-1=后退（替代 speed 正负判断）
    private volatile bool _homing;        // 找零（回零）进行中
    private int _connectedAxis = -1;     // 本次连接配置的轴号（防止在线改轴号后把参数下发到其他轴）

    // ---- 运动波形（CSV 逐拍下发）----
    private WaveformRuntime? _wf;
    private MotionUnits? _wfUnits;
    private CancellationTokenSource? _wfCts;
    private volatile bool _wfRunning;
    private int _wfLastDir;               // 波形运行中最后一次下发方向，避免重复 Vmove/Cancel

    public ZMotionDeviceController(AppConfig config)
    {
        ArgumentNullException.ThrowIfNull(config);
        _cfg = config;
    }

    public bool IsConnected => _connected;
    public event EventHandler<SensorDataEventArgs>? SensorDataReceived;
    public event EventHandler<string>? LimitTriggered;
    public event EventHandler<WaveformTickEventArgs>? WaveformTick;
    public event EventHandler<WaveformStopReason>? WaveformStopped;

    public bool IsWaveformRunning => _wfRunning;
    public bool IsHoming => _homing;

    public Task<bool> ConnectAsync(CancellationToken cancellationToken = default)
    {
        return Task.Run(() =>
        {
            lock (_lock)
            {
                if (_connected || _handle != IntPtr.Zero) return true;
                if (_disposed) throw new ObjectDisposedException(nameof(ZMotionDeviceController));
            }

            try
            {
                var conn = _cfg.Connection;
                int type = conn.Type.Value.ToUpperInvariant() switch
                {
                    "SERIAL" => 1,
                    "ETHERNET" => 2,
                    "PCI" => 4,
                    "LOCAL" => 5,
                    _ => 5 // 默认 LOCAL
                };

                int rc = zmcaux.ZAux_FastOpen(type, conn.Target.Value, (uint)conn.TimeoutMs.Value, out IntPtr handle);
                if (rc != 0)
                    throw new InvalidOperationException($"ZAux_FastOpen 失败 rc={rc}（type={type} target={conn.Target.Value}）");
                _handle = handle;

                // ========== EtherCAT 总线初始化 ==========
                // 参考正运动官方例程8-总线控制运动 的做法：
                //   控制器 ROM 里已经通过 RTSys 下载了包含 Ecat_Init 的 BASIC 程序（昨天下载过）
                //   直接用 ZAux_Execute 启动任务1跑总线初始化，response length 传 0（写命令不需要返回值）
                //   然后轮询 BASIC 全局变量 Bus_InitStatus 等它变成 1=成功
                try
                {
                    StringBuilder rbuf = new StringBuilder(4096);
                    int r2 = zmcaux.ZAux_Execute(_handle, "RUNTASK 1,Ecat_Init", rbuf, 0);
                    if (r2 != 0)
                        throw new InvalidOperationException($"RUNTASK 失败 rc={r2}。" +
                            (r2 == 2033 ? "控制器 ROM 里没有 Ecat_Init 函数，请在 RTSys 里重新下载 ZPJ 项目包到 ROM" : ""));
                }
                catch (InvalidOperationException)
                {
                    throw;
                }
                catch (Exception ex)
                {
                    throw new InvalidOperationException($"启动总线初始化失败：{ex.Message}。" +
                        "请确认控制器已上电，且 RTSys 没有占用网口连接。");
                }

                // 轮询 Bus_InitStatus，最多等 20s
                int initStatus = -1;
                for (int wait = 0; wait < 200; wait++)
                {
                    Thread.Sleep(100);
                    try
                    {
                        float fstatus = -1;
                        zmcaux.ZAux_Direct_GetUserVar(_handle, "Bus_InitStatus", ref fstatus);
                        initStatus = (int)fstatus;
                    }
                    catch { continue; }

                    if (initStatus == 1) break;      // 成功
                    if (initStatus == 2) break;      // 节点数不符（总线配置和实际硬件不一致）
                }
                if (initStatus != 1 && initStatus != 2)
                    throw new InvalidOperationException(
                        $"总线初始化未完成（Bus_InitStatus={initStatus}），" +
                        "请在 RTSys 里确认总线配置与实际硬件一致，且 ECAT初始化.Bas 已下载到控制器。");

                // 配置轴参数（参考例程 Form1.OnStart）
                ZMotionConfig z = _cfg.ZMotion;
                int axis = z.GetAxisNumber();

                ThrowRc(zmcaux.ZAux_Direct_SetAtype(_handle, axis, z.GetAtType()), $"SetAtype({axis},{z.GetAtType()})");  // 1=本地脉冲 65=EtherCAT CSP 66=CSV 67=CST
                ThrowRc(zmcaux.ZAux_Direct_SetUnits(_handle, axis, z.GetUnits()), $"SetUnits({axis})");
                ThrowRc(zmcaux.ZAux_Direct_SetLspeed(_handle, axis, z.GetLspeed()), $"SetLspeed({axis})");
                ThrowRc(zmcaux.ZAux_Direct_SetSpeed(_handle, axis, z.GetSpeed()), $"SetSpeed({axis})");
                ThrowRc(zmcaux.ZAux_Direct_SetAccel(_handle, axis, z.GetAccel()), $"SetAccel({axis})");
                ThrowRc(zmcaux.ZAux_Direct_SetDecel(_handle, axis, z.GetDecel()), $"SetDecel({axis})");
                ThrowRc(zmcaux.ZAux_Direct_SetSramp(_handle, axis, z.GetSramp()), $"SetSramp({axis})");

                // 软限位（P0 防爆冲）
                ThrowRc(zmcaux.ZAux_Direct_SetFsLimit(_handle, axis, z.GetSoftLimitPos()), $"SetPosLimit({axis},{z.GetSoftLimitPos()})");
                ThrowRc(zmcaux.ZAux_Direct_SetRsLimit(_handle, axis, z.GetSoftLimitNeg()), $"SetNegPosLimit({axis},{z.GetSoftLimitNeg()})");

                // 使能轴
                ThrowRc(zmcaux.ZAux_Direct_SetAxisEnable(_handle, axis, 1), $"SetAxisEnable({axis},1)");
                _connectedAxis = axis;    // 记录本次连接配置的轴号（ApplyMotionParams 校验用）

                _cts = new CancellationTokenSource();
                _connected = true;

                // ========== EC8124 AD 模块初始化（量程 ±5V + 通道使能） ==========
                // EC8124 的量程(0x2000)/通道使能(0x2002)是运行时 SDO 参数，不会持久化：
                // 硬件断电重启会丢，Ecat_Init 的 SLOT_STOP→SLOT_START 重新枚举总线也会丢。
                // 之前只有"总线诊断"按钮(ScanAINAndShow→InitEC8124)会写这两个参数，
                // 所以每次重启硬件后传感器无值、手动点一次诊断才恢复 —— 现在连接时自动补上。
                // 失败不阻断连接：读不到总线 AD 时 SensorLoopAsync 会自动降级到本体 GetAD。
                for (int attempt = 0; attempt < 5; attempt++)
                {
                    Thread.Sleep(500);      // 总线刚启动时从站可能尚未就绪，稍等再写 SDO
                    try
                    {
                        InitEC8124(1, 1);   // node=1、量程 ±5V，与"总线诊断"按钮里的调用完全一致
                        if (ReadEC8124AD(1).Count(e => e.rc == 0) >= 2)
                            break;          // 至少 2 路读通即初始化成功
                    }
                    catch { /* SDO 暂时不通则重试 */ }
                }
                _ = Task.Run(async () =>
                {
                    await Task.Delay(1500, cancellationToken: _cts.Token);
                    await SensorLoopAsync(_cts.Token);
                }, cancellationToken);
                _ = Task.Run(() => LimitMonitorLoopAsync(_cts.Token), cancellationToken);
                return true;
            }
            catch (DllNotFoundException ex)
            {
                throw new InvalidOperationException("找不到正运动控制库 zauxdll.dll / zmotion.dll，请放到程序目录", ex);
            }
        }, cancellationToken);
    }

    /// <summary>
    /// 原地置零：把当前指令位置（Mpos）与反馈位置（Dpos）同时设为 0，并不运动。
    /// 仅用于“把当前位置定义为某个坐标原点”（如波形开环积分起点）；要真实回到伺服 0 点请用 <see cref="MoveToServoZero"/>。
    /// </summary>
    public void ZeroPosition()
    {
        EnsureConnected();
        int axis = _cfg.ZMotion.GetAxisNumber();
        ThrowRc(zmcaux.ZAux_Direct_SetMpos(_handle, axis, 0f), $"SetMpos({axis})");
        ThrowRc(zmcaux.ZAux_Direct_SetDpos(_handle, axis, 0f), $"SetDpos({axis})");
    }

    /// <summary>
    /// 归零：回到伺服驱动器里存好的 0 点——在当前坐标系对坐标 0 做一次绝对定位运动（MOVEABS 0）。
    /// 前提：驱动器 0 点已与控制器坐标对齐且掉电保持（本机情况）；用常规 Speed/Accel 运动，软限位照常生效。
    /// 与 <see cref="ZeroPosition"/>（原地把 Mpos/Dpos 设为 0、不运动）不同：本方法会真实运动到 0。
    /// 阻塞直到到位或超时——上层必须放到后台线程调用，避免卡住 UI。
    /// </summary>
    public void MoveToServoZero()
    {
        EnsureConnected();
        if (_wfRunning) throw new InvalidOperationException("波形运行中，请先停止波形再归零");
        if (_homing) throw new InvalidOperationException("找零正在进行中，请稍候");

        ZMotionConfig z = _cfg.ZMotion;
        int axis = z.GetAxisNumber();
        int timeoutMs = Math.Max(1000, z.HomePhaseTimeoutMs.Value);
        _lastMoveDir = 0;   // 归零是规划定位运动，方向由轨迹决定，不让限位看门狗按手动方向误拦

        // 用常规运行参数做定位：复位到配置 Speed/Accel 并回读校验。
        // 之前用 TryApi 吞错，波形/找零残留的低速会让归零慢慢爬直到 30s 超时。
        ApplyRunSpeed(z, axis);

        // 已在 0 附近（< 回中容差）则不必运动，避免无谓抖动
        float tolUnits = Math.Max(0f, z.HomeCenterTolMm.Value) / 10f * z.GearDenominator.Value / z.Lead.Value / z.Units.Value;
        float cur = 0;
        zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref cur);
        if (Math.Abs(cur) <= Math.Max(1f, tolUnits))
            return;

        // 锁在硬限位上且归零方向是脱困方向：先反向走离限位开关（Mpos 验证）→ 清报警 → 再定位到 0
        int hwSt = ReadAxisStatus(axis);
        if (((hwSt & 0x10) != 0 && cur <= 0) || ((hwSt & 0x20) != 0 && cur >= 0))
        {
            // 走离量取“到 0 点距离”与 5mm 的较小者，脱困到位即等于归零方向正确
            float escapeTarget = Math.Min(Math.Abs(cur), 5f / 10f * z.GearDenominator.Value / z.Lead.Value);
            if (escapeTarget < 1f) escapeTarget = 1f;
            string eDiag = EscapeHardLimit(axis, cur >= 0 ? -1 : 1, z, escapeTarget);
            LimitTriggered?.Invoke(this, $"归零前硬限位脱困：{eDiag}");
        }

        // 绝对定位到坐标 0（= 伺服存好的 0 点）
        ThrowRc(zmcaux.ZAux_Direct_Single_MoveAbs(_handle, axis, 0f), $"归零 MoveAbs({axis},0)");
        var sw = System.Diagnostics.Stopwatch.StartNew();
        bool done = false;
        while (sw.ElapsedMilliseconds < timeoutMs)
        {
            Thread.Sleep(50);
            int idle = 0;
            zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref idle);   // IfIdle:0=运行 1=停止
            if (idle != 0) { done = true; break; }
        }
        if (!done)
        {
            TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
            throw new InvalidOperationException($"归零超时（{timeoutMs / 1000}s 未回到 0 点），已减速停止。请确认伺服 0 点可达、且坐标已与驱动器对齐。");
        }
    }

    public void Disconnect()
    {
        lock (_lock)
        {
            if (!_connected) return;
            _connected = false;
            try { StopWaveform(); } catch { }
            _cts?.Cancel();
            _cts?.Dispose();
            _cts = null;
            if (_handle != IntPtr.Zero)
            {
                try { zmcaux.ZAux_Close(_handle); } catch { }
                _handle = IntPtr.Zero;
            }
        }
    }

    public void Forward() => RunMove(1, "前进(+)");
    public void Backward() => RunMove(-1, "后退(-)");

    /// <summary>读取当前逻辑位置（Dpos）。未连接抛 InvalidOperationException。</summary>
    public float GetCurrentDpos()
    {
        EnsureConnected();
        int axis = _cfg.ZMotion.GetAxisNumber();
        float dpos = 0;
        int rc = zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos);
        ThrowRc(rc, $"GetDpos({axis})");
        return dpos;
    }

    /// <summary>读取当前规划速度（VpSpeed，units/s）。未连接抛 InvalidOperationException。</summary>
    public float GetCurrentVpSpeedUnits()
    {
        EnsureConnected();
        int axis = _cfg.ZMotion.GetAxisNumber();
        float v = 0;
        ThrowRc(zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref v), $"GetVpSpeed({axis})");
        return v;
    }

    /// <summary>
    /// 连接状态下即时下发轴运动参数（脉冲当量/最低速度/速度/加减速度/S曲线）。
    /// 这些参数之前只在 ConnectAsync 里下发一次，"应用参数"按钮只更新了本地 config，
    /// 导致必须断开重连才生效 —— 现在点击"应用参数"时立即调用本方法下发到控制器。
    /// 不含 Atype/轴使能/软限位：轴类型不宜在线切换，软限位有独立的"应用限位"按钮。
    /// </summary>
    public void ApplyMotionParams(float rate = 0)
    {
        EnsureConnected();
        ZMotionConfig z = _cfg.ZMotion;
        int axis = z.GetAxisNumber();
        if (axis != _connectedAxis)
            throw new InvalidOperationException($"轴号已从 {_connectedAxis} 改为 {axis}，需断开重连后新轴号才生效");
        ThrowRc(zmcaux.ZAux_Direct_SetUnits(_handle, axis, z.GetUnits()), $"SetUnits({axis})");
        ThrowRc(zmcaux.ZAux_Direct_SetLspeed(_handle, axis, z.GetLspeed()), $"SetLspeed({axis})");
        ThrowRc(zmcaux.ZAux_Direct_SetSpeed(_handle, axis, z.GetSpeed()), $"SetSpeed({axis})");
        ThrowRc(zmcaux.ZAux_Direct_SetAccel(_handle, axis, z.GetAccel()), $"SetAccel({axis})");
        ThrowRc(zmcaux.ZAux_Direct_SetDecel(_handle, axis, z.GetDecel()), $"SetDecel({axis})");
        ThrowRc(zmcaux.ZAux_Direct_SetSramp(_handle, axis, z.GetSramp()), $"SetSramp({axis})");
        if (rate != 0)
        {
            ApplySoftLimits(_cfg.ZMotion.SoftLimitPos.Value, _cfg.ZMotion.SoftLimitNeg.Value);
        }
    }

    /// <summary>
    /// 手动运动/归零前，把控制器轴的速度参数复位为配置值，并回读校验 SPEED。
    /// Vmove、MOVEABS 都是按控制器里 SPEED 的“当前值”跑的，而波形每拍改 Speed、StartWaveform 压 Lspeed，
    /// 之前用 TryApi 吞错下发——残留低速或下发失败时表现就是“界面改速度没效果、归零慢到超时”。
    /// 回读不符直接报错（与找零 SetHomeMotion 的设速校验同一范式），能区分“没发下去”和“被控制器任务覆盖”。
    /// </summary>
    private void ApplyRunSpeed(ZMotionConfig z, int axis)
    {
        float speed = z.GetSpeed();
        TryApi(() => zmcaux.ZAux_Direct_SetLspeed(_handle, axis, z.GetLspeed()));
        TryApi(() => zmcaux.ZAux_Direct_SetAccel(_handle, axis, z.GetAccel()));
        TryApi(() => zmcaux.ZAux_Direct_SetDecel(_handle, axis, z.GetDecel()));
        ThrowRc(zmcaux.ZAux_Direct_SetSpeed(_handle, axis, speed), $"SetSpeed({axis},{speed:F0})");

        float back = 0;
        if (zmcaux.ZAux_Direct_GetSpeed(_handle, axis, ref back) == 0 &&
            Math.Abs(back - speed) > Math.Max(2f, speed * 0.05f))
        {
            float backMmS = back * 10f * z.Lead.Value / z.GearDenominator.Value;
            throw new InvalidOperationException(
                $"速度下发未生效：请求 SPEED={speed:F0}，控制器回读={back:F0}（实际手动速度≈{backMmS:F1}mm/s）。" +
                "可能被控制器内 RTSys/BASIC 任务周期性覆盖，请检查控制器里运行的程序是否也在设置 SPEED。");
        }
    }

    /// <summary>波形/运动结束后把运行期改掉的 Speed/Lspeed/Accel/Decel 复位到配置值（失败静默，下次手动运动前还会复位并校验）。</summary>
    private void RestoreRunParams(int axis)
    {
        ZMotionConfig z = _cfg.ZMotion;
        TryApi(() => zmcaux.ZAux_Direct_SetSpeed(_handle, axis, z.GetSpeed()));
        TryApi(() => zmcaux.ZAux_Direct_SetLspeed(_handle, axis, z.GetLspeed()));
        TryApi(() => zmcaux.ZAux_Direct_SetAccel(_handle, axis, z.GetAccel()));
        TryApi(() => zmcaux.ZAux_Direct_SetDecel(_handle, axis, z.GetDecel()));
    }

    /// <summary>动态应用软限位（连接后也可改）。</summary>
    public void ApplySoftLimits(float posLimit, float negLimit)
    {
        EnsureConnected();
        int axis = _cfg.ZMotion.GetAxisNumber();
        ThrowRc(zmcaux.ZAux_Direct_SetFsLimit(_handle, axis, posLimit), $"SetPosLimit({axis},{posLimit})");
        ThrowRc(zmcaux.ZAux_Direct_SetRsLimit(_handle, axis, negLimit), $"SetNegPosLimit({axis},{negLimit})");
        _cfg.ZMotion.SoftLimitPos.Value = posLimit;
        _cfg.ZMotion.SoftLimitNeg.Value = negLimit;
    }

    private void RunMove(int dir, string label)
    {
        EnsureConnected();
        ZMotionConfig z = _cfg.ZMotion;
        int axis = z.GetAxisNumber();
        _lastMoveDir = dir;   // 记录用户指令方向，供 LimitMonitorLoopAsync 判断是否继续冲向限位

        // (0) 防爆冲：运动前 Dpos 越界拦截（软限位之外再套一层，双保险）。
        // 手动前进/后退不受“软限位保护”开关影响，始终拦截（开关仅作用于五种波形运动模式）。
        float dpos = GetCurrentDpos();
        float posLim = _cfg.ZMotion.GetSoftLimitPos();
        float negLim = _cfg.ZMotion.GetSoftLimitNeg();
        if (dir > 0 && dpos >= posLim)
            throw new InvalidOperationException($"已到正向软限位 {posLim:F1}，无法{label}（当前 Dpos={dpos:F1}）");
        if (dir < 0 && dpos <= negLim)
            throw new InvalidOperationException($"已到负向软限位 {negLim:F1}，无法{label}（当前 Dpos={dpos:F1}）");

        // (0.5) 硬限位脱困（按时序重写）：①读限位触发位，同向直接拒绝；②反向先走离限位开关
        // （用 Mpos 反馈确认真实走开）；③走离后再清报警；④继续反向运动。不再用断使能重启。
        int hwStatus = ReadAxisStatus(axis);
        bool atPosHard = (hwStatus & 0x10) != 0, atNegHard = (hwStatus & 0x20) != 0;
        if (dir > 0 && atPosHard)
            throw new InvalidOperationException($"当前顶在正向硬限位上，无法{label}，请反向（后退）脱困");
        if (dir < 0 && atNegHard)
            throw new InvalidOperationException($"当前顶在负向硬限位上，无法{label}，请反向（前进）脱困");
        bool escaping = (dir > 0 && atNegHard) || (dir < 0 && atPosHard);
        if (escaping)
        {
            string escapeDiag = EscapeHardLimit(axis, dir, z);
            LimitTriggered?.Invoke(this, $"硬限位脱困（顶{(dir > 0 ? "负" : "正")}向限位→{label}）：{escapeDiag}");
        }

        // (1) Vmove 前快照
        int beforeIdle = 0;
        float beforeSpeed = 0, beforeDpos = 0, beforeMpos = 0;
        zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref beforeIdle);
        zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref beforeSpeed);
        zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref beforeDpos);
        zmcaux.ZAux_Direct_GetMpos(_handle, axis, ref beforeMpos);

        // (1.5) Vmove 跑的是控制器 SPEED 当前值，每次手动运动前复位为配置速度并校验，
        // 避免波形/找零残留速度导致“改配置速度没效果”
        ApplyRunSpeed(z, axis);

        // (2) 发指令
        ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir), $"Vmove({dir})");

        // (3) 等 300ms 让指令生效，再读状态
        Thread.Sleep(300);
        int afterIdle = 0;
        float afterSpeed = 0, afterDpos = 0, afterMpos = 0;
        zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref afterIdle);
        zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref afterSpeed);
        zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref afterDpos);
        zmcaux.ZAux_Direct_GetMpos(_handle, axis, ref afterMpos);

        // (4) 如果指令发了但完全没反应 → 说明有底层问题，把状态差异返回给上层
        if (beforeIdle == afterIdle && Math.Abs(afterSpeed) < 0.01f && Math.Abs(afterDpos - beforeDpos) < 0.01f)
        {
            // 轴状态字也读一下，看看有没有总线状态问题
            int status = 0, enable = 0;
            zmcaux.ZAux_Direct_GetAxisStatus(_handle, axis, ref status);
            zmcaux.ZAux_Direct_GetAxisEnable(_handle, axis, ref enable);
            throw new InvalidOperationException(
                $"{label} 指令已发出但电机无响应（rc=0）。\n" +
                $"轴{axis} 状态：使能={enable} 状态字=0x{status:X8}（{DescribeAxisStatus(status)}）\n" +
                $"IfIdle={afterIdle}(0=运行,1=停止) VpSpeed={afterSpeed:F1} Dpos={afterDpos:F1}\n" +
                $"→ 可能轴号不对 / RTSys 已占用 EtherCAT 轴 / 伺服未使能");
        }

    }

    /// <summary>
    /// 驱动器侧遥测：绕开控制器，通过 EtherCAT SDO 直读 CiA402 状态字 0x6041、标准故障码 0x603F、
    /// 厂商报警码 0x2600（汇川等），解码出驱动器当前卡在哪个状态机状态/挂着什么报警。
    /// 用于脱困失败时定位“控制器侧命令全部 rc=0 但电机不动”的驱动器内部原因。任何一步失败只记文本不抛错。
    /// </summary>
    private string ReadDriveTelemetry(int axis)
    {
        try
        {
            int slot = DriveNode(axis, out string slotSrc);

            string ReadObj(uint index, string name)
            {
                int v = 0;
                int rc = zmcaux.ZAux_BusCmd_SDORead(_handle, 0, (uint)slot, index, 0, 6, ref v);
                return rc != 0 ? $"{name}(0x{index:X4})读失败rc={rc} " : $"{name}=0x{v & 0xFFFF:X4} ";
            }

            int sw = 0;
            int rcSw = zmcaux.ZAux_BusCmd_SDORead(_handle, 0, (uint)slot, 0x6041, 0, 6, ref sw);
            if (rcSw != 0) return $"SDO读状态字失败rc={rcSw}（节点{slot}/{slotSrc}）——请截图驱动器面板报警码";

            string state = (sw & 0x004F) == 0x0000 ? "Not_ready"
                : (sw & 0x004F) == 0x0040 ? "Switch_on_disabled"
                : (sw & 0x006F) == 0x0023 ? "Ready_switch_on"
                : (sw & 0x006F) == 0x0027 ? "Switched_on"
                : (sw & 0x006F) == 0x002F ? "Operation_enabled"
                : (sw & 0x004D) == 0x0005 ? "Quick_stop_active"
                : (sw & 0x004F) == 0x000F ? "Fault_active"
                : $"未知(0x{sw:X4})";
            bool fault = (sw & 0x0008) != 0;
            bool quickStop = (sw & 0x0002) == 0;

            return $"节点{slot}({slotSrc}) 状态字0x6041=0x{sw & 0xFFFF:X4}→{state}"
                + (fault ? " 【FAULT置位=驱动器内部报警未复位，控制器侧DriveClear/断使能均无法清除，需故障复位或消除报警源】" : "")
                + (quickStop && !fault ? " 【快停位=处于Quick_Stop状态】" : "")
                + " | " + ReadObj(0x603F, "标准故障码")
                + " | " + ReadObj(0x2600, "厂商报警码");
        }
        catch (Exception ex)
        {
            return $"SDO遥测异常:{ex.Message}";
        }
    }

    /// <summary>轴号 → EtherCAT 节点号：优先读 AXIS_SLOT 轴参数，读不到退回常见映射 node=ax+1。</summary>
    private int DriveNode(int axis, out string src)
    {
        int slot = 0;
        src = "AXIS_SLOT";
        if (zmcaux.ZAux_Direct_GetVariableInt(_handle, $"AXIS_SLOT({axis})", ref slot) != 0)
        {
            slot = axis + 1;
            src = "ax+1推测";
        }
        return slot;
    }


    /// <summary>按 ZMC 轴状态字位定义解码：bit4/bit5=正/负硬限位，bit9/bit10=正/负软限位，bit11/bit22=驱动器报警。</summary>
    private static string DescribeAxisStatus(int axisstate)
    {
        if (axisstate == 0) return "正常";
        var parts = new List<string>();
        if (((axisstate >> 4) & 1) == 1) parts.Add("正向硬限位报警");
        if (((axisstate >> 5) & 1) == 1) parts.Add("负向硬限位报警");
        if (((axisstate >> 9) & 1) == 1) parts.Add("正向软限位");
        if (((axisstate >> 10) & 1) == 1) parts.Add("负向软限位");
        if (((axisstate >> 11) & 1) == 1) parts.Add("驱动器报警(bit11)");
        if (((axisstate >> 22) & 1) == 1) parts.Add("伺服报警(bit22)");
        // 其余置位的位原样列出，避免漏掉未知报警
        for (int b = 0; b < 32; b++)
        {
            if (b == 4 || b == 5 || b == 9 || b == 10 || b == 11 || b == 22) continue;
            if (((axisstate >> b) & 1) == 1) parts.Add($"bit{b}");
        }
        return parts.Count > 0 ? string.Join(", ", parts) : "正常";
    }

    public void Stop()
    {
        EnsureConnected();
        _lastMoveDir = 0;   // 用户主动停，清除方向标记
        ThrowRc(zmcaux.ZAux_Direct_Single_Cancel(_handle, _cfg.ZMotion.GetAxisNumber(), 2), "Cancel(2)");
    }

    /// <summary>
    /// 找零点（双硬限位对中）：本设备无原点开关，改为向负/正硬限位各撞一次、记录两端触发位置、
    /// 取中点作为机械零点，再把轴开回中点并置 Dpos/Mpos=0。因为以机械限位为绝对基准，
    /// 能抵消撞硬限位后皮带打滑/丢步造成的零点漂移。全程用速度模式 Vmove + 轮询，与 CSV 范式一致。
    /// 撞限位后的反向退出复用硬限位“救回”时序（先直接反向走、走不动才清报警重试，Mpos 反馈验证走离量）。阻塞式，须由上层放后台线程调用。
    /// </summary>
    public HomingResult? Home()
    {
        EnsureConnected();
        if (_wfRunning) throw new InvalidOperationException("波形运行中，请先停止波形再找零");
        if (_homing) throw new InvalidOperationException("找零正在进行中，请稍候");

        ZMotionConfig z = _cfg.ZMotion;
        int axis = z.GetAxisNumber();

        // mm ↔ user units（与 MainForm 同基准：1mm = gearDen/(10*lead*units) units）
        float MmU(float mm) => mm / 10f * z.GearDenominator.Value / z.Lead.Value / z.Units.Value;
        float U2Mm(float u) => u * 10f * z.Lead.Value * z.Units.Value / z.GearDenominator.Value;

        float creep = Math.Max(1f, z.HomeSeekSpeedMmS.Value);
        float cruise = Math.Max(creep + 1f, z.HomeCruiseSpeedMmS.Value);
        var p = new HomeParams
        {
            CreepUnits = MmU(creep),
            CreepAccel = MmU(Math.Max(1f, z.HomeSeekAccelMmS2.Value)),
            CruiseUnits = MmU(cruise),
            CruiseAccel = MmU(Math.Max(1f, z.HomeCruiseAccelMmS2.Value)),
            BackoffUnits = MmU(Math.Max(0f, z.HomeBackoffMm.Value)),
            TolUnits = MmU(Math.Max(0.05f, z.HomeCenterTolMm.Value)),
            CreepZoneUnits = MmU(Math.Max(0.5f, z.HomeCreepZoneMm.Value)),
            MaxTravelUnits = MmU(Math.Max(10f, z.HomeMaxTravelMm.Value)),
            TimeoutMs = Math.Max(1000, z.HomePhaseTimeoutMs.Value),
        };

        float origSoftPos = z.SoftLimitPos.Value, origSoftNeg = z.SoftLimitNeg.Value;
        _homing = true;
        _lastMoveDir = 0;
        try
        {
            // 临时放开软限位（否则软限位会先于硬限位拦住，到不了限位）。速度/加减速由各阶段内部 SetHomeMotion 设定。
            TryApi(() => zmcaux.ZAux_Direct_SetFsLimit(_handle, axis, 1e9f));
            TryApi(() => zmcaux.ZAux_Direct_SetRsLimit(_handle, axis, -1e9f));

            // 1) 先清一次错误态（万一当前正压在限位上）
            ClearAxisError(axis);

            // 2) 撞负限位→退出；3) 撞正限位→退出（高速巡航撞限位后低速精确再触发，退出复用硬限位救回时序）
            float pNeg = SeekHardLimit(axis, -1, p, z);
            RetreatFromLimit(axis, +1, p, z);
            float pPos = SeekHardLimit(axis, +1, p, z);
            RetreatFromLimit(axis, -1, p, z);

            // 4) 中点 = 两端触发位的平均；巡航快回、末段进入低速精确停靠
            float center = (pNeg + pPos) / 2f;
            DriveToDpos(axis, center, p, twoSpeed: true);

            // 5) 中点定义为零点（Mpos+Dpos 同步清零）
            TryApi(() => zmcaux.ZAux_Direct_SetMpos(_handle, axis, 0f));
            ThrowRc(zmcaux.ZAux_Direct_SetDpos(_handle, axis, 0f), $"SetDpos({axis}) 对中置零");

            // 把实测两端与中点换算为 mm（相对旧零点）返回；CenterMm 即本次纠正掉的漂移量
            return new HomingResult(U2Mm(pNeg), U2Mm(pPos), U2Mm(center));
        }
        finally
        {
            TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
            _lastMoveDir = 0;
            // 恢复运行速度/加减速度与软限位
            TryApi(() => zmcaux.ZAux_Direct_SetSpeed(_handle, axis, z.GetSpeed()));
            TryApi(() => zmcaux.ZAux_Direct_SetAccel(_handle, axis, z.GetAccel()));
            TryApi(() => zmcaux.ZAux_Direct_SetDecel(_handle, axis, z.GetDecel()));
            TryApi(() => zmcaux.ZAux_Direct_SetFsLimit(_handle, axis, origSoftPos));
            TryApi(() => zmcaux.ZAux_Direct_SetRsLimit(_handle, axis, origSoftNeg));
            _homing = false;
        }
    }

    /// <summary>
    /// 清总线伺服轴报警：DriveClear 当前告警(0)+外部输入告警(2)。只清报警，不做 Datum/不断使能。
    /// 新脱困时序下报警清除安排在“走离限位之后”——压住限位期间清了会被驱动器立刻重触发。
    /// </summary>
    private void ClearAxisError(int axis)
    {
        TryApi(() => zmcaux.ZAux_BusCmd_DriveClear(_handle, (uint)axis, 0));
        TryApi(() => zmcaux.ZAux_BusCmd_DriveClear(_handle, (uint)axis, 2));
    }
    
    

    /// <summary>
    /// 硬限位脱困（反向走离，持续重试直到脱离）：①Cancel(2) 撤销挂起旧指令（防爆冲）；②发一次反向 Vmove；
    /// ③每隔 50ms 轮询“当前是否仍是硬限位状态”（读 AXISSTATUS 限位实时位），若仍在限位且本轮 Mpos 没走动
    /// （驱动器报警锁着拒绝执行，真机实证：顶负限位前进时 AXISSTATUS=0x820，bit11 报警挂着，Mpos 仅动 -8 units），
    /// 则每隔 retryIntervalMs 重试一次反向走：Cancel+DriveClear(0/2) 清报警并等 bit11 清零→重发反向 Vmove；
    /// ④直到硬限位实时位清零（真正脱离）才停止检测并返回，上层继续同向运动。
    /// 全程不断使能、不用 Datum。反复重试到超时仍未脱离则附 SDO 遥测定位驱动器侧原因。
    /// 复位找零的“退出限位”也复用本逻辑（传入低速爬行速度/找零超时），确保与手动脱困同一套检测与反向移动逻辑。
    /// </summary>
    private string EscapeHardLimit(int axis, int dir, ZMotionConfig z, float targetUnits = 0f,
        float? speedUnits = null, float? accelUnits = null, int? timeoutMsOverride = null)
    {
        float MmU(float mm) => mm / 10f * z.GearDenominator.Value / z.Lead.Value / z.Units.Value;
        if (targetUnits <= 0f)
        {
            targetUnits = MmU(5f);   // 默认走离 5mm，足够松开限位开关
        }
        int limitBit = dir > 0 ? 0x20 : 0x10;   // 要脱离的那个限位触发位
        // 找零退出用低速爬行速度（speedUnits 传入），超时相应放大；手动/归零脱困用配置速度。
        float effSpeed = speedUnits ?? Math.Max(1f, z.GetSpeed());
        int timeoutMs = timeoutMsOverride
            ?? Math.Min(3000, Math.Max(1500, (int)(targetUnits / effSpeed * 1500) + 800));
        var sb = new StringBuilder();
        var sw = System.Diagnostics.Stopwatch.StartNew();

        // ① 撤销挂起旧指令，防止恢复后先执行旧方向运动（“点后退先向前冲”根因）
        CancelPending(axis);
        // 找零阶段按传入低速设定（SetHomeMotion 带 GetSpeed 回读校验）；手动/归零脱困用配置速度。
        if (speedUnits.HasValue) SetHomeMotion(axis, speedUnits.Value, accelUnits ?? speedUnits.Value);
        else ApplyRunSpeed(z, axis);

        // ② 反向走离：发一次反向 Vmove 后，每隔一段时间（50ms 轮询）检测当前是否仍是硬限位状态；
        //    只要还在硬限位且本轮没走动，就重试反向走（Cancel→清报警→等报警位清零→重发反向 Vmove），
        //    直到硬限位实时位清零（真正脱离）才停止检测。EtherCAT 限位位偶尔不复位则用走离目标量兼底。
        float anchor = ReadMpos(axis);            // 走离累计基准（全程不重置，用于进度/兼底判定）
        float attemptStart = anchor;              // 本轮重试起点（判断“这次有没有真的在走”）
        int retryIntervalMs = Math.Min(1000, Math.Max(400, timeoutMs / 6));   // 每隔此时间仍未走开就重试反向走
        int lastRetryAt = 0;
        int attempt = 0;
        bool cleared = false;
        ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir), $"Escape.Vmove({dir})");
        while (sw.ElapsedMilliseconds < timeoutMs)
        {
            Thread.Sleep(50);
            float mpos = ReadMpos(axis);
            float movedTotal = (mpos - anchor) * dir;          // 累计走离量（正=真实走开）
            float movedThis = (mpos - attemptStart) * dir;     // 本轮走离量（用于判“这次动不动”）
            bool stillOnLimit = (ReadAxisStatus(axis) & limitBit) != 0;   // 当前是否仍是硬限位状态（实时位）

            // 脱离硬限位状态（实时位清零）且确实走开 → 停止检测，完成
            if (!stillOnLimit && movedTotal >= 1f)
            {
                CancelPending(axis);
                if (!cleared) ClearAxisError(axis);
                sb.Append($"走离{movedTotal:F0}units(目标{targetUnits:F0})，耗时{sw.ElapsedMilliseconds}ms");
                if (cleared) sb.Append($"（清报警重试{attempt}次后走开）");
                return sb.ToString();
            }
            // 走过目标量但限位位仍置位（EtherCAT 位可能不复位）：按已离开开关收尾
            if (movedTotal >= targetUnits)
            {
                CancelPending(axis);
                if (!cleared) ClearAxisError(axis);
                sb.Append($"走离{movedTotal:F0}units(目标已达，限位位仍置位按已离开处理)，耗时{sw.ElapsedMilliseconds}ms");
                return sb.ToString();
            }
            // 仍是硬限位状态且本轮没走动 → 每隔 retryIntervalMs 重试反向走：Cancel→清报警→等报警位清零→重发反向 Vmove
            if (movedThis < 1f && sw.ElapsedMilliseconds - lastRetryAt >= retryIntervalMs)
            {
                lastRetryAt = (int)sw.ElapsedMilliseconds;
                attempt++;
                CancelPending(axis);
                ClearAxisError(axis);
                int waitMs = WaitForAlarmClear(axis, 1500);
                sb.Append($"[{sw.ElapsedMilliseconds}ms仍在{(dir > 0 ? "负" : "正")}向限位未走开(Mpos仅{movedTotal:F0})，第{attempt}次清报警等{waitMs}ms重试反向走; ");
                cleared = true;
                attemptStart = ReadMpos(axis);
                ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir), $"Escape.Vmove({dir}) retry{attempt}");
            }
        }
        // ③ 反复重试到超时仍未脱离硬限位状态：停下并附驱动器侧 SDO 遥测定位原因
        CancelPending(axis);
        _lastMoveDir = 0;
        int stopReason = 0;
        zmcaux.ZAux_Direct_GetAxisStopReason(_handle, axis, ref stopReason);
        int stNow = ReadAxisStatus(axis);
        float finalMoved = (ReadMpos(axis) - anchor) * dir;
        throw new InvalidOperationException(
            $"硬限位脱困失败（已反复重试反向走离 {attempt} 次，直至硬限位状态仍未解除）：反馈位置仅动 {finalMoved:F0} units（目标 {targetUnits:F0}）。\n" +
            $"过程：{sb}\n" +
            $"AXISSTATUS=0x{stNow:X8}（{DescribeAxisStatus(stNow)}） AXISSTOPREASON=0x{stopReason:X8}\n" +
            $"驱动器侧：{ReadDriveTelemetry(axis)}\n" +
            "可能原因：① 限位信号进了驱动器本体 DI，驱动器在信号消失前拒绝一切运动——断电手动挪离开关或在驱动器参数屏蔽 DI 限位；"
            + "② 驱动器内部报警需厂商复位方式（按驱动器面板故障码查手册）。");
    }

    /// <summary>轮询等待 AXISSTATUS 驱动器报警位 bit11(0x800)/bit22(0x400000) 清零，100ms/拍，返回实际等待毫秒数（超时不报错，由上层用 Mpos 验证）。</summary>
    private int WaitForAlarmClear(int axis, int timeoutMs)
    {
        int waited = 0;
        while (waited < timeoutMs)
        {
            int st = ReadAxisStatus(axis);
            if ((st & 0x800) == 0 && ((st >> 22) & 1) == 0)
                return waited;
            Thread.Sleep(100);
            waited += 100;
        }
        return waited;
    }

    /// <summary>撤销挂起运动指令（imode=2 减速停止）；失败不抛错（静止时部分固件返回非 0 属正常）。</summary>
    private void CancelPending(int axis) =>
        TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));

    /// <summary>硬限位触发位：dir&lt;0 看 AXISSTATUS bit5(负限位 0x20)；dir&gt;0 看 bit4(正限位 0x10)。</summary>
    private static bool LimitBit(int status, int dir) =>
        dir < 0 ? ((status >> 5) & 1) == 1 : ((status >> 4) & 1) == 1;

    private int ReadAxisStatus(int axis) { int s = 0; zmcaux.ZAux_Direct_GetAxisStatus(_handle, axis, ref s); return s; }
    private float ReadDpos(int axis) { float d = 0; zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref d); return d; }
    private float ReadMpos(int axis) { float m = 0; zmcaux.ZAux_Direct_GetMpos(_handle, axis, ref m); return m; }

    /// <summary>dir 方向硬限位“当前是否有效”（读 AXISSTATUS 实时位，不用 AXISSTOPREASON——后者是锁存历史会误判）。</summary>
    private bool LimitLive(int axis, int dir) => LimitBit(ReadAxisStatus(axis), dir);

    /// <summary>读轴状态快照（使能/±软限位/±硬限位/伺服报警），供界面状态灯高频轮询；单次读失败按未触发处理。</summary>
    public AxisStatusInfo ReadAxisStatusInfo()
    {
        EnsureConnected();
        ZMotionConfig z = _cfg.ZMotion;
        int axis = z.GetAxisNumber();
        int enable = 0;
        zmcaux.ZAux_Direct_GetAxisEnable(_handle, axis, ref enable);
        int status = ReadAxisStatus(axis);
        float dpos = 0;
        zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos);

        // 软限位：AXISSTATUS bit9(0x200)/bit10(0x400) 为触发位，另按 Dpos 越界兜底（停止后位可能不保持）
        bool posSoft = (status & 0x200) != 0 || dpos >= z.GetSoftLimitPos();
        bool negSoft = (status & 0x400) != 0 || dpos <= z.GetSoftLimitNeg();
        return new AxisStatusInfo(
            Enabled: enable != 0,
            PosSoftLimit: posSoft,
            NegSoftLimit: negSoft,
            PosHardLimit: (status & 0x10) != 0,
            NegHardLimit: (status & 0x20) != 0,
            ServoAlarm: (status & 0x800) != 0 || ((status >> 22) & 1) == 1);   // bit11(真机报警位)/bit22 任一
    }

    /// <summary>找零各阶段的速度/加减速/几何参数（均为 user units 或 ms），集中传递避免参数爆炸。</summary>
    private struct HomeParams
    {
        public float CreepUnits, CreepAccel, CruiseUnits, CruiseAccel;
        public float BackoffUnits, TolUnits, CreepZoneUnits, MaxTravelUnits;
        public int TimeoutMs;
    }

    /// <summary>
    /// 设置找零速度/加减速，并用 GetSpeed 回读校验：不符重试一次仍不行就报错。
    /// 之前 SetSpeed 用 TryApi 吞掉失败，导致轴仍按默认 Speed(≈1000mm/s) 高速猛撞——这里杜绝。
    /// </summary>
    private void SetHomeMotion(int axis, float speedUnits, float accelUnits)
    {
        float back = 0;
        for (int i = 0; i < 2; i++)
        {
            TryApi(() => zmcaux.ZAux_Direct_SetAccel(_handle, axis, accelUnits));
            TryApi(() => zmcaux.ZAux_Direct_SetDecel(_handle, axis, accelUnits));
            ThrowRc(zmcaux.ZAux_Direct_SetSpeed(_handle, axis, speedUnits), $"Home.SetSpeed({speedUnits:F0})");
            back = 0;
            if (zmcaux.ZAux_Direct_GetSpeed(_handle, axis, ref back) == 0 &&
                Math.Abs(back - speedUnits) <= Math.Max(2f, speedUnits * 0.05f))
                return;
            Thread.Sleep(50);
        }
        throw new InvalidOperationException(
            $"找零设速失败：请求 SPEED={speedUnits:F0}，回读={back:F0}。轴可能仍在报警/被 RTSys 占用而无法改参数。");
    }

    /// <summary>
    /// 朝 dir 走，等到“该方向硬限位实时有效”或“已顶住(仍朝限位发运动但轴几乎不动=堵转)”任一即返回 true。
    /// 若 maxTravel 内一路畅通、既没限位信号也没堵转，判限位没接返回 false。带起步宽限期避免把加速初段误判成堵转。
    /// </summary>
    private bool WaitHitOrStall(int axis, int dir, float maxTravel, float start, HomeParams p)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        float last = start;
        int stallPolls = 0;
        const int pollMs = 30, graceMs = 700, stallTripPolls = 14;   // 连续约 420ms 不动 = 顶住
        float eps = Math.Max(15f, p.TolUnits * 0.15f);               // 单拍最小期望位移
        while (sw.ElapsedMilliseconds < p.TimeoutMs)
        {
            Thread.Sleep(pollMs);
            if (LimitLive(axis, dir)) return true;                   // 限位信号实时有效
            float d = ReadDpos(axis);
            if (Math.Abs(d - start) > maxTravel) return false;       // 走超最大行程仍未停 = 限位没接
            if (Math.Abs(d - last) < eps)
            {
                if (sw.ElapsedMilliseconds > graceMs && ++stallPolls >= stallTripPolls)
                    return true;                                     // 指令朝限位却不动 = 已顶到（控制器/驱动器已 stop）
            }
            else stallPolls = 0;
            last = d;
        }
        return false;
    }

    /// <summary>
    /// 朝 dir 撞硬限位并返回触发位 Dpos。先判“是否已压在该限位上”（开机就压着的情况直接取当前位置，
    /// 绝不再朝限位方向发运动——修复“已压正限位还往正走”）；否则两段式：高速巡航撞限位→退出→低速精确再撞取干净点。
    /// </summary>
    private float SeekHardLimit(int axis, int dir, HomeParams p, ZMotionConfig z)
    {
        ClearAxisError(axis);

        // 0) 已经压在该方向限位上：直接以当前位置为触发点，不再朝该方向发运动
        if (LimitLive(axis, dir))
            return ReadDpos(axis);

        // 1) 高速巡航撞限位
        SetHomeMotion(axis, p.CruiseUnits, p.CruiseAccel);
        float start = ReadDpos(axis);
        ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir), $"Home.Vmove({dir}) cruise");
        if (!WaitHitOrStall(axis, dir, p.MaxTravelUnits, start, p))
        {
            TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
            throw new InvalidOperationException(
                $"找零：向{(dir < 0 ? "负" : "正")}向巡航一路畅通，在超时/最大行程内都没顶到硬限位，疑限位未接/未映射(FWD_IN/REV_IN)或电平反(INVERT_IN)，已急停。");
        }
        TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));

        // 2) 退出限位开关一点，为低速精确再触发做准备（复用硬限位救回时序）
        RetreatFromLimit(axis, -dir, p, z);

        // 3) 低速精确再撞一次取干净触发点（退出后若已实时压住则直接用当前位置）
        ClearAxisError(axis);
        if (!LimitLive(axis, dir))
        {
            SetHomeMotion(axis, p.CreepUnits, p.CreepAccel);
            float cstart = ReadDpos(axis);
            ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir), $"Home.Vmove({dir}) creep");
            float creepMax = Math.Max(p.BackoffUnits * 6f + p.CreepZoneUnits * 3f, 20f);
            if (!WaitHitOrStall(axis, dir, creepMax, cstart, p))
            {
                TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
                throw new InvalidOperationException(
                    $"找零：低速精确段向{(dir < 0 ? "负" : "正")}向未再次顶到限位（退出后应很近），请检查限位回差/抖动。");
            }
            TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
        }
        return ReadDpos(axis);
    }

    /// <summary>
    /// 从限位朝 escapeDir 退出（复位找零的“反向移动”）：复用硬限位“救回”时序 EscapeHardLimit——
    /// 碰触限位后先直接反向走，走不动（Mpos 不跟随=报警锁着）才清报警重试，用 Mpos 真实反馈验证走离量；
    /// 退出用低速爬行速度、找零阶段超时。检测走 AXISSTATUS 限位实时位，与手动/归零脱困同一套逻辑。
    /// </summary>
    private void RetreatFromLimit(int axis, int escapeDir, HomeParams p, ZMotionConfig z)
    {
        float need = p.BackoffUnits + 3f * Math.Max(p.TolUnits, 1f);
        EscapeHardLimit(axis, escapeDir, z, need, p.CreepUnits, p.CreepAccel, p.TimeoutMs);
    }

    /// <summary>将 Dpos 驱动到 target(±tol)；仅在方向变化时重发 Vmove，到位即 Cancel。
    /// twoSpeed=true 时先巡航快跑、进入 creepZone 后转低速精确停靠；false 全程低速。</summary>
    private void DriveToDpos(int axis, float target, HomeParams p, bool twoSpeed)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        int dir = 0; bool creeping = false;
        SetHomeMotion(axis, twoSpeed ? p.CruiseUnits : p.CreepUnits, twoSpeed ? p.CruiseAccel : p.CreepAccel);
        float stopTol = Math.Max(p.TolUnits, 1f);
        while (sw.ElapsedMilliseconds < p.TimeoutMs)
        {
            float diff = target - ReadDpos(axis);
            if (Math.Abs(diff) <= stopTol) break;
            int want = diff > 0 ? 1 : -1;
            if (twoSpeed && !creeping && Math.Abs(diff) <= p.CreepZoneUnits)
            {
                creeping = true;
                SetHomeMotion(axis, p.CreepUnits, p.CreepAccel);
            }
            if (want != dir)
            {
                ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, want), $"Home.Vmove({want}) drive");
                dir = want;
            }
            Thread.Sleep(30);
        }
        TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
    }

    // ==================== 运动波形：CSV 逐拍下发 ====================

    /// <summary>
    /// 启动波形：以速度模式逐拍下发生成器产生的带符号速度。
    /// 调用前 UI 应已完成 WaveformValidator 时域校验。运行期把 Lspeed 调低、
    /// Accel/Decel 设为加速度上限，以尽量逼近理想波形（减轻过零换向削顶）。
    /// </summary>
    public void StartWaveform(WaveformRuntime runtime)
    {
        EnsureConnected();
        if (_wfRunning) throw new InvalidOperationException("波形已在运行中");
        ArgumentNullException.ThrowIfNull(runtime);
        if (_wfLastDir != 0) _wfLastDir = 0;

        _wf = runtime;
        _wfUnits = new MotionUnits(_cfg.ZMotion);
        int axis = _cfg.ZMotion.GetAxisNumber();
        MotionUnits mu = _wfUnits;
        double accelLim = runtime.Limits.AccelLimitMmS2 > 0 ? runtime.Limits.AccelLimitMmS2 : 5000.0;
        // 减速度：阶梯型模式（方波/脉冲/PRTS）从各自页签读取，<=0 时回退用加速度上限（与原行为一致）。
        double decelLim = runtime.Limits.DecelLimitMmS2 > 0 ? runtime.Limits.DecelLimitMmS2 : accelLim;
        
        // 运行期参数（失败不阻断，下一拍靠 Cancel 兼容）
        TryApi(() => zmcaux.ZAux_Direct_SetLspeed(_handle, axis, mu.MmSToUnitsPS(1.0)));
        TryApi(() => zmcaux.ZAux_Direct_SetAccel(_handle, axis, mu.MmS2ToUnitsPS2(accelLim)));
        TryApi(() => zmcaux.ZAux_Direct_SetDecel(_handle, axis, mu.MmS2ToUnitsPS2(decelLim)));

        runtime.Generator.Reset();
        _wfCts = new CancellationTokenSource();
        _wfRunning = true;
        var token = _wfCts.Token;
        _ = Task.Run(() => WaveformLoopAsync(runtime, axis, token), token);
    }

    private async Task WaveformLoopAsync(WaveformRuntime rt, int axis, CancellationToken ct)
    {
        MotionUnits mu = _wfUnits!;
        IWaveformGenerator gen = rt.Generator;
        var sw = System.Diagnostics.Stopwatch.StartNew();
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(rt.DtSec));
        bool enforceSoft = _cfg.Waveform.GetEnforceSoftLimit();
        WaveformStopReason reason = WaveformStopReason.Completed;
        try
        {
            // 软限位保护关闭：运行期临时放开控制器 FSLIMIT/RSLIMIT，
            // 否则 CSV 速度指令会被控制器在限位处停下，无法坚持把运动做完（硬限位不受影响）。
            if (!enforceSoft)
            {
                TryApi(() => zmcaux.ZAux_Direct_SetFsLimit(_handle, axis, 1e9f));
                TryApi(() => zmcaux.ZAux_Direct_SetRsLimit(_handle, axis, -1e9f));
            }
            while (!ct.IsCancellationRequested)
            {
                await timer.WaitForNextTickAsync(ct).ConfigureAwait(false);
                double t = sw.Elapsed.TotalSeconds;

                float dpos = 0;
                zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos);
                double posMm = mu.UnitsToMm(dpos);

                // 软限位保护开启：逐拍监测，触碰限位且仍朝限位方向 → 停止整个波形
                if (enforceSoft)
                {
                    float posLim = _cfg.ZMotion.GetSoftLimitPos();
                    float negLim = _cfg.ZMotion.GetSoftLimitNeg();
                    if ((dpos >= posLim && _wfLastDir > 0) || (dpos <= negLim && _wfLastDir < 0))
                    {
                        reason = WaveformStopReason.SoftLimit;
                        break;
                    }
                }

                double v = gen.NextVelocity(t, posMm);

                _lastMoveDir = Math.Sign(v);      // 让 LimitMonitorLoopAsync 的方向拦截生效
                DispatchSpeed(axis, mu, v);

                WaveformTick?.Invoke(this, new WaveformTickEventArgs(t, v, posMm));

                if (gen.IsFinished(t)) break;
            }
        }
        catch (OperationCanceledException) { reason = WaveformStopReason.UserStopped; }
        finally
        {
            TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
            _lastMoveDir = 0;
            _wfLastDir = 0;
            _wfRunning = false;
            // 先恢复软限位再让轴停稳：若波形结束时仍处在临时放开的越界区，
            // 限位窗口收紧到当前位置附近可避免控制器报“超出软限位”锁存错
            if (!enforceSoft)
            {
                TryApi(() => zmcaux.ZAux_Direct_SetFsLimit(_handle, axis, _cfg.ZMotion.GetSoftLimitPos()));
                TryApi(() => zmcaux.ZAux_Direct_SetRsLimit(_handle, axis, _cfg.ZMotion.GetSoftLimitNeg()));
            }
            // 波形运行期把 Speed/Lspeed/Accel/Decel 改成逐拍值，结束必须复位到配置值，
            // 否则之后前进/后退/归零会带着波形残留速度跑
            if (_connected && _handle != IntPtr.Zero)
                RestoreRunParams(axis);
            // 上报停止原因：供 UI 复位“启动波形”按钮；触碰软限位时额外弹一次确定提示
            if (reason == WaveformStopReason.SoftLimit)
                LimitTriggered?.Invoke(this, "触碰软限位，运动已停止");
            WaveformStopped?.Invoke(this, reason);
        }
    }

    /// <summary>将带符号速度(mm/s)换算为 CSV 下发：死区内减速停；过零换向时重发 Vmove；否则实时改 Speed。</summary>
    private void DispatchSpeed(int axis, MotionUnits mu, double vMmS)
    {
        const double eps = 0.5;   // mm/s 死区，避免频繁换向
        if (Math.Abs(vMmS) < eps)
        {
            if (_wfLastDir != 0)
            {
                TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2));
                _wfLastDir = 0;
            }
            return;
        }

        int dir = Math.Sign(vMmS);
        if (dir != _wfLastDir)
        {
            TryApi(() => zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir));
            _wfLastDir = dir;
        }
        TryApi(() => zmcaux.ZAux_Direct_SetSpeed(_handle, axis, mu.MmSToUnitsPS(Math.Abs(vMmS))));
    }

    public void StopWaveform()
    {
        _wfCts?.Cancel();
        _wfCts?.Dispose();
        _wfCts = null;
        if (_connected && _wf != null)
            TryApi(() => zmcaux.ZAux_Direct_Single_Cancel(_handle, _cfg.ZMotion.GetAxisNumber(), 2));
        _wfRunning = false;
        _wfLastDir = 0;
        _lastMoveDir = 0;
    }

    private static void TryApi(Func<int> call) { try { call(); } catch { /* 单次下发失败不影响下一拍 */ } }

    /// <summary>扫描所有 AIN 通道，返回每个 ionum 的当前值（未连接的通道为 null）。</summary>
    public (int ionum, float? value)[] ScanAIN(int maxChannels = 32)
    {
        EnsureConnected();
        var result = new (int, float?)[maxChannels];
        for (int i = 0; i < maxChannels; i++)
        {
            float v = 0;
            int rc = zmcaux.ZAux_Direct_GetAD(_handle, i, ref v);
            result[i] = (i, rc == 0 ? v : null);
        }
        return result;
    }

    /// <summary>枚举 EtherCAT 总线节点（注意：不调 InitBus，避免干扰 RTSys 已初始化的总线）。</summary>
    public (int node, uint vendor, uint device, byte inCnt, byte outCnt, byte ainCnt, byte aoutCnt)[] ScanBusNodes()
    {
        EnsureConnected();

        // 直接 GetNodeNum — RTSys 已经初始化过总线，不要二次 InitBus
        int nodeNum = 0;
        int rc = zmcaux.ZAux_BusCmd_GetNodeNum(_handle, 0, ref nodeNum);
        if (rc != 0 || nodeNum <= 0)
            return Array.Empty<(int, uint, uint, byte, byte, byte, byte)>();

        var list = new List<(int, uint, uint, byte, byte, byte, byte)>();
        for (int node = 0; node < nodeNum; node++)
        {
            int vendor = 0, device = 0;
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 0, ref vendor);
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 1, ref device);
            int inC = 0, outC = 0, ainC = 0, aoutC = 0;
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 10, ref inC);
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 11, ref outC);
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 12, ref ainC);
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 13, ref aoutC);
            list.Add((node, (uint)vendor, (uint)device, (byte)inC, (byte)outC, (byte)ainC, (byte)aoutC));
        }
        return list.ToArray();
    }

    /// <summary>终极诊断：扫轴 0~10，把每个轴的状态字/使能/位置/速度都打出来，帮定位哪个轴真的在工作。</summary>
    public string ScanAllAxes(int max = 11)
    {
        EnsureConnected();
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"=== 扫轴 0~{max - 1} ===");
        sb.AppendLine($"{"Axis",-5}{"Enable",-8}{"Idle",-8}{"Status",-12}{"Dpos",-12}{"VpSpeed",-12}");
        sb.AppendLine(new string('-', 65));

        for (int axis = 0; axis < max; axis++)
        {
            int enable = 0, idle = 0, status = 0;
            float dpos = 0, speed = 0;
            int rc1 = zmcaux.ZAux_Direct_GetAxisEnable(_handle, axis, ref enable);
            int rc2 = zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref idle);
            int rc3 = zmcaux.ZAux_Direct_GetAxisStatus(_handle, axis, ref status);
            int rc4 = zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos);
            int rc5 = zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref speed);

            // 只打"有意义"的轴（非 0 状态字 / 已使能 / 有速度）或前 4 个
            bool interesting = (status != 0 || enable != 0 || Math.Abs(speed) > 0.01f);
            if (interesting || axis < 4)
            {
                string tag = interesting ? " ← 活跃" : "";
                sb.AppendLine($"{axis,-5}{enable,-8}{idle,-8}0x{status:X8}{dpos,-12:F1}{speed,-12:F1}{tag}");
            }
        }
        return sb.ToString();
    }

    // ---------- BASIC 命令封装（RTSys 模式下必须走 BASIC 解释器） ----------

    /// <summary>
    /// 通过 ZAux_Execute 发一条 BASIC 命令到控制器，返回命令输出的字符串。
    /// 
    /// 注意：正运动 SDK 里有两个"在线命令"入口：
    ///   ZAux_DirectCommand —— 底层直接命令，不经过 BASIC 解释器，RTSys 模式下无法调用
    ///                         NODE_AIO / AIN / DRIVE_* 等 BASIC 函数
    ///   ZAux_Execute       —— 经过 BASIC 解释器执行，能调用所有 BASIC 函数
    /// 
    /// RTSys 模式下必须用 ZAux_Execute 才能读 NODE_AIO、AIN、USER_VAR 等。
    /// 例：BasicCmd("?AIN(3)") → "2.000"；BasicCmd("?NODE_AIO(0,1,0)") → "8"
    /// </summary>
    public string BasicCmd(string cmd, int bufSize = 1024)
    {
        EnsureConnected();
        var sb = new System.Text.StringBuilder(bufSize);
        int rc = zmcaux.ZAux_Execute(_handle, cmd, sb, (uint)(bufSize - 1));
        if (rc != 0) throw new InvalidOperationException($"ZAux_Execute(\"{cmd}\") 失败 rc={rc}");
        return sb.ToString().Trim();
    }

    /// <summary>
    /// 用 DirectCommand 读取全局 AIN 编号 n 的模拟量值。
    /// 前提：控制器（RTSys）已经配置好 NODE_AIO，让总线模块的 AIN 映射到全局编号空间。
    /// </summary>
    public float ReadAIN(int n)
    {
        EnsureConnected();
        string resp = BasicCmd($"?AIN({n})");
        if (!float.TryParse(resp, System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture, out float v))
        {
            throw new InvalidOperationException($"AIN({n}) 返回 \"{resp}\" 无法解析为数值");
        }
        return v;
    }

    /// <summary>
    /// 批量读取 AIN startIndex..startIndex+count-1，返回每个通道的（编号, 值）元组。
    /// 解析失败的通道 value 为 null。
    /// </summary>
    public (int index, float? value)[] ReadAINBatch(int startIndex, int count)
    {
        var result = new (int, float?)[count];
        for (int i = 0; i < count; i++)
        {
            int idx = startIndex + i;
            try
            {
                result[i] = (idx, ReadAIN(idx));
            }
            catch
            {
                result[i] = (idx, null);
            }
        }
        return result;
    }

    /// <summary>
    /// 诊断：读取每个 EtherCAT 节点的 NODE_AIO 配置（AIN 起始编号）和 AIN 数量。
    /// 返回值里 cmdAin/cmdAout 是 DirectCommand 的原始响应，方便定位 BAS 命令是否成功。
    /// 没配 NODE_AIO 或命令执行失败的节点 ainBase 返回 -1。
    /// </summary>
    public (int node, int ainBase, int ainCount, int aoutBase, int aoutCount, string cmdAin, string cmdAout)[] ReadNodeAIOMap()
    {
        EnsureConnected();
        int nodeNum = 0;
        zmcaux.ZAux_BusCmd_GetNodeNum(_handle, 0, ref nodeNum);
        if (nodeNum <= 0) return Array.Empty<(int, int, int, int, int, string, string)>();

        var list = new List<(int, int, int, int, int, string, string)>();
        for (int node = 0; node < nodeNum; node++)
        {
            int ainBase = -1, ainCnt = 0, aoutBase = -1, aoutCnt = 0;
            string cmdAin = "";
            string cmdAout = "";
            try
            {
                cmdAin = BasicCmd($"?NODE_AIO(0,{node},0)");
                int.TryParse(cmdAin, out ainBase);
            }
            catch (Exception ex) { cmdAin = $"ERR: {ex.Message}"; }
            try
            {
                cmdAout = BasicCmd($"?NODE_AIO(0,{node},3)");
                int.TryParse(cmdAout, out aoutBase);
            }
            catch (Exception ex) { cmdAout = $"ERR: {ex.Message}"; }

            // NODE_INFO sel=12 是 ain 个数，sel=13 是 aout 个数
            int n = 0;
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 12, ref n);
            ainCnt = n; n = 0;
            zmcaux.ZAux_BusCmd_GetNodeInfo(_handle, 0, (uint)node, 13, ref n);
            aoutCnt = n;

            list.Add((node, ainBase, ainCnt, aoutBase, aoutCnt, cmdAin, cmdAout));
        }
        return list.ToArray();
    }

    /// <summary>诊断指定轴：使能状态、运动状态、位置、速度。</summary>
    public string DumpAxisInfo(int axis)
    {
        EnsureConnected();
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"=== 轴 {axis} 诊断 ===");

        // 使能状态
        int enable = 0;
        sb.AppendLine($"使能(SetAxisEnable=1): rc={zmcaux.ZAux_Direct_GetAxisEnable(_handle, axis, ref enable)} value={enable}");

        // 运动状态（0=运行中，1=停止）
        int idle = 0;
        sb.AppendLine($"运动状态(GetIfIdle):    rc={zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref idle)} value={(idle == 0 ? "运行中" : "停止")}");

        // 轴状态字
        int status = 0;
        sb.AppendLine($"轴状态(GetAxisStatus): rc={zmcaux.ZAux_Direct_GetAxisStatus(_handle, axis, ref status)} value=0x{status:X8}");

        // 当前位置
        float dpos = 0;
        sb.AppendLine($"位置(GetDpos):          rc={zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos)} value={dpos:F3}");

        // 当前规划速度
        float speed = 0;
        sb.AppendLine($"速度(GetVpSpeed):       rc={zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref speed)} value={speed:F3}");

        return sb.ToString();
    }

    /// <summary>
    /// 传感器轮询：用 ZAux_Direct_GetAD 读全局 AIN 编号 n = AIN_Start + 0..3。
    /// AIN 起始编号来自 config（默认 0），如果总线模块配好了 NODE_AIO，
    /// 总线 AIN 会映射到该编号之后。
    /// </summary>
    // ---------- SDO 直接读总线 AD 模块 ----------

    /// <summary>
    /// SDO 读 EtherCAT 从站对象字典（Service Data Object，配置类、单次数据读）。
    /// type: 1=bool 2=int8 3=int16 4=int32 5=uint8 6=uint16 7=uint32
    /// </summary>
    public (int rc, int value) SDOReadRaw(int node, uint index, uint subindex, uint type = 0x06)
    {
        EnsureConnected();
        int value = 0;
        int rc = zmcaux.ZAux_BusCmd_SDORead(_handle, 0, (uint)node, index, subindex, type, ref value);
        return (rc, value);
    }

    /// <summary>
    /// PDO 读 EtherCAT 从站 Process Data Object（实时通道、适合周期性 AD/IO 读取）。
    /// 参数和 SDORead 一样，但走 PDO 通道，通常支持复合对象内部的 subindex 读。
    /// type: 1=bool 2=int8 3=int16 4=int32 5=uint8 6=uint16 7=uint32
    /// </summary>
    public (int rc, int value) NodePdoReadRaw(int node, uint index, uint subindex, uint type = 0x06)
    {
        EnsureConnected();
        int value = 0;
        int rc = zmcaux.ZAux_BusCmd_NodePdoRead(_handle, (uint)node, index, subindex, type, ref value);
        return (rc, value);
    }

    /// <summary>
    /// SDO 写 EtherCAT 从站对象字典（配置量程、通道使能等）。
    /// type: 1=bool 2=int8 3=int16 4=int32 5=uint8 6=uint16 7=uint32
    /// </summary>
    public int SDOWriteRaw(int node, uint index, uint subindex, uint type, int value)
    {
        EnsureConnected();
        return zmcaux.ZAux_BusCmd_SDOWrite(_handle, 0, (uint)node, index, subindex, type, value);
    }

    /// <summary>
    /// 初始化 EC8124：写量程（0x2000）和使能通道（0x2002）。
    /// rangeMode: 0=±10V, 1=±5V。
    /// </summary>
    public (int rc, string log) InitEC8124(int node = 1, int rangeMode = 0)
    {
        var sb = new System.Text.StringBuilder();

        // 1. 量程配置 0x2000 (UINT16)
        int rc = SDOWriteRaw(node, 0x2000, 0, 0x06, rangeMode);
        sb.AppendLine($"  SDOWrite 0x2000(range={rangeMode}): rc={rc}");
        // 读回验证
        var (rc2, v2) = SDOReadRaw(node, 0x2000, 0, 0x06);
        sb.AppendLine($"  SDORead  0x2000: rc={rc2} value={v2}");

        // 2. 通道使能 0x2002 (DT2002 复合类型，sub=1~4 分别使能 ch0~ch3)
        sb.AppendLine("  通道使能 0x2002 (DT2002 sub1~4):");
        for (int ch = 1; ch <= 4; ch++)
        {
            int rc3 = SDOWriteRaw(node, 0x2002, (uint)ch, 0x06, 1);
            sb.AppendLine($"    sub{ch}=1: rc={rc3}");
        }
        // 读回验证
        sb.AppendLine("  读回验证 (NodePdoRead 0x6401 sub1~4):");
        for (int ch = 1; ch <= 4; ch++)
        {
            var (rc4, v4) = NodePdoReadRaw(node, 0x6401, (uint)ch, 0x06);
            int signed = (int)(v4 & 0xFFFF);// - (int)HW_ZERO;
            sb.AppendLine($"    ch{ch - 1}: rc={rc4} raw=0x{v4 & 0xFFFF:X4}=raw{signed}");
        }

        return (rc, sb.ToString());
    }

    /// <summary>
    /// 读 EC8124 的 4 路 AD（DeviceID=0x8124，对象 0x6401 DT6401 复合类型）。
    /// 优先 NodePdoRead（PDO 通道），失败降级 SDORead。
    /// 硬件量程 ±5V，PDO 声明类型 UINT16：-5V→0, 0V→32768, +5V→65535。
    /// 返回值已减去硬件零点 32768，范围 -32768~+32767，0V≈0。
    /// </summary>
    public (int rc, float voltage)[] ReadEC8124AD(int node = 1)
    {
        // EC8124 ±5V 量程：UINT16 0~65535 → 按 INT16 解释减 HW_ZERO(32768) → -32768~+32767
        const uint HW_ZERO = 32768u;
        var result = new (int rc, float voltage)[4];
        for (int ch = 0; ch < 4; ch++)
        {
            // DT6401: sub=ch+1 对应 ch0~ch3，UINT16（type=6，PDO 声明类型）
            var (rc, raw) = NodePdoReadRaw(node, 0x6401, (uint)(ch + 1), 0x06);
            if (rc != 0)
                (rc, raw) = SDOReadRaw(node, 0x6401, (uint)(ch + 1), 0x06);

            // UINT16 → INT16：减硬件零点 32768，得到 -32768~+32767 的有符号 ADC 值
            int signed = (int)(raw & 0xFFFF) - (int)HW_ZERO;
            result[ch] = (rc, signed);
        }
        return result;
    }

    /// <summary>
    /// 软限位实时监控：周期读 Dpos 与正/负软限位比较。
    /// 越界且用户指令仍指向限位方向 → 强制减速停止（Cancel imode=2）；
    /// 用户指令指向限位外（反向）不拦，否则停在限位外就再也回不来了。
    /// 方向不再用 speed 正负判断（CSP 模式下 VpSpeed 符号可能不可靠），
    /// 改用 RunMove 设置的 _lastMoveDir 记录用户意图。
    /// 进入越界区时通过 LimitTriggered 提示一次（持续越界不重复提示）。
    /// </summary>
    private async Task LimitMonitorLoopAsync(CancellationToken ct)
    {
        const int checkIntervalMs = 50;   // 检测周期
        int axis = _cfg.ZMotion.GetAxisNumber();

        while (!ct.IsCancellationRequested)
        {
            try
            {
                // 找零（双限位对中）期间本就要主动撞到物理限位（常超软限位），暂停软限位监控，避免与对中运动互推。
                // 波形运行期间（无论开关开/关）由 WaveformLoopAsync 逐拍自行处理软限位（开=触碰即停并上报，关=临时放开限位跑完），
                // 此处监控跳过避免与波形逐拍逻辑互推/重复提示；手动前进/后退（非波形）始终监控。
                if (_homing || _wfRunning)
                {
                    _wasOutOfLimit = false;
                    await Task.Delay(checkIntervalMs, ct).ConfigureAwait(false);
                    continue;
                }
                if (_connected && _handle != IntPtr.Zero)
                {
                    float dpos = 0;
                    if (zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos) == 0)
                    {
                        float posLim = _cfg.ZMotion.GetSoftLimitPos();
                        float negLim = _cfg.ZMotion.GetSoftLimitNeg();

                        bool hitPos = dpos >= posLim;
                        bool hitNeg = dpos <= negLim;
                        bool outOfLimit = hitPos || hitNeg;

                        // 只拦"用户仍在冲向限位"的方向（用按钮指令方向，不用 speed 符号）
                        // 正向限位 + 前进指令(_lastMoveDir>0) → 拦
                        // 负向限位 + 后退指令(_lastMoveDir<0) → 拦
                        // 其他情况（反向指令、停止、未知）不拦
                        if ((hitPos && _lastMoveDir > 0) || (hitNeg && _lastMoveDir < 0))
                            zmcaux.ZAux_Direct_Single_Cancel(_handle, axis, 2);

                        if (outOfLimit && !_wasOutOfLimit)
                        {
                            float lim = hitPos ? posLim : negLim;
                            string side = hitPos ? "正向" : "负向";
                            LimitTriggered?.Invoke(this,
                                $"已到{side}软限位 {lim:F1}（当前位置 {dpos:F1}），已强制停止");
                        }
                        _wasOutOfLimit = outOfLimit;
                    }
                }
            }
            catch { /* 单次读/停失败不影响监控循环，下一轮重试 */ }

            await Task.Delay(checkIntervalMs, ct).ConfigureAwait(false);
        }
    }

    private async Task SensorLoopAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            if (_connected)
            {
                double[] values = new double[4];

                // 优先：直接从 EC8124 读（NodePdoRead → SDORead 降级链）
                try
                {
                    var ec = ReadEC8124AD(1);
                    int okCount = ec.Count(e => e.rc == 0);
                    if (okCount >= 2)   // 至少 2 路通就用它
                    {
                        for (int i = 0; i < 4; i++) values[i] = ec[i].voltage;
                        SensorDataReceived?.Invoke(this, new SensorDataEventArgs(values));
                        await Task.Delay(_cfg.Sensor.GetUpdateIntervalMs(), ct).ConfigureAwait(false);
                        continue;
                    }
                }
                catch { }

                // 降级：本体 GetAD（AIN start + 0..count-1）
                int start = _cfg.Sensor.GetAinStart();
                int count = _cfg.Sensor.GetAinCount();
                for (int i = 0; i < Math.Min(count, values.Length); i++)
                {
                    float v = 0;
                    int rc = zmcaux.ZAux_Direct_GetAD(_handle, start + i, ref v);
                    values[i] = (rc == 0) ? v : double.NaN;
                }
                SensorDataReceived?.Invoke(this, new SensorDataEventArgs(values));
            }
            await Task.Delay(_cfg.Sensor.GetUpdateIntervalMs(), ct).ConfigureAwait(false);
        }
    }

    private void EnsureConnected()
    {
        if (!_connected || _handle == IntPtr.Zero)
            throw new InvalidOperationException("未连接到正运动控制器");
    }

    private static void ThrowRc(int rc, string api)
    {
        if (rc != 0) throw new InvalidOperationException($"ZAux_{api} 失败 rc={rc}");
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        Disconnect();
        GC.SuppressFinalize(this);
    }
}
