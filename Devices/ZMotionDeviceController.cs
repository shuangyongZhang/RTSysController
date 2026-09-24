using cszmcaux;
using MotorControlApp.Configuration;
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
    private int _connectedAxis = -1;     // 本次连接配置的轴号（防止在线改轴号后把参数下发到其他轴）

    public ZMotionDeviceController(AppConfig config)
    {
        ArgumentNullException.ThrowIfNull(config);
        _cfg = config;
    }

    public bool IsConnected => _connected;
    public event EventHandler<SensorDataEventArgs>? SensorDataReceived;
    public event EventHandler<string>? LimitTriggered;

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

    public void Disconnect()
    {
        lock (_lock)
        {
            if (!_connected) return;
            _connected = false;
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
        int axis = _cfg.ZMotion.GetAxisNumber();

        // (0) 防爆冲：运动前 Dpos 越界拦截（软限位之外再套一层，双保险）
        float dpos = GetCurrentDpos();
        float posLim = _cfg.ZMotion.GetSoftLimitPos();
        float negLim = _cfg.ZMotion.GetSoftLimitNeg();
        if (dir > 0 && dpos >= posLim)
            throw new InvalidOperationException($"已到正向软限位 {posLim:F1}，无法{label}（当前 Dpos={dpos:F1}）");
        if (dir < 0 && dpos <= negLim)
            throw new InvalidOperationException($"已到负向软限位 {negLim:F1}，无法{label}（当前 Dpos={dpos:F1}）");

        // (1) Vmove 前快照
        int beforeIdle = 0;
        float beforeSpeed = 0, beforeDpos = 0;
        zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref beforeIdle);
        zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref beforeSpeed);
        zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref beforeDpos);

        // (2) 发指令
        ThrowRc(zmcaux.ZAux_Direct_Single_Vmove(_handle, axis, dir), $"Vmove({dir})");

        // (3) 等 300ms 让指令生效，再读状态
        Thread.Sleep(300);
        int afterIdle = 0;
        float afterSpeed = 0, afterDpos = 0;
        zmcaux.ZAux_Direct_GetIfIdle(_handle, axis, ref afterIdle);
        zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref afterSpeed);
        zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref afterDpos);

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

    /// <summary>按 ZMC 轴状态字位定义解码：bit4=正向硬限位 bit5=负向硬限位 bit22=伺服报警。</summary>
    private static string DescribeAxisStatus(int axisstate)
    {
        if (axisstate == 0) return "正常";
        var parts = new List<string>();
        if (((axisstate >> 4) & 1) == 1) parts.Add("正向硬限位报警");
        if (((axisstate >> 5) & 1) == 1) parts.Add("负向硬限位报警");
        if (((axisstate >> 22) & 1) == 1) parts.Add("伺服报警");
        // 其余置位的位原样列出，避免漏掉未知报警
        for (int b = 0; b < 32; b++)
        {
            if (b == 4 || b == 5 || b == 22) continue;
            if (((axisstate >> b) & 1) == 1) parts.Add($"bit{b}");
        }
        return parts.Count > 0 ? string.Join(", ", parts) : "正常";
    }

    public void Stop()
    {
        EnsureConnected();
        ThrowRc(zmcaux.ZAux_Direct_Single_Cancel(_handle, _cfg.ZMotion.GetAxisNumber(), 2), "Cancel(2)");
    }

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
    /// 越界且速度仍指向限位方向 → 强制减速停止（Cancel imode=2）；
    /// 速度离开限位方向（回程退出）不拦，否则停在限位外就再也回不来了。
    /// 进入越界区时通过 LimitTriggered 提示一次（持续越界不重复提示）。
    /// 控制器侧 FS_LIMIT/RS_LIMIT 硬级软限位仍然有效，本循环是上位机的第二道防线。
    /// </summary>
    private async Task LimitMonitorLoopAsync(CancellationToken ct)
    {
        const int checkIntervalMs = 50;   // 检测周期
        int axis = _cfg.ZMotion.GetAxisNumber();

        while (!ct.IsCancellationRequested)
        {
            try
            {
                if (_connected && _handle != IntPtr.Zero)
                {
                    float dpos = 0, speed = 0;
                    if (zmcaux.ZAux_Direct_GetDpos(_handle, axis, ref dpos) == 0)
                    {
                        zmcaux.ZAux_Direct_GetVpSpeed(_handle, axis, ref speed);
                        float posLim = _cfg.ZMotion.GetSoftLimitPos();
                        float negLim = _cfg.ZMotion.GetSoftLimitNeg();

                        bool hitPos = dpos >= posLim;
                        bool hitNeg = dpos <= negLim;
                        bool outOfLimit = hitPos || hitNeg;

                        // 只拦"继续冲向限位"的方向
                        if ((hitPos && speed > 0) || (hitNeg && speed < 0))
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
