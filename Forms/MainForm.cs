using MotorControlApp.Configuration;
using MotorControlApp.Devices;

namespace MotorControlApp.Forms;

/// <summary>
/// 主界面：连接正运动控制卡 → 配置轴参数 → 前进/后退/停止 → 显示 4 路传感器。
/// 所有可编辑参数来自 ZMotion SDK（zauxdll.dll），参考例程 Form1。
/// </summary>
public class MainForm : Form
{
    private readonly AppConfig _config;
    private IDeviceController? _device;
    private bool _useSimulator;

    // 顶部状态栏
    private Label _lblStatus = null!;

    // 连接面板
    private TextBox _txtTarget = null!;
    private TextBox _txtTimeout = null!;
    private Button _btnSearch = null!;
    private Button _btnConnect = null!;

    // 轴参数面板
    private TextBox _txtAxisNumber = null!;
    private TextBox _txtSpeed = null!;
    private TextBox _txtAccel = null!;
    private TextBox _txtDecel = null!;
    private TextBox _txtLspeed = null!;
    private TextBox _txtUnits = null!;
    private TextBox _txtSramp = null!;
    private Button _btnUpdate = null!;

    // 运动控制
    private Button _btnForward = null!;
    private Button _btnBackward = null!;
    private Button _btnStop = null!;

    // 传感器
    private readonly TextBox[] _sensorBoxes = new TextBox[4];
    private readonly int[] _zeroOffsets = new int[4];   // 归零偏移（点击"归零校准"时记录当前 raw，显示时减去）

    private volatile bool _connecting;
    private volatile bool _disconnecting;

    public MainForm(AppConfig config)
    {
        _config = config;
        _useSimulator = config.Connection.UseSimulator.Value;
        BuildUi();
    }

    private void BuildUi()
    {
        Text = _useSimulator ? "电机控制器（模拟模式）" : "电机控制器（正运动 ZMotion）";
        ClientSize = new Size(440, 820);
        StartPosition = FormStartPosition.CenterScreen;
        FormBorderStyle = FormBorderStyle.FixedSingle;
        MaximizeBox = false;
        MinimizeBox = false;

        // 状态栏
        _lblStatus = new Label { Text = "未连接", Location = new Point(16, 13), Size = new Size(408, 22), ForeColor = Color.DimGray };
        Controls.Add(_lblStatus);

        BuildConnectionPanel();
        BuildAxisPanel();
        BuildMotorPanel();
        BuildSensorPanel();

        FormClosing += MainForm_FormClosing;
    }

    // ============================================================ 连接面板

    private void BuildConnectionPanel()
    {
        var group = new GroupBox
        {
            Text = "连接",
            Location = new Point(16, 46),
            Size = new Size(408, 118)
        };

        // 连接类型（固定 Ethernet）
        group.Controls.Add(new Label { Text = "类型：Ethernet", Location = new Point(16, 26), Size = new Size(110, 22), ForeColor = Color.SeaGreen, Font = new Font(Font.FontFamily, 9, FontStyle.Bold) });

        // 目标 + 搜索按钮
        group.Controls.Add(new Label { Text = "PAC IP：", Location = new Point(140, 26), Size = new Size(54, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtTarget = new TextBox { Location = new Point(198, 23), Size = new Size(120, 27), Text = _config.Connection.Target.Value };
        group.Controls.Add(_txtTarget);

        _btnSearch = new Button { Text = "搜索", Location = new Point(322, 22), Size = new Size(48, 28) };
        _btnSearch.Click += (_, _) => SafeRun(SearchEthAndFill, ShowErr);
        group.Controls.Add(_btnSearch);

        // 超时
        group.Controls.Add(new Label { Text = "超时(ms)：", Location = new Point(16, 62), Size = new Size(72, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtTimeout = new TextBox { Location = new Point(92, 59), Size = new Size(60, 27), Text = _config.Connection.TimeoutMs.Value.ToString(), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtTimeout);

        // 连接按钮
        _btnConnect = new Button { Text = "连接", Location = new Point(252, 58), Size = new Size(120, 34) };
        _btnConnect.Click += BtnConnect_Click;
        group.Controls.Add(_btnConnect);

        Controls.Add(group);
    }

    // ============================================================ 轴参数面板

    private void BuildAxisPanel()
    {
        var group = new GroupBox
        {
            Text = "轴参数",
            Location = new Point(16, 172),
            Size = new Size(408, 264)
        };

        // 第一行：轴号 + 脉冲当量
        group.Controls.Add(new Label { Text = "轴号：", Location = new Point(16, 28), Size = new Size(50, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtAxisNumber = new TextBox { Location = new Point(70, 25), Size = new Size(80, 27), Text = _config.ZMotion.AxisNumber.Value.ToString(), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtAxisNumber);

        group.Controls.Add(new Label { Text = "脉冲当量：", Location = new Point(180, 28), Size = new Size(68, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtUnits = new TextBox { Location = new Point(252, 25), Size = new Size(70, 27), Text = _config.ZMotion.Units.Value.ToString("0.###"), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtUnits);
        group.Controls.Add(new Label { Text = "mm/pulse", Location = new Point(326, 28), Size = new Size(60, 22), ForeColor = Color.DimGray });

        // 第二行：运行速度
        group.Controls.Add(new Label { Text = "速度：", Location = new Point(16, 66), Size = new Size(50, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtSpeed = new TextBox { Location = new Point(70, 63), Size = new Size(80, 27), Text = _config.ZMotion.Speed.Value.ToString("0.#"), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtSpeed);
        group.Controls.Add(new Label { Text = "mm/s", Location = new Point(154, 66), Size = new Size(48, 22), ForeColor = Color.DimGray });

        group.Controls.Add(new Label { Text = "最低速度：", Location = new Point(220, 66), Size = new Size(68, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtLspeed = new TextBox { Location = new Point(292, 63), Size = new Size(70, 27), Text = _config.ZMotion.Lspeed.Value.ToString("0.#"), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtLspeed);
        group.Controls.Add(new Label { Text = "mm/s", Location = new Point(366, 66), Size = new Size(40, 22), ForeColor = Color.DimGray });

        // 第三行：加减速
        group.Controls.Add(new Label { Text = "加速度：", Location = new Point(16, 104), Size = new Size(50, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtAccel = new TextBox { Location = new Point(70, 101), Size = new Size(80, 27), Text = _config.ZMotion.Accel.Value.ToString("0"), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtAccel);
        group.Controls.Add(new Label { Text = "mm/s²", Location = new Point(154, 104), Size = new Size(52, 22), ForeColor = Color.DimGray });

        group.Controls.Add(new Label { Text = "减速度：", Location = new Point(220, 104), Size = new Size(68, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtDecel = new TextBox { Location = new Point(292, 101), Size = new Size(70, 27), Text = _config.ZMotion.Decel.Value.ToString("0"), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtDecel);
        group.Controls.Add(new Label { Text = "mm/s²", Location = new Point(366, 104), Size = new Size(40, 22), ForeColor = Color.DimGray });

        // 第四行：S 曲线 + 更新按钮
        group.Controls.Add(new Label { Text = "S曲线：", Location = new Point(16, 142), Size = new Size(50, 22), TextAlign = ContentAlignment.MiddleRight });
        _txtSramp = new TextBox { Location = new Point(70, 139), Size = new Size(80, 27), Text = _config.ZMotion.Sramp.Value.ToString("0"), TextAlign = HorizontalAlignment.Right };
        group.Controls.Add(_txtSramp);
        group.Controls.Add(new Label { Text = "ms (0=关闭)", Location = new Point(154, 142), Size = new Size(74, 22), ForeColor = Color.DimGray });

        _btnUpdate = new Button { Text = "应用参数", Location = new Point(292, 138), Size = new Size(92, 30) };
        _btnUpdate.Click += (_, _) =>
        {
            string? err = ReadAndApplyParams();
            if (err != null) TipForm.Show(this, err, false, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
            else TipForm.Show(this, "参数已应用", true, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
        };
        group.Controls.Add(_btnUpdate);

        Controls.Add(group);
    }

    // ============================================================ 运动控制

    private void BuildMotorPanel()
    {
        var group = new GroupBox
        {
            Text = "运动控制",
            Location = new Point(16, 450),
            Size = new Size(408, 86)
        };

        _btnForward = new Button { Text = "◀  后退", Location = new Point(16, 30), Size = new Size(110, 44), Font = new Font(Font.FontFamily, 12, FontStyle.Bold) };
        _btnForward.Click += (_, _) => SafeRun(() => _device?.Backward(), ShowErr);
        _btnForward.Enabled = _device != null && _device.IsConnected;

        _btnBackward = new Button { Text = "停止", Location = new Point(149, 30), Size = new Size(110, 44), Font = new Font(Font.FontFamily, 12, FontStyle.Bold), BackColor = Color.LightCoral };
        _btnBackward.Click += (_, _) => SafeRun(() => _device?.Stop(), ShowErr);
        _btnBackward.Enabled = _device != null && _device.IsConnected;

        _btnStop = new Button { Text = "前进  ▶", Location = new Point(282, 30), Size = new Size(110, 44), Font = new Font(Font.FontFamily, 12, FontStyle.Bold) };
        _btnStop.Click += (_, _) => SafeRun(() => _device?.Forward(), ShowErr);
        _btnStop.Enabled = _device != null && _device.IsConnected;

        group.Controls.AddRange(new Control[] { _btnForward, _btnBackward, _btnStop });
        Controls.Add(group);
    }

    // ============================================================ 传感器

    private void BuildSensorPanel()
    {
        var group = new GroupBox
        {
            Text = "传感器实时数据",
            Location = new Point(16, 550),
            Size = new Size(408, 248)  // 底部加扫描按钮行
        };

        // 4 个通道显示
        for (int i = 0; i < 4; i++)
        {
            int rowY = 26 + i * 40;
            group.Controls.Add(new Label
            {
                Text = $"CH{i}",
                Location = new Point(16, rowY + 4),
                Size = new Size(40, 22),
                TextAlign = ContentAlignment.MiddleCenter,
                ForeColor = Color.DimGray
            });
            _sensorBoxes[i] = new TextBox
            {
                Location = new Point(62, rowY),
                Size = new Size(160, 27),
                ReadOnly = true,
                TextAlign = HorizontalAlignment.Right,
                Font = new Font("Consolas", 12)
            };
            group.Controls.Add(_sensorBoxes[i]);
            group.Controls.Add(new Label
            {
                Text = "raw",
                Location = new Point(228, rowY + 4),
                Size = new Size(40, 22),
                ForeColor = Color.DimGray
            });
        }

        // 扫描按钮 + 归零校准 + 清除
        var btnScan = new Button
        {
            Text = "总线诊断",
            Location = new Point(16, 190),
            Size = new Size(100, 32)
        };
        btnScan.Click += (_, _) => SafeRun(ScanAINAndShow, ShowErr);
        group.Controls.Add(btnScan);

        var btnZero = new Button
        {
            Text = "归零校准",
            Location = new Point(124, 190),
            Size = new Size(100, 32)
        };
        btnZero.Click += BtnZeroCal_Click;
        group.Controls.Add(btnZero);

        var btnClearZero = new Button
        {
            Text = "清除零点",
            Location = new Point(232, 190),
            Size = new Size(100, 32)
        };
        btnClearZero.Click += (_, _) =>
        {
            Array.Clear(_zeroOffsets, 0, _zeroOffsets.Length);
            TipForm.Show(this, "零点已清除", true, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
        };
        group.Controls.Add(btnClearZero);

        Controls.Add(group);
    }

    // ============================================================ 连接

    private async void BtnConnect_Click(object? sender, EventArgs e)
    {
        if (_connecting || _disconnecting) return;

        if (_device is { IsConnected: true })
        {
            DisconnectDevice();
            return;
        }

        // 先把界面值写回 config（连接用最新参数）
        string? reason = ReadAndApplyParams();
        if (reason != null)
        {
            TipForm.Show(this, reason, false, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
            return;
        }

        _connecting = true;
        _btnConnect.Enabled = false;
        _lblStatus.Text = "连接中...";
        _lblStatus.ForeColor = Color.DimGray;

        try
        {
            _device?.Dispose();
            _device = DeviceFactory.Create(_config);
            _device.SensorDataReceived += Device_SensorDataReceived;
            _device.Disconnected += Device_Disconnected;

            await _device.ConnectAsync();

            _lblStatus.Text = $"已连接：{_config.Connection.Type.Value} {_config.Connection.Target.Value}";
            _lblStatus.ForeColor = Color.SeaGreen;
            _btnConnect.Text = "断开";
            SetConnectedButtons(true);
            TipForm.Show(this, "连接成功", true, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
        }
        catch (Exception ex)
        {
            _lblStatus.Text = "连接失败";
            _lblStatus.ForeColor = Color.IndianRed;
            TipForm.Show(this, $"连接失败：{ex.Message}", false, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
        }
        finally
        {
            _connecting = false;
            _btnConnect.Enabled = true;
        }
    }

    private void DisconnectDevice()
    {
        if (_disconnecting) return;
        _disconnecting = true;

        try
        {
            _device?.Disconnect();
            SetConnectedButtons(false);
            _lblStatus.Text = "未连接";
            _lblStatus.ForeColor = Color.DimGray;
            _btnConnect.Text = "连接";
            for (int i = 0; i < _sensorBoxes.Length; i++)
                _sensorBoxes[i].Text = "";
        }
        finally { _disconnecting = false; }
    }

    private void Device_Disconnected(object? sender, EventArgs e)
    {
        if (IsDisposed || !IsHandleCreated) return;
        BeginInvoke(DisconnectDevice);
    }

    private void Device_SensorDataReceived(object? sender, SensorDataEventArgs e)
    {
        if (IsDisposed || !IsHandleCreated) return;
        BeginInvoke(() =>
        {
            for (int i = 0; i < _sensorBoxes.Length && i < e.Values.Count; i++)
            {
                int raw = (int)Math.Round(e.Values[i]);
                int calibrated = raw - _zeroOffsets[i];
                _sensorBoxes[i].Text = calibrated.ToString();
            }
        });
    }

    /// <summary>归零校准：把当前 4 路 raw 作为零点，之后显示减去该值。</summary>
    private void BtnZeroCal_Click(object? sender, EventArgs e)
    {
        if (_device is not Devices.ZMotionDeviceController zmc)
        {
            TipForm.Show(this, "未连接，无法校准", false, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
            return;
        }

        var ec = zmc.ReadEC8124AD(1);
        bool anyOk = false;
        for (int i = 0; i < ec.Length; i++)
        {
            if (ec[i].rc == 0)
            {
                _zeroOffsets[i] = (int)Math.Round(ec[i].voltage);   // voltage 存的是 raw 原始码
                anyOk = true;
            }
        }

        if (anyOk)
            TipForm.Show(this, $"已归零：ch0={_zeroOffsets[0]} ch1={_zeroOffsets[1]} ch2={_zeroOffsets[2]} ch3={_zeroOffsets[3]}", true, 3000);
        else
            TipForm.Show(this, "EC8124 读取失败，未校准", false, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
    }

    private void SetConnectedButtons(bool connected)
    {
        _btnForward.Enabled = connected;
        _btnBackward.Enabled = connected;
        _btnStop.Enabled = connected;
        _txtTarget.Enabled = !connected;
        _txtTimeout.Enabled = !connected;
        _btnSearch.Enabled = !connected;  // 只有 Ethernet 模式，未连接时始终可搜索
    }

    // ============================================================ 参数读写

    /// <summary>总线 + AIN + 轴诊断（精简版）。</summary>
    private void ScanAINAndShow()
    {
        if (_device is not Devices.ZMotionDeviceController zmc)
            throw new InvalidOperationException("未连接到正运动控制器");

        var sb = new System.Text.StringBuilder();

        // (1) 总线节点
        sb.AppendLine("=== EtherCAT 总线节点 ===");
        var nodes = zmc.ScanBusNodes();
        sb.AppendLine($"共 {nodes.Length} 个节点：");
        foreach (var n in nodes)
        {
            string hint = n.device == 0x8124 ? " EC8124 AD" : (n.ainCnt > 0 ? " AD模块" : (n.inCnt + n.outCnt > 0 ? " IO模块" : ""));
            sb.AppendLine($"  Node {n.node}  DevID=0x{n.device:X8}  IN={n.inCnt} OUT={n.outCnt} AIN={n.ainCnt} AOUT={n.aoutCnt}{hint}");
        }
        sb.AppendLine();

        // (2) NODE_AIO 映射（关键！总线 AIN 必须映射到全局编号才能被 GetAD 读到）
        sb.AppendLine("=== NODE_AIO 映射 ===");
        var aioMap = zmc.ReadNodeAIOMap();
        bool aioOk = true;
        foreach (var m in aioMap)
        {
            string status;
            if (m.cmdAin.StartsWith("ERR"))
            { status = $"BASIC命令失败：{m.cmdAin}"; aioOk = false; }
            else if (m.ainCount > 0 && m.ainBase <= 0)
            { status = $"AIN有{m.ainCount}路但起始编号=0（可能和本体冲突）"; aioOk = false; }
            else if (m.ainCount > 0)
            { status = $"AIN已映射到全局{m.ainBase}~{m.ainBase + m.ainCount - 1}"; }
            else
            { status = "无AIN"; }
            sb.AppendLine($"  Node {m.node}：{status}");
        }
        sb.AppendLine();

        // (3) ★ 关键：BASIC 层直接测 EtherCAT 状态 + SDO_READ
        sb.AppendLine("=== BASIC 层 EtherCAT 直接测 ===");
        try
        {
            sb.AppendLine("  -- 节点状态 (NODE_STATUS) --");
            for (int n = 0; n < 3; n++)
            {
                try { sb.AppendLine($"    Node{n}: NODE_STATUS={zmc.BasicCmd($"?NODE_STATUS(0,{n})")}"); }
                catch { sb.AppendLine($"    Node{n}: ERR"); }
            }
            sb.AppendLine("  -- SDO_READ Node1 idx=0x6401 sub=1 (BASIC层) --");
            try { sb.AppendLine($"    BASIC SDO_READ → {zmc.BasicCmd("SDO_READ(0,1,&H6401,1,2,_v) : ?\"rc=\"; _v")}"); }
            catch { sb.AppendLine("    BASIC SDO_READ → ERR"); }
            sb.AppendLine("  -- C# SDK SDOReadRaw Node1 idx=0x6401 sub=1 --");
            try { var (rc, val) = zmc.SDOReadRaw(1, 0x6401, 1, 0x02); sb.AppendLine($"    rc={rc} value={val}"); }
            catch (Exception ex) { sb.AppendLine($"    ERR: {ex.Message}"); }
            sb.AppendLine("  -- C# SDK NodePdoReadRaw Node1 idx=0x6401 sub=1 --");
            try { var (rc, val) = zmc.NodePdoReadRaw(1, 0x6401, 1, 0x02); sb.AppendLine($"    rc={rc} value={val}"); }
            catch (Exception ex) { sb.AppendLine($"    ERR: {ex.Message}"); }
        }
        catch (Exception ex) { sb.AppendLine($"  BASIC 层测试整体失败: {ex.Message}"); }
        sb.AppendLine();

        // (4) GetAD 全局 AIN 扫描（只打印有值的，全 0 则提示）
        sb.AppendLine("=== GetAD 全局 AIN 扫描 ===");
        var channels = zmc.ScanAIN(32);
        var alive = channels.Where(c => c.value.HasValue && Math.Abs(c.value.Value) > 0.001f).ToArray();
        if (alive.Length == 0)
        {
            sb.AppendLine("  32 路 AIN 全部 = 0.000");
            sb.AppendLine("  原因 1：NODE_AIO 没配，总线 AD 模块不在全局编号空间");
            sb.AppendLine("  原因 2：本体 AIN 通道传感器未接或浮空");
        }
        else
        {
            foreach (var c in alive)
                sb.AppendLine($"  AIN[{c.ionum:D2}] = {c.value.Value:F3}");
        }
        sb.AppendLine();

        // (5) ★ 配置 + 读回 EC8124
        sb.AppendLine("=== 初始化 EC8124 (量程=±10V + 通道使能) ===");
        try
        {
            var (rc, log) = zmc.InitEC8124(1, 0);
            sb.Append(log);
            sb.AppendLine($"  初始化整体 rc={rc}");
        }
        catch (Exception ex) { sb.AppendLine($"  InitEC8124 失败: {ex.Message}"); }
        sb.AppendLine();

        // (6) 轴 0 状态
        sb.AppendLine("=== 轴 0 状态 ===");
        sb.Append(zmc.DumpAxisInfo(0));

        // (7) 结论
        sb.AppendLine();
        sb.AppendLine("--- 结论 ---");
        bool ecOk = false;
        try { ecOk = zmc.ReadEC8124AD(1).Count(e => e.rc == 0) >= 2; } catch { }
        if (ecOk) sb.AppendLine("✅ EC8124 NodePdoRead 通了！UI传感器将直接读总线AD");
        if (alive.Length > 0) sb.AppendLine($"✅ 本体 GetAD 有 {alive.Length} 路活跃（AIN[0]={alive.First().ionum}）");
        if (!ecOk && alive.Length == 0) sb.AppendLine("⚠ 总线+本体都没读到 → 检查RTSys是否正常、传感器接线");

        MessageBox.Show(sb.ToString(), "总线 + AIN + 轴 诊断", MessageBoxButtons.OK, MessageBoxIcon.Information);
    }
    /// <summary>正运动 SDK 广播搜索同一网段内的 PAC 控制器，找到第一个后自动填入目标 IP。</summary>
    private void SearchEthAndFill()
    {
        if (_device is { IsConnected: true })
            throw new InvalidOperationException("已连接，请先断开再搜索");

        var sb = new System.Text.StringBuilder(10240);
        int rc = cszmcaux.zmcaux.ZAux_SearchEthlist(sb, 10230, 1500);
        if (rc != 0 || sb.Length == 0)
            throw new InvalidOperationException($"未找到正运动控制器（rc={rc}，请确认 PC 与 PAC 在同一网段且 PAC 已启动）");

        string[] ips = sb.ToString().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (ips.Length == 0)
            throw new InvalidOperationException($"搜索返回为空（原始：{sb}）");

        _txtTarget.Text = ips[0];
        TipForm.Show(this, $"找到 {ips.Length} 台控制器，已填入 {ips[0]}", true, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
    }

    private string? ReadAndApplyParams()
    {
        if (!TryParseInt(_txtAxisNumber.Text, out int axis)) return "轴号必须是整数";
        if (!TryParseFloat(_txtUnits.Text, out float units) || units <= 0) return "脉冲当量必须 > 0";
        if (!TryParseFloat(_txtSpeed.Text, out float speed) || speed <= 0) return "速度必须 > 0";
        if (!TryParseFloat(_txtAccel.Text, out float accel) || accel <= 0) return "加速度必须 > 0";
        if (!TryParseFloat(_txtDecel.Text, out float decel) || decel <= 0) return "减速度必须 > 0";
        if (!TryParseFloat(_txtLspeed.Text, out float lspeed) || lspeed <= 0) return "最低速度必须 > 0";
        if (!TryParseFloat(_txtSramp.Text, out float sramp) || sramp < 0) return "S 曲线必须 >= 0";
        if (!TryParseInt(_txtTimeout.Text, out int timeout) || timeout <= 0) return "超时必须 > 0";

        _config.Connection.Type.Value = "Ethernet";  // 固定网口连接
        _config.Connection.Target.Value = _txtTarget.Text.Trim();
        _config.Connection.TimeoutMs.Value = timeout;
        _config.ZMotion.AxisNumber.Value = axis;
        _config.ZMotion.Units.Value = units;
        _config.ZMotion.Speed.Value = speed;
        _config.ZMotion.Accel.Value = accel;
        _config.ZMotion.Decel.Value = decel;
        _config.ZMotion.Lspeed.Value = lspeed;
        _config.ZMotion.Sramp.Value = sramp;
        return null;
    }

    private static bool TryParseInt(string? t, out int v) => int.TryParse(t, out v);
    private static bool TryParseFloat(string? t, out float v) => float.TryParse(t, out v);

    private static void SafeRun(Action action, Action<string> onError)
    {
        try { action(); }
        catch (Exception ex) { onError(ex.Message); }
    }

    private void ShowErr(string msg) =>
        TipForm.Show(this, msg, false, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));

    // ============================================================ 关闭

    private void MainForm_FormClosing(object? sender, FormClosingEventArgs e)
    {
        _device?.Disconnect();
        _device?.Dispose();
        ReadAndApplyParams(); // 尽力保存
        ConfigService.TrySave(_config);
    }
}



