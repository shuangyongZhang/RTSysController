using System.Globalization;
using MotorControlApp.Configuration;
using MotorControlApp.Devices;
using MotorControlApp.Waveforms;

namespace MotorControlApp.Forms;

/// <summary>
/// MainForm 的运动波形部分：公共参数置顶 + 五种模式各一个 TabPage，
/// 改参数即做时域校验（多正弦为合成预览实时提示超限），超限禁止启动；底部实时读数。
/// </summary>
public partial class MainForm
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private TabControl _tabs = null!;
    private TextBox _txtWfDt = null!;
    private TextBox _txtWfAccel = null!;
    private CheckBox _chkEnforceSoft = null!;   // 软限位保护开关
    private readonly ToolTip _tip = new();      // 参数悬停说明
    private Button _btnWfStart = null!;
    private Button _btnWfStop = null!;
    private Button _btnWfScale = null!;
    private Button _btnWfPreview = null!;
    private Button _btnWfLive = null!;
    private Label _lblWfPeakV = null!;
    private Label _lblWfStroke = null!;
    private Label _lblWfPeakA = null!;
    private Label _lblWfStatus = null!;
    private Label _lblRdVel = null!;
    private Label _lblRdAccel = null!;
    private Label _lblRdPos = null!;

    // 每个模式页一组输入框（键->TextBox）
    private readonly Dictionary<string, TextBox>[] _modeInputs = new Dictionary<string, TextBox>[5];
    private WaveformSimResult? _wfLastResult;
    private bool _wfSubscribed;
    
    // 单正弦 f/S/An/amax 四量耦合联动：记录用户最近直接编辑的字段（作为反推基准），
    // 并在程序性回填派生量时抑制 TextChanged，避免重入与把回填误判为用户编辑。
    private string _sineEditKey = "S";
    private bool _suppressSineLinkage;

    // 实时读数：相邻两拍速度差分算加速度
    private double _rdLastV;
    private DateTime _rdLastT = DateTime.MinValue;

    private static readonly string[] ModeNames = { "单正弦", "多正弦叠加", "方波", "脉冲", "伪随机PRTS" };

    private WaveformMode ActiveMode() => (WaveformMode)Math.Clamp(_tabs.SelectedIndex, 0, 4);

    private void BuildWaveformPanel()
    {
        for (int i = 0; i < _modeInputs.Length; i++) _modeInputs[i] = new Dictionary<string, TextBox>();

        // 悬停提示：慢速弹出，不干扰操作
        _tip.AutoPopDelay = 12000;
        _tip.InitialDelay = 350;
        _tip.ReshowDelay = 100;

        var group = new GroupBox
        {
            Text = "运动波形（可复现轨迹）",
            Location = new Point(508, 46),
            Size = new Size(480, 402)
        };

        // ---- 顶部公共参数 ----
        const string tipDt = "每拍下发生成器算出的带符号速度给控制器的时间步长 dt。建议 5~10ms：小于网络往返时延会下发不及时，过大则波形失真（控制频率须远高于波形频率）。";
        var lblDt = new Label { Text = "控制周期：", Location = new Point(8, 24), Size = new Size(68, 22), TextAlign = ContentAlignment.MiddleRight };
        _tip.SetToolTip(lblDt, tipDt);
        group.Controls.Add(lblDt);
        _txtWfDt = new TextBox { Location = new Point(76, 22), Size = new Size(48, 24), Text = _config.Waveform.ControlPeriodMs.Value.ToString(Inv), TextAlign = HorizontalAlignment.Right };
        _txtWfDt.TextChanged += (_, _) => ValidateNow();
        _tip.SetToolTip(_txtWfDt, tipDt);
        group.Controls.Add(_txtWfDt);
        group.Controls.Add(new Label { Text = "ms", Location = new Point(126, 24), Size = new Size(25, 22), ForeColor = Color.DimGray });

        const string tipAccel = "校验用的峰值加速度限制（mm/s²），0=不校验；运行期控制器的 Accel/Decel 也按此值钳制。仅对连续速度波形（正弦类）判限；方波/脉冲/PRTS 等阶梯型由控制器加减速钳制，不据此报超限。";
        var lblAccel = new Label { Text = "加速度上限：", Location = new Point(150, 24), Size = new Size(80, 22), TextAlign = ContentAlignment.MiddleRight };
        _tip.SetToolTip(lblAccel, tipAccel);
        group.Controls.Add(lblAccel);
        _txtWfAccel = new TextBox { Location = new Point(232, 22), Size = new Size(60, 24), Text = _config.Waveform.AccelLimitMmS2.Value.ToString("0.#", Inv), TextAlign = HorizontalAlignment.Right };
        _txtWfAccel.TextChanged += (_, _) => ValidateNow();
        _tip.SetToolTip(_txtWfAccel, tipAccel);
        group.Controls.Add(_txtWfAccel);
        var lblAccelUnit = new Label { Text = "mm/s²(0=不校验)", Location = new Point(294, 24), Size = new Size(100, 22), ForeColor = Color.DimGray };
        _tip.SetToolTip(lblAccelUnit, tipAccel);
        group.Controls.Add(lblAccelUnit);

        // 软限位保护开关：勾选=碰限位停；取消=坚持把运动做完
        _chkEnforceSoft = new CheckBox
        {
            Text = "软限位保护",
            Location = new Point(392, 23),
            Size = new Size(84, 24),
            Checked = _config.Waveform.EnforceSoftLimit.Value,
            AutoCheck = false,
            ForeColor = Color.SeaGreen
        };
        _chkEnforceSoft.Click += (_, _) =>
        {
            _chkEnforceSoft.Checked = !_chkEnforceSoft.Checked;
            _config.Waveform.EnforceSoftLimit.Value = _chkEnforceSoft.Checked;
            _chkEnforceSoft.ForeColor = _chkEnforceSoft.Checked ? Color.SeaGreen : Color.Firebrick;
            ValidateNow();
        };
        _tip.SetToolTip(_chkEnforceSoft, "开启：运动碰到软限位立即停止并提示触碰限位。\n关闭：忽略软限位，五种波形运动坚持把整个轨迹做完（启动前不再拦超行程，运行中限位处不停；硬限位保护仍然有效，请确认机械行程安全）。");
        group.Controls.Add(_chkEnforceSoft);

        // ---- 模式分页 ----
        _tabs = new TabControl { Location = new Point(10, 52), Size = new Size(460, 176), Font = this.Font };
        for (int m = 0; m < ModeNames.Length; m++)
            _tabs.TabPages.Add(BuildModeTab((WaveformMode)m, ModeNames[m]));
        _tabs.SelectedIndex = Math.Clamp(_config.Waveform.GetMode(), 0, 4);
        _tabs.SelectedIndexChanged += (_, _) => ValidateNow();
        group.Controls.Add(_tabs);

        // ---- 启停 / 缩放 / 预览 ----
        _btnWfStart = new Button { Text = "启动波形", Location = new Point(10, 234), Size = new Size(96, 30), Enabled = false };
        _btnWfStart.Click += BtnWfStart_Click;
        group.Controls.Add(_btnWfStart);

        _btnWfStop = new Button { Text = "停止", Location = new Point(112, 234), Size = new Size(56, 30), Enabled = false, BackColor = Color.LightCoral };
        _btnWfStop.Click += (_, _) => SafeRun(() => _device?.StopWaveform(), ShowErr);
        group.Controls.Add(_btnWfStop);

        _btnWfScale = new Button { Text = "一键缩放至安全", Location = new Point(174, 234), Size = new Size(104, 30), Enabled = false };
        _btnWfScale.Click += BtnWfScale_Click;
        group.Controls.Add(_btnWfScale);

        _btnWfPreview = new Button { Text = "轨迹预览", Location = new Point(284, 234), Size = new Size(82, 30) };
        _btnWfPreview.Click += BtnWfPreview_Click;
        group.Controls.Add(_btnWfPreview);

        _btnWfLive = new Button { Text = "实时曲线", Location = new Point(372, 234), Size = new Size(82, 30) };
        _btnWfLive.Click += BtnWfLive_Click;
        group.Controls.Add(_btnWfLive);

        // ---- 实时读数 ----
        group.Controls.Add(new Label { Text = "速度", Location = new Point(10, 268), Size = new Size(36, 22), TextAlign = ContentAlignment.MiddleRight, ForeColor = Color.DimGray });
        _lblRdVel = new Label { Location = new Point(48, 268), Size = new Size(104, 22), Text = "-- mm/s", Font = new Font("Consolas", 10) };
        group.Controls.Add(_lblRdVel);
        group.Controls.Add(new Label { Text = "加速度", Location = new Point(158, 268), Size = new Size(48, 22), TextAlign = ContentAlignment.MiddleRight, ForeColor = Color.DimGray });
        _lblRdAccel = new Label { Location = new Point(208, 268), Size = new Size(128, 22), Text = "-- mm/s2", Font = new Font("Consolas", 10) };
        group.Controls.Add(_lblRdAccel);
        group.Controls.Add(new Label { Text = "位置", Location = new Point(340, 268), Size = new Size(36, 22), TextAlign = ContentAlignment.MiddleRight, ForeColor = Color.DimGray });
        _lblRdPos = new Label { Location = new Point(378, 268), Size = new Size(92, 22), Text = "-- mm", Font = new Font("Consolas", 10), TextAlign = ContentAlignment.MiddleRight };
        group.Controls.Add(_lblRdPos);

        // ---- 校验 / 合成预览结果 ----
        _lblWfPeakV = new Label { Location = new Point(10, 294), Size = new Size(228, 22), Text = "峰值速度：--" };
        _lblWfStroke = new Label { Location = new Point(244, 294), Size = new Size(226, 22), Text = "行程：--" };
        _lblWfPeakA = new Label { Location = new Point(10, 318), Size = new Size(460, 22), Text = "峰值加速度：--" };
        group.Controls.Add(_lblWfPeakV);
        group.Controls.Add(_lblWfStroke);
        group.Controls.Add(_lblWfPeakA);

        _lblWfStatus = new Label { Location = new Point(10, 344), Size = new Size(460, 52), ForeColor = Color.DimGray, Text = "提示：改参数即自动时域校验，超限禁止启动；悬停参数可查看说明；关闭“软限位保护”后超行程不再拦，坚持把运动做完。" };
        group.Controls.Add(_lblWfStatus);

        Controls.Add(group);
        ValidateNow();
    }

    /// <summary>构建一个模式页：普通字段两列排布，宽文本字段整行置于下方；标签/输入框悬停显示参数说明。</summary>
    private TabPage BuildModeTab(WaveformMode mode, string title)
    {
        var page = new TabPage(title) { Font = this.Font };
        var dict = _modeInputs[(int)mode];
        var defs = ParamDefs(mode).ToList();

        int y = 6;
        const int colGap = 226, rowH = 22, baseX = 6;
        int col = 0;
        foreach (var (label, key, initial, wide, hint) in defs)
        {
            if (wide) continue;
            int x = baseX + col * colGap;
            var lbl = new Label { Text = label, Location = new Point(x, y + 1), Size = new Size(120, 20), TextAlign = ContentAlignment.MiddleRight };
            if (hint.Length > 0) _tip.SetToolTip(lbl, hint);
            page.Controls.Add(lbl);
            var tb = new TextBox { Location = new Point(x + 124, y), Size = new Size(col == 0 ? 92 : 96, 21), Text = initial };
            tb.TextChanged += (_, _) =>
            {
                if (_suppressSineLinkage) return;                 // 忽略联动回填触发的程序性 TextChanged
                if (key is "f" or "S" or "An" or "amax") _sineEditKey = key;  // 记住用户直接编辑的耦合字段
                ValidateNow();
            };
            if (hint.Length > 0) _tip.SetToolTip(tb, hint);
            page.Controls.Add(tb);
            dict[key] = tb;
            if (++col == 2) { col = 0; y += rowH; }
        }
        if (col == 1) y += rowH;   // 奇数个普通字段换行

        foreach (var (label, key, initial, wide, hint) in defs)
        {
            if (!wide) continue;
            var lblW = new Label { Text = label, Location = new Point(baseX, y), Size = new Size(444, 18), ForeColor = Color.DimGray };
            if (hint.Length > 0) _tip.SetToolTip(lblW, hint);
            page.Controls.Add(lblW);
            y += 20;
            var tbW = new TextBox { Location = new Point(baseX, y), Size = new Size(438, 24), Text = initial };
            tbW.TextChanged += (_, _) => { if (!_suppressSineLinkage) ValidateNow(); };
            if (hint.Length > 0) _tip.SetToolTip(tbW, hint);
            page.Controls.Add(tbW);
            dict[key] = tbW;
            y += 30;
        }
        return page;
    }

    // (标签, 键, 初始值, 是否宽文本框, 悬停说明)
    private IEnumerable<(string Label, string Key, string Initial, bool Wide, string Hint)> ParamDefs(WaveformMode mode)
    {
        var w = _config.Waveform;
        string N(double v) => v.ToString("0.###", Inv);
        switch (mode)
        {
            case WaveformMode.Sine:
                double omega0 = 2 * Math.PI * w.SineFreqHz.Value;
                yield return ("频率 f (Hz)", "f", N(w.SineFreqHz.Value), false, "正弦速度频率 f：与行程幅值 S 共同决定峰值速度/加速度（An=2πf·S，amax=(2πf)²·S），四个量相互约束，改任一个其他会跟着变。受控制周期限制：f·dt 建议 ≤0.05（如 dt=10ms 时 f 建议 ≤5Hz）。");
                yield return ("行程幅值 S (mm)", "S", N(w.SineStrokeMm.Value), false, "单边幅值 S：位置在“S·cosψ−S”到“S·cosψ+S”之间摆动（ψ=±90° 时才是关于 0 对称的 [−S,+S]，ψ=0° 时实际是 [0,2S]），峰峰值=2S。S 需 ≤ 软限位窗口宽度，否则超行程（关闭软限位保护后不再拦，但会冲出限位窗，注意机械行程安全）。");
                yield return ("峰值速度 An (mm/s)", "An", N(omega0 * w.SineStrokeMm.Value), false, "速度正弦峰值 An=2πf·S，对应需求文档中的“正弦转速幅值”。可直接修改此框，系统会反推行程幅值 S（保持 f 不变）。");
                yield return ("峰值加速度 amax (mm/s²)", "amax", N(omega0 * omega0 * w.SineStrokeMm.Value), false, "加速度峰值 amax=(2πf)²·S。可直接修改此框，系统会反推行程幅值 S（保持 f 不变），并与“加速度上限”校验项配合判限。");
                yield return ("速度偏置 n0 (mm/s)", "n0", N(w.SineBiasMmS.Value), false, "叠加在正弦上的恒速偏置 n0：>0 整体向正方向漂移，轨迹不再对称于 0 点，容易单侧冲出软限位；一般保持 0。");
                yield return ("初相位 (度)", "psi", N(w.SinePhaseDeg.Value), false, "初相位 φ（度）：决定 t=0 时刻在正弦周期中的位置，只平移时间轴，不改变波形形状与行程（但会改变行程区间相对 0 点的对称中心，见行程幅值说明）。");
                yield return ("总时长 T (s)", "T", N(w.SineDurationS.Value), false, "总运行时长 T（秒），到时自动停止。T·f 即周期数量。");
                yield return ("周期间歇 t (s)", "t", N(w.SineDwellS.Value), false, "每个正弦周期结束后的零速停顿 t（秒），0=连续运行不间歇。");
                break;
            case WaveformMode.MultiSine:
                yield return ("基频 f0 (Hz)", "f0", N(w.MsBaseFreqHz.Value), false, "基频 f0：第 k 个分量频率 fk=k·f0（整数倍保证合成周期信号、位置不漂移），合成主周期 =1/f0。");
                yield return ("成分数 N (16~32)", "N", w.MsCount.Value.ToString(Inv), false, "叠加的正弦分量个数 N（代码上限 64）：越多波形越复杂；N 大时同相叠加峰值≈ΣAk，易超速/超行程。");
                yield return ("基频幅值 A1 (mm/s)", "A1", N(w.MsFundAmpMmS.Value), false, "基频分量（k=1）的速度幅值 A1（mm/s）；第 k 分量幅值 Ak=A1/k^p。注意是速度幅值不是位置幅值。");
                yield return ("幅值衰减 p", "p", N(w.MsDecayP.Value), false, "幅值衰减指数 p：高频分量按 Ak=A1/k^p 衰减。p 大→低频主导波形平滑；p 小→高频成分多、波形毛刺多。");
                yield return ("相位策略 (0/1/2)", "phase", w.MsPhaseMode.Value.ToString(Inv), false, "各分量初相策略：0=全部同相（峰值最大≈ΣAk，最易相长干涉）；1=奇偶交替 0/π；2=按随机种子取 0~2π（更接近随机频谱）。");
                yield return ("随机种子", "seed", w.MsSeed.Value.ToString(Inv), false, "随机种子：相同种子+相同参数=完全相同的波形（可复现）；改种子换一条相位序列。");
                yield return ("总时长 T (s)", "T", N(w.MsDurationS.Value), false, "总运行时长 T（秒），到时自动停止。");
                yield return ("显式分量表 f,A,相位;...(留空=按规则生成, 相位单位度)", "table", w.MsTable.Value, true, "显式分量表，每行“频率Hz,速度幅值mm/s,初相度”，分号分隔，如 0.2,30,0;0.6,15,45。填了则覆盖基频/成分数/衰减/相位策略等规则参数；清空回到规则生成。");
                break;
            case WaveformMode.Square:
                yield return ("循环次数 n", "n", w.SqCycles.Value.ToString(Inv), false, "整条途经点表循环执行 n 遍；每遍依次到达每个途经点并停顿。最后一遍结束后停在最后一个途经点。");
                yield return ("减速度 (mm/s²)", "decel", N(w.SqDecelMmS2.Value), false, "方波运行期控制器的减速度上限（mm/s²）：到点/方向切换/停止时按此值减速。阶梯型波形的实际加减速由控制器钳制，此值不参与时域波形判限；仅本模式生效，关闭界面自动保存、下次打开自动读取。");
                yield return ("途经点 P,v,t;... (mm,mm/s,t=到点停顿s)", "wps", w.SqWaypoints.Value, true, "途经点序列“P,v,t”分号分隔：目标位置 P(mm，相对启动时原地置零后的 0 点)、逼近速度 v(mm/s)、到点后停顿 t(s)。位置反馈判到达，途经点 P 必须落在软限位窗内（关闭软限位保护后限位处不停，注意机械行程）。");
                break;
            case WaveformMode.Pulse:
                yield return ("脉冲速度 v1 (mm/s)", "v1", N(w.PuSpeedMmS.Value), false, "脉冲速度 v1：每个脉冲前半 +v1 冲出、后半 −v1 回基位，净位移 0。单个脉冲最大偏离 = v1×ti/2。");
                yield return ("循环次数 n", "n", w.PuCycles.Value.ToString(Inv), false, "整个脉冲序列（含所有宽度和间隔）循环执行 n 遍。");
                yield return ("脉冲间隔 gap (s)", "gap", N(w.PuGapS.Value), false, "相邻脉冲间的零速间隔 gap（秒），0=脉冲背靠背连续执行。");
                yield return ("减速度 (mm/s²)", "decel", N(w.PuDecelMmS2.Value), false, "脉冲运行期控制器的减速度上限（mm/s²）：每脉冲回基位/换向时按此值减速。阶梯型波形由控制器钳制，此值不参与时域判限；仅本模式生效，关闭界面自动保存、下次打开自动读取。");
                yield return ("脉冲宽度 t1,t2,t3,t4 (s)", "durs", w.PuDurations.Value, true, "脉冲宽度序列“t1,t2,...”逗号分隔（秒）：每个 ti 拆成前半加速、后半回基位，按顺序执行并整体循环 n 次。");
                break;
            case WaveformMode.Prts:
                yield return ("速度幅值 V (mm/s)", "V", N(w.PrtsV.Value), false, "三态速度幅值 V：速度在 +V / 0 / −V 三个状态间伪随机切换（每态连续保持 K 拍）。");
                yield return ("每态保持 K 拍", "K", w.PrtsK.Value.ToString(Inv), false, "每个速度状态连续保持 K 个控制拍（K×dt 秒）后才允许切换，越大运动越平缓。");
                yield return ("位置波动 S (mm)", "S", N(w.PrtsS.Value), false, "位置波动约束 ±S（mm）：序列生成时前瞻约束保证位置始终在 [−S, +S] 内且周期末归零。S 需 ≤ 软限位窗半宽（关闭软限位保护后不再拦）。");
                yield return ("总时长 T (s)", "T", N(w.PrtsDurationS.Value), false, "总运行时长 T（秒）：到时后自动追加回零段使末端位置≈ 0。");
                yield return ("随机种子", "seed", w.PrtsSeed.Value.ToString(Inv), false, "固定种子离线生成整条速度序列：相同参数+种子每次轨迹完全一致（可复现）。");
                yield return ("减速度 (mm/s²)", "decel", N(w.PrtsDecelMmS2.Value), false, "PRTS 运行期控制器的减速度上限（mm/s²）：速度态切换/回零时按此值减速。阶梯型波形由控制器钳制，此值不参与时域判限；仅本模式生效，关闭界面自动保存、下次打开自动读取。");
                yield return ("显式逐拍速度序列 +/0/- 或mm/s(留空=按规则生成)", "table", w.PrtsTable.Value, true, "逐拍指定速度：每个 token 一拍，用逗号/空格/分号分隔；token 写 + / - / 0 代表±V与零速（V 取上方“速度幅值”），也可直接写 mm/s 数值（如 +100,-100,0）。非空则覆盖 K/S/T/种子等规则参数，逐拍原样下发（不前瞻、不回零）；位置=序列积分，超行程由校验器判限，如需末尾回零请自行在尾部补反向拍。清空回到规则生成。");
                break;
        }
    }

    // ==================== 提交输入 -> 校验 -> 启停 ====================

    /// <summary>把界面值写回 config（best-effort），再跑时域校验/合成预览并刷新结果标签。</summary>
    private void ValidateNow()
    {
        bool ok = CommitInputsToConfig(out string err);
        if (!ok)
        {
            _lblWfStatus.Text = "参数格式错误：" + err;
            _lblWfStatus.ForeColor = Color.Firebrick;
            _btnWfStart.Enabled = false;
            _btnWfScale.Enabled = false;
            return;
        }

        try
        {
            var rt = WaveformFactory.Build(_config);
            var result = WaveformValidator.Simulate(rt.Generator, rt.DtSec, rt.Limits);
            _wfLastResult = result;

            _lblWfPeakV.Text = $"峰值速度：{result.PeakVelocityMmS:F0} mm/s (限 {MotionUnits.VmaxMmS:F0})";
            _lblWfStroke.Text = $"行程：[{result.MinPosMm:F1}, {result.MaxPosMm:F1}] mm";
            _lblWfPeakA.Text = rt.Generator.ContinuousVelocity
                ? $"峰值加速度：{result.PeakAccelMmS2:F0} mm/s2"
                : $"速度阶跃率：{result.PeakAccelMmS2:F0} mm/s2（阶梯型仅参考，实际由控制器加减速钳制，不判限）";

            if (result.Ok)
            {
                // 软限位保护关闭且轨迹冲出限位窗：不拦启动，但必须醒目提醒机械行程风险
                MotionLimits lm = rt.Limits;
                bool overStroke = !_config.Waveform.GetEnforceSoftLimit() &&
                    (result.MaxPosMm > lm.PosMm + 1e-6 || result.MinPosMm < lm.NegMm - 1e-6);
                if (overStroke)
                {
                    _lblWfStatus.Text = $"可启动（预估 {result.SimulatedSeconds:F1}s）。注意：软限位保护已关闭，行程 [{result.MinPosMm:F1}, {result.MaxPosMm:F1}] 冲出限位窗 [{lm.NegMm:F1}, {lm.PosMm:F1}]，运动将坚持把波形做完，请确认硬限位/机械行程安全！";
                    _lblWfStatus.ForeColor = Color.DarkOrange;
                }
                else
                {
                    _lblWfStatus.Text = $"校验通过，可启动（预估时长 {result.SimulatedSeconds:F1}s）";
                    _lblWfStatus.ForeColor = Color.SeaGreen;
                }
                _btnWfStart.Enabled = _device is { IsConnected: true } && !_device.IsWaveformRunning;
                _btnWfScale.Enabled = false;
            }
            else
            {
                _lblWfStatus.Text = "超限：" + string.Join("；", result.Violations);
                _lblWfStatus.ForeColor = Color.Firebrick;
                _btnWfStart.Enabled = false;
                _btnWfScale.Enabled = result.SuggestedScale > 0 && result.SuggestedScale < 1;
            }
        }
        catch (Exception ex)
        {
            _lblWfStatus.Text = "校验失败：" + ex.Message;
            _lblWfStatus.ForeColor = Color.Firebrick;
            _btnWfStart.Enabled = false;
            _btnWfScale.Enabled = false;
        }
    }

    private void BtnWfStart_Click(object? sender, EventArgs e)
    {
        if (_device == null || !_device.IsConnected)
        {
            TipForm.Show(this, "未连接，无法启动波形", false, 2500);
            return;
        }
        if (!CommitInputsToConfig(out string err)) { TipForm.Show(this, err, false, 3000); return; }

        WaveformRuntime rt;
        try
        {
            rt = WaveformFactory.Build(_config);
            var result = WaveformValidator.Simulate(rt.Generator, rt.DtSec, rt.Limits);
            if (!result.Ok)
            {
                TipForm.Show(this, "超限，禁止启动：" + string.Join("；", result.Violations), false, 4000);
                ValidateNow();
                return;
            }
        }
        catch (Exception ex)
        {
            TipForm.Show(this, "启动失败：" + ex.Message, false, 4000);
            return;
        }

        // 开环模式（单正弦/多正弦/脉冲/PRTS）必须从坐标 0 点起走，否则轨迹整体偏移；方波为绝对位置闭环无需归零。
        // 触碰软限位停止后轴常停在限位处，故启动前先真实归零回伺服 0 点（MoveToServoZero 已在 0 附近会立即返回）。
        bool needHome = ActiveMode() != WaveformMode.Square && _device is ZMotionDeviceController;

        // 锁定按钮并后台执行“归0 → 启动”（归零是阻塞式真实运动，不能卡 UI 线程）
        _btnWfStart.Enabled = false;
        _btnWfStop.Enabled = false;
        _btnWfStart.Text = needHome ? "归0中…" : "启动中…";
        ConfigService.TrySave(_config);
        var dev = _device;

        Task.Run(() =>
        {
            try
            {
                if (needHome && dev is ZMotionDeviceController zmc)
                    zmc.MoveToServoZero();   // 真实归零：轴回到坐标 0（伺服保存的 0 点）

                dev.StartWaveform(rt);
                if (IsDisposed) return;
                BeginInvoke(() =>
                {
                    _btnWfStart.Text = "启动波形";
                    // 极短波形可能已跑完并先触发 WaveformStopped，此处按当前运行态设“停止”按钮，避免错乱
                    _btnWfStop.Enabled = dev.IsWaveformRunning;
                    TipForm.Show(this, $"波形已启动：{rt.Generator.ModeName}", true, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
                });
            }
            catch (Exception ex)
            {
                if (IsDisposed) return;
                BeginInvoke(() =>
                {
                    _btnWfStart.Text = "启动波形";
                    TipForm.Show(this, "启动失败：" + ex.Message, false, 4000);
                    ValidateNow();   // 恢复按钮可用状态
                });
            }
        });
    }

    private void BtnWfScale_Click(object? sender, EventArgs e)
    {
        if (_wfLastResult == null || _wfLastResult.SuggestedScale <= 0 || _wfLastResult.SuggestedScale >= 1) return;
        double s = _wfLastResult.SuggestedScale;
        var w = _config.Waveform;
        switch (ActiveMode())
        {
            case WaveformMode.Sine: w.SineStrokeMm.Value *= s; break;
            case WaveformMode.MultiSine:
                w.MsFundAmpMmS.Value *= s;   // 规则法基频幅值
                // 显式分量表非空时表覆盖规则，必须同步缩放表内各分量幅值，否则点了不生效
                if (!string.IsNullOrWhiteSpace(w.MsTable.Value))
                    w.MsTable.Value = WaveformFactory.ScaleComponentTable(w.MsTable.Value, s);
                break;
            case WaveformMode.Pulse: w.PuSpeedMmS.Value *= s; break;
            case WaveformMode.Prts: w.PrtsV.Value *= s; break;
            default: return;   // 方波不自动缩放
        }
        RefreshActiveTab();   // 用新 config 回填输入框并重新校验
    }

    /// <summary>轨迹预览：离线仿真逐拍序列画双联图（上速度/下位置），不依赖硬件连接。</summary>
    private void BtnWfPreview_Click(object? sender, EventArgs e)
    {
        if (!CommitInputsToConfig(out string err)) { TipForm.Show(this, err, false, 3000); return; }
        try
        {
            var rt = WaveformFactory.Build(_config);
            var series = WaveformSampler.Capture(rt.Generator, rt.DtSec, rt.Limits);
            new WaveformPreviewForm(series).Show(this);   // 非模式，属主为主窗体
        }
        catch (Exception ex)
        {
            TipForm.Show(this, "预览失败：" + ex.Message, false, 4000);
        }
    }

    /// <summary>实时曲线：订阅设备逐拍事件，运行中自动刷新；可先开窗再启动波形。</summary>
    private void BtnWfLive_Click(object? sender, EventArgs e)
    {
        if (_device == null || !_device.IsConnected)
        {
            TipForm.Show(this, "未连接，无实时数据源", false, 2500);
            return;
        }
        try
        {
            var rt = WaveformFactory.Build(_config);
            WaveformPreviewForm.CreateLive(_device, rt.DtSec, rt.Limits, rt.Generator.ModeName).Show(this);
        }
        catch (Exception ex)
        {
            TipForm.Show(this, "打开实时曲线失败：" + ex.Message, false, 4000);
        }
    }

    /// <summary>把 config 值回填到当前模式页的输入框（缩放后刷新用）。</summary>
    private void RefreshActiveTab()
    {
        var dict = _modeInputs[(int)ActiveMode()];
        _suppressSineLinkage = true;   // 切页/复位回填：不视为用户编辑，不改动 _sineEditKey
        try
        {
            foreach (var (_, key, initial, _, _) in ParamDefs(ActiveMode()))
                if (dict.TryGetValue(key, out var tb)) tb.Text = initial;
        }
        finally { _suppressSineLinkage = false; }
    }

    // ==================== 输入 <-> config ====================

    private bool CommitInputsToConfig(out string error)
    {
        error = "";
        var w = _config.Waveform;
        w.Mode.Value = Math.Clamp(_tabs.SelectedIndex, 0, 4);
        var mode = (WaveformMode)w.Mode.Value;
        var dict = _modeInputs[(int)mode];

        // 公共参数
        if (!double.TryParse(_txtWfDt.Text, NumberStyles.Float, Inv, out double dtms) || dtms < 1)
        { error = "控制周期必须 >= 1ms"; return false; }
        w.ControlPeriodMs.Value = (int)Math.Round(dtms);
        if (!double.TryParse(_txtWfAccel.Text, NumberStyles.Float, Inv, out double accel))
        { error = "加速度上限格式错误"; return false; }
        w.AccelLimitMmS2.Value = accel;

        bool D(string key, out double v) { v = 0; return dict.TryGetValue(key, out var tb) && double.TryParse(tb.Text, NumberStyles.Float, Inv, out v); }
        bool I(string key, out int v) { v = 0; return dict.TryGetValue(key, out var tb) && int.TryParse(tb.Text.Trim(), out v); }
        string Get(string key) => dict.TryGetValue(key, out var tb) ? tb.Text.Trim() : "";

        switch (mode)
        {
            case WaveformMode.Sine:
                if (!D("f", out double f) || f <= 0) { error = "频率格式错误"; return false; }
                double omega = 2 * Math.PI * f;
                // f/S/An/amax 四量耦合（An=ω·S，amax=ω²·S）：以用户最近直接编辑的一项为基准反推其余，
                // 保持 f 不变（编辑 f 时则保持 S 不变，An/amax 跟随重算）。
                double S;
                switch (_sineEditKey)
                {
                    case "An":
                        if (!D("An", out double anIn) || anIn <= 0) { error = "峰值速度格式错误"; return false; }
                        S = anIn / omega;
                        break;
                    case "amax":
                        if (!D("amax", out double amaxIn) || amaxIn <= 0) { error = "峰值加速度格式错误"; return false; }
                        S = amaxIn / (omega * omega);
                        break;
                    default: // "f" 或 "S"：以行程幅值 S 为基准
                        if (!D("S", out double sIn) || sIn <= 0) { error = "行程幅值格式错误"; return false; }
                        S = sIn;
                        break;
                }
                if (!D("n0", out double n0)) { error = "偏置格式错误"; return false; }
                if (!D("psi", out double psi)) { error = "初相位格式错误"; return false; }
                if (!D("T", out double T) || T <= 0) { error = "总时长格式错误"; return false; }
                if (!D("t", out double t)) { error = "间歇格式错误"; return false; }
                w.SineFreqHz.Value = f; w.SineStrokeMm.Value = S; w.SineBiasMmS.Value = n0;
                w.SinePhaseDeg.Value = psi; w.SineDurationS.Value = T; w.SineDwellS.Value = Math.Max(0, t);
                // 反算派生量并回填文本框（跳过用户正在编辑的那一格，避免打断输入；抑制联动防重入）
                double anOut = omega * S, amaxOut = omega * omega * S;
                string NF(double v) => v.ToString("0.###", Inv);
                void SetBox(string k, string val)
                {
                    if (k == _sineEditKey) return;
                    if (dict.TryGetValue(k, out var box) && box.Text != val) box.Text = val;
                }
                _suppressSineLinkage = true;
                try
                {
                    SetBox("S", NF(S)); SetBox("An", NF(anOut)); SetBox("amax", NF(amaxOut));
                }
                finally { _suppressSineLinkage = false; }
                break;

            case WaveformMode.MultiSine:
                if (!D("f0", out double f0) || f0 <= 0) { error = "基频格式错误"; return false; }
                if (!I("N", out int N) || N < 1 || N > 64) { error = "成分数必须 1~64"; return false; }
                if (!D("A1", out double A1) || A1 <= 0) { error = "基频幅值格式错误"; return false; }
                if (!D("p", out double p)) { error = "衰减指数格式错误"; return false; }
                if (!I("phase", out int phase)) phase = 0;
                if (!I("seed", out int seed)) seed = 0;
                if (!D("T", out double Tm) || Tm <= 0) { error = "总时长格式错误"; return false; }
                w.MsBaseFreqHz.Value = f0; w.MsCount.Value = N; w.MsFundAmpMmS.Value = A1;
                w.MsDecayP.Value = p; w.MsPhaseMode.Value = Math.Clamp(phase, 0, 2);
                w.MsSeed.Value = seed; w.MsDurationS.Value = Tm; w.MsTable.Value = Get("table");
                break;

            case WaveformMode.Square:
                if (!I("n", out int n) || n < 1) { error = "循环次数格式错误"; return false; }
                if (!D("decel", out double sqDecel) || sqDecel < 0) { error = "减速度格式错误"; return false; }
                w.SqCycles.Value = n; w.SqWaypoints.Value = Get("wps"); w.SqDecelMmS2.Value = sqDecel;
                break;

            case WaveformMode.Pulse:
                if (!D("v1", out double v1) || v1 <= 0) { error = "脉冲速度格式错误"; return false; }
                if (!I("n", out int np) || np < 1) { error = "循环次数格式错误"; return false; }
                if (!D("gap", out double gap)) { error = "间隔格式错误"; return false; }
                if (!D("decel", out double puDecel) || puDecel < 0) { error = "减速度格式错误"; return false; }
                w.PuSpeedMmS.Value = v1; w.PuCycles.Value = np; w.PuGapS.Value = Math.Max(0, gap);
                w.PuDurations.Value = Get("durs"); w.PuDecelMmS2.Value = puDecel;
                break;

            case WaveformMode.Prts:
                if (!D("V", out double V) || V <= 0) { error = "速度幅值格式错误"; return false; }
                if (!I("K", out int K) || K < 1) { error = "每态保持拍数格式错误"; return false; }
                if (!D("S", out double Sp) || Sp <= 0) { error = "位置波动格式错误"; return false; }
                if (!D("T", out double Tp) || Tp <= 0) { error = "总时长格式错误"; return false; }
                if (!I("seed", out int seeds)) seeds = 0;
                if (!D("decel", out double prtsDecel) || prtsDecel < 0) { error = "减速度格式错误"; return false; }
                w.PrtsV.Value = V; w.PrtsK.Value = K; w.PrtsS.Value = Sp;
                w.PrtsDurationS.Value = Tp; w.PrtsSeed.Value = seeds; w.PrtsDecelMmS2.Value = prtsDecel;
                w.PrtsTable.Value = Get("table");
                break;
        }

        return true;
    }

    // ==================== 连接状态 / 实时读数 ====================

    private void SetWaveformConnected(bool connected)
    {
        if (_device != null)
        {
            if (connected && !_wfSubscribed)
            {
                _device.WaveformTick += Device_WaveformTick;
                _device.WaveformStopped += Device_WaveformStopped;
                _wfSubscribed = true;
            }
            else if (!connected && _wfSubscribed)
            {
                _device.WaveformTick -= Device_WaveformTick;
                _device.WaveformStopped -= Device_WaveformStopped;
                _wfSubscribed = false;
            }
        }

        _btnWfStop.Enabled = connected && (_device?.IsWaveformRunning ?? false);
        if (connected) ValidateNow();
        else { _btnWfStart.Enabled = false; _btnWfScale.Enabled = false; }
    }

    private void Device_WaveformTick(object? sender, WaveformTickEventArgs e)
    {
        if (IsDisposed || !IsHandleCreated) return;
        BeginInvoke(() =>
        {
            _lblRdVel.Text = $"{e.VelMmS:F0} mm/s";
            _lblRdPos.Text = $"{e.PosMm:F1} mm";
            UpdateAccelReadout(e.VelMmS);
        });
    }

    /// <summary>波形停止（完成/用户停/触碰软限位）：复位“启动波形”按钮并按原因提示。后台线程触发， marshal 回 UI。</summary>
    private void Device_WaveformStopped(object? sender, WaveformStopReason reason)
    {
        if (IsDisposed || !IsHandleCreated) return;
        BeginInvoke(() =>
        {
            _btnWfStart.Text = "启动波形";
            // 复位按钮：重新启用“启动波形”、禁用“停止”（SetWaveformConnected → ValidateNow 依据连接与运行状态设置）
            if (_device != null) SetWaveformConnected(_device.IsConnected);
            switch (reason)
            {
                case WaveformStopReason.SoftLimit:
                    TipForm.Show(this, "触碰软限位，运动停止", false, 3500);
                    break;
                case WaveformStopReason.Completed:
                    TipForm.Show(this, "波形已完成", true, (int)(_config.Ui.GetTipDisplaySeconds() * 1000));
                    break;
                    // UserStopped：用户主动停止，无需额外提示
            }
        });
    }

    /// <summary>轴轮询里的实时读数刷新（非波形时也显示真实速度/位置）。</summary>
    private void UpdateWaveformReadouts()
    {
        if (_device is not ZMotionDeviceController zmc || !zmc.IsConnected) return;
        if (_device.IsWaveformRunning) return;   // 波形中由 WaveformTick 驱动，避免重复
        try
        {
            float vUnits = zmc.GetCurrentVpSpeedUnits();
            double vMmS = FromUnits(vUnits);    // user units/s -> mm/s（FromUnits 已返回 mm）
            _lblRdVel.Text = $"{vMmS:F0} mm/s";
            _lblRdPos.Text = $"{FromUnits(zmc.GetCurrentDpos()):F1} mm";
            UpdateAccelReadout(vMmS);
        }
        catch { /* 读数失败下一轮重试 */ }
    }

    private void UpdateAccelReadout(double vMmS)
    {
        var now = DateTime.UtcNow;
        if (_rdLastT != DateTime.MinValue)
        {
            double dt = (now - _rdLastT).TotalSeconds;
            if (dt > 1e-6)
                _lblRdAccel.Text = $"{(vMmS - _rdLastV) / dt:F0} mm/s2";
        }
        _rdLastV = vMmS;
        _rdLastT = now;
    }
}
