using System.Globalization;
using MotorControlApp.Devices;
using MotorControlApp.Waveforms;

namespace MotorControlApp.Forms;

/// <summary>
/// 轨迹预览窗口：离线仿真逐拍序列双联图（上=速度 v(t)，下=积分位置 p(t)）。
/// 纯 GDI 绘制：橙色速度阶梯 + 青色位置阶梯(带填充)，位置窗含零线、软限位虚线与网格。
/// 数据来自 WaveformSampler（与启动校验同一积分回路），不依赖硬件连接。Esc 或关窗退出。
/// 横轴固定时间比例尺（一屏 10s，PRTS 一屏 40 拍），不随运行时长压缩：
/// 窗体底部拖拽条左右平移视图窗口；实时模式运行中自动跟随最新数据并锁定拖拽，结束后拖拽回看历史段。
/// </summary>
public sealed class WaveformPreviewForm : Form
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private static readonly Color VelColor = Color.FromArgb(230, 80, 30);     // 速度：橙红
    private static readonly Color PosColor = Color.FromArgb(0, 150, 136);     // 位置：青
    private static readonly Color PosFillColor = Color.FromArgb(50, 0, 150, 136);
    private static readonly Color GridColor = Color.FromArgb(228, 228, 228);
    private static readonly Color LimitColor = Color.FromArgb(200, 60, 60);

    private const int MarginL = 64, MarginR = 16, MarginT = 46, MarginB = 30, PaneGap = 30;
    private const int TargetPoints = 4000;   // 抽稀目标点数（保 min/max 不丢台阶沿）
    private const int ScrubH = 38;           // 底部拖拽条预留高度
    private const double WindowSec = 10.0;   // 固定比例尺：非 PRTS 一屏 10s
    private const int PrtsWindowBeats = 40;  // 固定比例尺：PRTS 一屏 40 拍

    private WaveformSeries _s;
    private readonly Font _smallFont;

    // ---- 固定比例尺视图窗口 + 底部拖拽条 ----
    private readonly TrackBar _scrub = null!;
    private bool _follow = true;      // 跟随最新数据（视图钉在右缘）
    private bool _liveRunning;        // 波形运行中：拖拽条禁用
    private bool _syncScrub;          // 程序同步滑块值时抑制 ValueChanged 的“用户拖拽”判定

    // ---- 实时模式：订阅设备 WaveformTick，计时器抽帧刷新 ----
    private readonly IDeviceController? _liveDevice;
    private readonly object _liveLock = new();
    private readonly List<double> _lt = new(), _lv = new(), _lp = new();
    private readonly System.Windows.Forms.Timer? _liveTimer;
    private double _liveLastT = -1;

    /// <summary>静态预览构造：一次性传入离线仿真序列。</summary>
    public WaveformPreviewForm(WaveformSeries series) : this(series, null, 0.01, null)
    {
    }

    /// <summary>实时曲线构造：订阅 device.WaveformTick，约 10fps 抽帧重绘；新序列 t 归零时自动清空重画。</summary>
    public static WaveformPreviewForm CreateLive(IDeviceController device, double dtSec, MotionLimits? limits, string modeName)
    {
        var empty = new WaveformSeries
        {
            TimeSec = Array.Empty<double>(), VelMmS = Array.Empty<double>(), PosMm = Array.Empty<double>(),
            DtSec = dtSec, Converged = true, ModeName = modeName, Limits = limits,
        };
        return new WaveformPreviewForm(empty, device, dtSec, limits);
    }

    private WaveformPreviewForm(WaveformSeries series, IDeviceController? device, double dtSec, MotionLimits? limits)
    {
        _s = series;
        _liveDevice = device;
        Text = device is null ? $"轨迹预览 - {series.ModeName}" : $"轨迹预览 - 实时（{series.ModeName}），等待波形拍数据…";
        Font = SystemFonts.MessageBoxFont;   // 含中文字形
        _smallFont = new Font(Font.FontFamily, 7.5f);
        ClientSize = new Size(900, 600);
        MinimumSize = new Size(520, 400);
        StartPosition = FormStartPosition.CenterParent;
        ShowInTaskbar = false;
        BackColor = Color.White;
        KeyPreview = true;
        KeyDown += (_, ev) => { if (ev.KeyCode == Keys.Escape) Close(); };
        SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);
        UpdateStyles();

        // 底部拖拽条：固定比例尺下左右平移视图；实时运行中锁定，结束后解禁回看
        _scrub = new TrackBar
        {
            Dock = DockStyle.Bottom,
            TickStyle = TickStyle.None,
            AutoSize = false,
            Height = 34,
            Enabled = false,
        };
        _scrub.ValueChanged += (_, _) => { if (!_syncScrub) _follow = false; Invalidate(); };
        Controls.Add(_scrub);

        if (_liveDevice is not null)
        {
            _follow = true;   // 运行中跟随最新：视图钉在右缘
            _liveDevice.WaveformTick += OnLiveTick;
            _liveDevice.WaveformStopped += OnLiveStopped;
            _liveTimer = new System.Windows.Forms.Timer { Interval = 100 };
            _liveTimer.Tick += (_, _) => RefreshLiveFrame();
            _liveTimer.Start();
        }
        else
        {
            // 静态预览：默认从 t=0 看起，超一屏用拖拽条回看/前看
            _follow = false;
            SyncScrub(series);
        }
    }

    /// <summary>后台控制线程回调：只加锁存点，不碰 UI。</summary>
    private void OnLiveTick(object? sender, WaveformTickEventArgs e)
    {
        _liveRunning = true;
        lock (_liveLock)
        {
            if (e.TimeSec < _liveLastT) { _lt.Clear(); _lv.Clear(); _lp.Clear(); _follow = true; }   // 新一次启动，t 回退 → 清空并恢复跟随
            _lt.Add(e.TimeSec); _lv.Add(e.VelMmS); _lp.Add(e.PosMm);
            _liveLastT = e.TimeSec;
        }
    }

    /// <summary>UI 抽帧：把事件线程数据快照成新序列并重绘（10fps，数组最多几万点，开销可忽略）。</summary>
    private void RefreshLiveFrame()
    {
        WaveformSeries snap;
        lock (_liveLock)
        {
            if (_lt.Count == 0 || _lt.Count == _s.TimeSec.Length) return;
            snap = new WaveformSeries
            {
                TimeSec = _lt.ToArray(), VelMmS = _lv.ToArray(), PosMm = _lp.ToArray(),
                DtSec = _s.DtSec, Converged = true, ModeName = _s.ModeName, Limits = _s.Limits,
            };
        }
        _s = snap;
        if (Text.EndsWith("…")) Text = $"轨迹预览 - 实时（{snap.ModeName}）";
        SyncScrub(snap);
        Invalidate();
    }

    /// <summary>波形停止（后台线程）：解锁拖拽条供回看，视图保持钉在尾部不跳。</summary>
    private void OnLiveStopped(object? sender, WaveformStopReason reason)
    {
        if (IsDisposed || !IsHandleCreated) return;
        BeginInvoke(() => { _liveRunning = false; SyncScrub(_s); Invalidate(); });
    }

    /// <summary>同步拖拽条范围/值/使能：跟随中钉右缘；运行中锁定；停止且超过一屏才允许拖拽。</summary>
    private void SyncScrub(WaveformSeries s)
    {
        int wb = WindowBeats(s.DtSec, s.ModeName.Contains("PRTS"), s.TimeSec.Length);
        int maxVal = Math.Max(0, s.TimeSec.Length - 1 - wb);
        _syncScrub = true;
        if (_scrub.Maximum != maxVal) _scrub.Maximum = maxVal;
        if (_follow) _scrub.Value = maxVal;
        _scrub.Enabled = !_liveRunning && maxVal > 0;
        _syncScrub = false;
    }

    /// <summary>一屏拍数：非 PRTS 按固定 10s 折算（dt 换算拍数）；PRTS 固定 40 拍。</summary>
    private static int WindowBeats(double dtSec, bool isPrts, int n)
    {
        double dt = Math.Max(1e-6, dtSec);
        return isPrts ? PrtsWindowBeats : Math.Max(2, (int)Math.Round(WindowSec / dt));
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        var g = e.Graphics;
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        g.Clear(Color.White);

        // ---- 标题行 ----
        int nTicks = _s.TimeSec.Length;
        double tEnd = nTicks > 0 ? _s.TimeSec[^1] : 0;
        bool isPrts = _s.ModeName.Contains("PRTS");
        string head = isPrts
            ? $"{_s.ModeName}：速度 V(K)（上）与积分位置 p(K)（下）   每拍 dt={_s.DtSec * 1000:F0}ms   共 {nTicks} 拍（K=0~{nTicks - 1}）"
            : $"{_s.ModeName}：速度 v(t)（上）与积分位置 p(t)（下）   采样 dt={_s.DtSec * 1000:F0}ms   时长 {tEnd:F1}s";
        g.DrawString(head, Font, Brushes.Black, MarginL, 8);
        if (!_s.Converged)
            g.DrawString("[仿真达上限未收敛，序列被截断，图形仅供参考]", _smallFont, new SolidBrush(Color.Red), MarginL + 4, 28);
        if (_s.TimeSec.Length < 2)
        {
            string hint = _liveDevice is not null ? "已订阅波形拍事件，启动波形后自动绘制…" : "无数据：请检查参数（时长/循环数）";
            g.DrawString(hint, Font, Brushes.DarkGray, MarginL, MarginT + 40);
            return;
        }

        int availH = ClientSize.Height - MarginT - MarginB - PaneGap - ScrubH;
        int availW = ClientSize.Width - MarginL - MarginR;
        if (availW < 60 || availH < 60) return;
        var vRect = new Rectangle(MarginL, MarginT, availW, (int)(availH * 0.46));
        var pRect = new Rectangle(MarginL, MarginT + vRect.Height + PaneGap, availW, availH - vRect.Height);

        // ---- 值域 ----
        double vMax = 0; double pMin = double.MaxValue, pMax = double.MinValue;
        foreach (double v in _s.VelMmS) vMax = Math.Max(vMax, Math.Abs(v));
        foreach (double p in _s.PosMm) { pMin = Math.Min(pMin, p); pMax = Math.Max(pMax, p); }
        vMax = Math.Max(vMax * 1.15, 1e-6);
        // 位置纵轴以 0 为中心（对称上下界）
        double pAbsMax = Math.Max(Math.Abs(pMin), Math.Abs(pMax));
        double pad = Math.Max(pAbsMax * 0.12, 1e-6);
        pMin = -(pAbsMax + pad);
        pMax = pAbsMax + pad;

        // 软限位线：仅当与数据量级相当时才纳入值域显示，避免把曲线压扁
        bool showLimits = _s.Limits is not null && _s.Limits.PosMm > _s.Limits.NegMm;
        double limSpan = Math.Max(pMax - pMin, 1e-9);
        bool limInView = showLimits && _s.Limits!.PosMm <= pMax + limSpan && _s.Limits.NegMm >= pMin - limSpan;

        // ---- 固定比例尺视图窗口：一屏 10s（PRTS 40 拍）；跟随中钉最新尾部，结束后拖拽条定历史段 ----
        double dtSec = Math.Max(1e-6, _s.DtSec);
        int winBeats = WindowBeats(dtSec, isPrts, nTicks);
        double winSec = winBeats * dtSec;
        double tStart = _follow ? Math.Max(0, tEnd - winSec) : _scrub.Value * dtSec;
        double tStop = tStart + winSec;   // 窗口宽度恒定：数据不足一屏也不把比例尺撑满

        // 窗口外数据不绘：只取窗口内点（首尾各多带一拍保证连线连续，绘制再裁剪到绘图框）
        int i0 = 0; while (i0 < nTicks && _s.TimeSec[i0] < tStart) i0++;
        i0 = Math.Max(0, i0 - 1);
        int i1 = i0; while (i1 < nTicks && _s.TimeSec[i1] <= tStop) i1++;
        i1 = Math.Min(nTicks, i1 + 1);
        if (i1 - i0 < 2) i1 = Math.Min(nTicks, i0 + 2);
        int wLen = i1 - i0;
        var tW = new double[wLen]; var vW = new double[wLen]; var pW = new double[wLen];
        Array.Copy(_s.TimeSec, i0, tW, 0, wLen);
        Array.Copy(_s.VelMmS, i0, vW, 0, wLen);
        Array.Copy(_s.PosMm, i0, pW, 0, wLen);

        string winInfo = isPrts
            ? $"窗口 {(int)(tStart / dtSec)}~{(int)(tStop / dtSec)}拍 / 共{nTicks}拍"
            : $"窗口 {tStart:F1}~{tStop:F1}s / 共 {tEnd:F1}s";
        g.DrawString(winInfo, _smallFont, Brushes.Gray,
            new RectangleF(ClientSize.Width - 300, 10, 284, 16), new StringFormat { Alignment = StringAlignment.Far });

        float Xv(double t) => vRect.X + (float)((t - tStart) / winSec) * vRect.Width;
        float Yv(double v) => vRect.Y + (float)((vMax - v) / (2 * vMax)) * vRect.Height;
        float Yp(double p) => pRect.Bottom - (float)((p - pMin) / (pMax - pMin)) * pRect.Height;

        DrawGridAndAxes(g, vRect, -vMax, vMax, Yv, "F0");
        DrawGridAndAxes(g, pRect, pMin, pMax, Yp, "F1");
        DrawTimeTicks(g, vRect, pRect, tStart, tStop, isPrts, dtSec);
        
        // ---- 0 基准横轴（实线）：速度、位置两图都在值=0 处画一条醒目的实线作为分界 ----
        using (var zp = new Pen(Color.FromArgb(120, 120, 120), 1.3f))
        {
            float yv0 = Yv(0);
            g.DrawLine(zp, vRect.X, yv0, vRect.Right, yv0);
            float yp0 = Yp(0);
            g.DrawLine(zp, pRect.X, yp0, pRect.Right, yp0);
        }
        
        // ---- 速度/位置序列（裁剪到绘图框内：跨窗口边界的点不外溢到边距）----
        System.Drawing.Drawing2D.GraphicsState clipSt = g.Save();
        g.SetClip(Rectangle.Union(vRect, pRect));

        // ---- 速度阶梯线 ----
        var vPts = MapPoints(tW, vW, Xv, Yv);
        using (var pen = new Pen(VelColor, 2f)) g.DrawLines(pen, vPts);

        // ---- 位置填充（以 0 线为分界：正半轴向上填、负半轴向下填）+ 阶梯线 ----
        var pPts = MapPoints(tW, pW, Xv, Yp);
        float zeroY = Yp(Math.Clamp(0, pMin, pMax));
        using (var fb = new SolidBrush(PosFillColor))
        {
            // MapPoints 输出为阶梯点对 (x0,y),(x1,y)（同 y），逐段填充曲线到 0 线之间的矩形，
            // 保证正负各自以 0 为界分布（FillPolygon 对跨 0 自交多边形填充会错乱）。
            for (int i = 0; i + 1 < pPts.Length; i += 2)
            {
                float x0 = pPts[i].X, x1 = pPts[i + 1].X, y = pPts[i].Y;
                if (x1 <= x0) continue;
                float top = Math.Min(y, zeroY), h = Math.Abs(y - zeroY);
                g.FillRectangle(fb, x0, top, x1 - x0, h);
            }
        }
        using (var pen = new Pen(PosColor, 2f)) g.DrawLines(pen, pPts);

        g.Restore(clipSt);
        g.DrawString(isPrts ? "速度 V(K)" : "速度 v(t)", _smallFont, new SolidBrush(VelColor), MarginL - 58, vRect.Y - 16);
        g.DrawString(isPrts ? "位置 p(K)" : "位置 p(t)", _smallFont, new SolidBrush(PosColor), MarginL - 58, pRect.Y - 16);

        // ---- 软限位虚线 ----
        if (limInView)
        {
            using var lp = new Pen(LimitColor, 1f) { DashStyle = System.Drawing.Drawing2D.DashStyle.Dash };
            using var lb = new SolidBrush(LimitColor);
            foreach (double lim in new[] { _s.Limits!.PosMm, _s.Limits.NegMm })
            {
                if (lim < pMin || lim > pMax) continue;
                float y = Yp(lim);
                g.DrawLine(lp, pRect.X, y, pRect.Right, y);
                g.DrawString($"软限位 {lim:F1}", _smallFont, lb, pRect.Right - 88, y - 15);
            }
        }

        // ---- 图例 ----
        DrawLegend(g, vRect.Right - 150, MarginT - 20, isPrts);
    }

    // ==================== 绘图辅助 ====================

    /// <summary>抽稀到约 TargetPoints 点：每桶保 min/max（按时间序），台阶沿不丢失。</summary>
    private static List<(double T, double V)> Decimate(double[] t, double[] v, int target)
    {
        int n = t.Length;
        var output = new List<(double, double)>(Math.Min(n, target) + 2);
        if (n <= target)
        {
            for (int i = 0; i < n; i++) output.Add((t[i], v[i]));
            return output;
        }
        int buckets = Math.Max(2, target / 2);
        int size = (n + buckets - 1) / buckets;
        for (int b = 0; b < buckets; b++)
        {
            int s = b * size, e = Math.Min(n, s + size);
            if (s >= e) break;
            int iMin = s, iMax = s;
            for (int i = s; i < e; i++)
            {
                if (v[i] < v[iMin]) iMin = i;
                if (v[i] > v[iMax]) iMax = i;
            }
            if (iMin == iMax) output.Add((t[iMin], v[iMin]));
            else if (iMin < iMax) { output.Add((t[iMin], v[iMin])); output.Add((t[iMax], v[iMax])); }
            else { output.Add((t[iMax], v[iMax])); output.Add((t[iMin], v[iMin])); }
        }
        return output;
    }

    /// <summary>把每拍采样点展开成阶梯图（steps-post）：一拍内水平保持到下一拍起点，拍间形成垂直跳变，
    /// 忠实反映 CSV 速度模式“逐拍恒定速度”的物理本质（直上直下），而非把跳变摊成斜线。</summary>
    private static PointF[] MapPoints(double[] t, double[] v, Func<double, float> mx, Func<double, float> my)
    {
        var dec = Decimate(t, v, TargetPoints);
        int n = dec.Count;
        var pts = new List<PointF>(n * 2);
        for (int i = 0; i < n; i++)
        {
            float x0 = mx(dec[i].T);
            float y = my(dec[i].V);
            pts.Add(new PointF(x0, y));
            // 水平保持到下一拍的起点；下一拍从同一 x 起画自身高度 → 两点同 x 不同 y，DrawLines 连成垂直跳变
            float x1 = i + 1 < n ? mx(dec[i + 1].T) : x0;
            pts.Add(new PointF(x1, y));
        }
        return pts.ToArray();
    }

    private void DrawGridAndAxes(Graphics g, Rectangle r, double vmin, double vmax, Func<double, float> mapY, string valFmt)
    {
        using var frame = new Pen(Color.FromArgb(150, 150, 150));
        using var grid = new Pen(GridColor);
        using var txt = new SolidBrush(Color.Gray);
        var right = new StringFormat { Alignment = StringAlignment.Far, LineAlignment = StringAlignment.Center };
        // 按量程自适应小数位：避免小数值（如 V 设得过小时位置仅 0.0x mm）被定宽格式舍入成一排 0。
        string fmt = AutoFmt(vmax - vmin, valFmt);
        g.DrawRectangle(frame, r.X, r.Y, r.Width, r.Height);
        for (int i = 0; i <= 4; i++)
        {
            double val = vmin + (vmax - vmin) * i / 4;
            float y = mapY(val);
            if (i != 0 && i != 4) g.DrawLine(grid, r.X, y, r.Right, y);
            g.DrawString(val.ToString(fmt, Inv), _smallFont, txt, r.X - 6, y, right);
        }
    }
    
    /// <summary>根据轴量程选合适的小数位：量程越小位数越多，保底不显示成全 0；大回退到传入的默认格式。</summary>
    private static string AutoFmt(double span, string fallback)
    {
        double a = Math.Abs(span);
        if (a >= 100) return "F0";
        if (a >= 10) return "F1";
        if (a >= 1) return "F2";
        if (a >= 0.1) return "F3";
        if (a >= 0.001) return "F4";
        return a < 1e-9 ? fallback : "G4";   // 量程≈0（数据全零）时退回默认，避免无意义长串
    }

    /// <summary>视图窗口内按“整齐”的时间/拍数步长画竖直网格与标签；PRTS 轴按整数拍标注。</summary>
    private void DrawTimeTicks(Graphics g, Rectangle vRect, Rectangle pRect, double tStart, double tStop, bool isPrts, double dtSec)
    {
        using var grid = new Pen(GridColor);
        using var txt = new SolidBrush(Color.Gray);
        var center = new StringFormat { Alignment = StringAlignment.Center };
        double span = tStop - tStart;
        if (span <= 1e-9) return;
        float Xt(double t) => vRect.X + (float)((t - tStart) / span) * vRect.Width;

        double step;
        if (isPrts)
            step = Math.Max(1, Math.Ceiling(span / dtSec / 5)) * dtSec;   // 整数拍步长
        else
        {
            double raw = span / 5;
            step = raw switch
            {
                <= 0.2 => 0.2, <= 0.5 => 0.5, <= 1 => 1, <= 2 => 2, <= 5 => 5,
                <= 10 => 10, <= 15 => 15, <= 30 => 30, <= 60 => 60, _ => 120,
            };
        }
        for (double tv = Math.Ceiling(tStart / step) * step; tv <= tStop + 1e-9; tv += step)
        {
            float x = Xt(tv);
            g.DrawLine(grid, x, vRect.Y, x, vRect.Bottom);
            g.DrawLine(grid, x, pRect.Y, x, pRect.Bottom);
            string label = isPrts ? Math.Round(tv / dtSec).ToString("F0", Inv) : tv.ToString("F1", Inv);
            g.DrawString(label, _smallFont, txt, x, pRect.Bottom + 4, center);
        }
    }

    private void DrawLegend(Graphics g, int x, int y, bool isPrts)
    {
        using var vp = new Pen(VelColor, 3f);
        using var pp = new Pen(PosColor, 3f);
        using var txt = new SolidBrush(Color.DimGray);
        g.DrawLine(vp, x, y + 6, x + 22, y + 6);
        g.DrawString(isPrts ? "V(K)" : "v(t)", _smallFont, txt, x + 26, y - 2);
        g.DrawLine(pp, x + 82, y + 6, x + 104, y + 6);
        g.DrawString(isPrts ? "p(K)" : "p(t)", _smallFont, txt, x + 108, y - 2);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _liveTimer?.Stop();
            _liveTimer?.Dispose();
            if (_liveDevice is not null)
            {
                _liveDevice.WaveformTick -= OnLiveTick;
                _liveDevice.WaveformStopped -= OnLiveStopped;
            }
            _smallFont.Dispose();
        }
        base.Dispose(disposing);
    }
}
