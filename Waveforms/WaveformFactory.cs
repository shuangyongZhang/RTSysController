using System.Globalization;
using System.Text;
using MotorControlApp.Configuration;

namespace MotorControlApp.Waveforms;

/// <summary>运动波形模式。</summary>
public enum WaveformMode
{
    Sine = 0,
    MultiSine = 1,
    Square = 2,
    Pulse = 3,
    Prts = 4,
}

/// <summary>工厂产出的运行时包：生成器 + 限位 + 控制周期 + 逐周期位置校正参数（仅连续波形生效）。</summary>
public sealed record WaveformRuntime(IWaveformGenerator Generator, MotionLimits Limits, double DtSec,
    bool PosCorrEnabled = false, double PosCorrKpPerS = 0.2, double PosCorrMaxMmS = 10);

/// <summary>
/// 从 WaveformConfig 构造对应的波形生成器与运行限制。集中在此，UI/设备层只管调用。
/// </summary>
public static class WaveformFactory
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static WaveformRuntime Build(AppConfig cfg)
    {
        ArgumentNullException.ThrowIfNull(cfg);
        WaveformConfig w = cfg.Waveform;
        double dt = w.GetDtSec();
        MotionLimits lim = BuildLimits(cfg, w.GetAccelLimit());
        var mode = (WaveformMode)w.GetMode();

        IWaveformGenerator gen = mode switch
        {
            WaveformMode.Sine => new SineGenerator(
                w.SineFreqHz.Value, w.SineStrokeMm.Value, w.SineBiasMmS.Value,
                DegToRad(w.SinePhaseDeg.Value), w.SineDurationS.Value, w.SineDwellS.Value),

            WaveformMode.MultiSine => BuildMultiSine(w),

            WaveformMode.Square => new SquareGenerator(
                ParseWaypoints(w.SqWaypoints.Value), Math.Max(1, w.SqCycles.Value)),

            WaveformMode.Pulse => new PulseGenerator(
                w.PuSpeedMmS.Value, ParseDoubles(w.PuDurations.Value), Math.Max(1, w.PuCycles.Value), w.PuGapS.Value),

            WaveformMode.Prts => BuildPrts(w, dt),

            _ => throw new InvalidOperationException($"未知波形模式 {mode}"),
        };

        return new WaveformRuntime(gen, lim, dt, w.GetPosCorrEnabled(), w.GetPosCorrKp(), w.GetPosCorrMaxMmS());
    }

    private static IWaveformGenerator BuildMultiSine(WaveformConfig w)
    {
        // 显式分量表优先
        if (!string.IsNullOrWhiteSpace(w.MsTable.Value))
        {
            var comps = ParseComponents(w.MsTable.Value);
            return new MultiSineGenerator(comps, w.MsDurationS.Value);
        }

        var phase = (MsPhaseMode)Math.Clamp(w.MsPhaseMode.Value, 0, 2);
        return MultiSineGenerator.CreateFromRule(
            w.MsBaseFreqHz.Value, Math.Clamp(w.MsCount.Value, 1, 64), w.MsFundAmpMmS.Value,
            w.MsDecayP.Value, phase, w.MsSeed.Value, w.MsDurationS.Value);
    }
    
    /// <summary>PRTS 构造：显式速度序列优先（逐拍原样下发），为空则回退规则法随机生成。</summary>
    private static IWaveformGenerator BuildPrts(WaveformConfig w, double dt)
    {
        if (!string.IsNullOrWhiteSpace(w.PrtsTable.Value))
        {
            var seq = ParsePrtsSequence(w.PrtsTable.Value, w.PrtsV.Value);
            return PrtsGenerator.CreateFromSequence(seq, dt);
        }
        return new PrtsGenerator(
            w.PrtsV.Value, Math.Max(1, w.PrtsK.Value), w.PrtsS.Value,
            w.PrtsDurationS.Value, w.PrtsSeed.Value, dt);
    }

    /// <summary>由软限位（units）换算成 mm 域的运行限制，并携带软限位保护开关状态。
    /// 阶梯型模式（方波/脉冲/PRTS）的减速度从各自页签读取；其余模式减速度沿用加速度上限。</summary>
    public static MotionLimits BuildLimits(AppConfig cfg, double accelLimitMmS2)
    {
        var mu = new MotionUnits(cfg.ZMotion);
        double posMm = mu.UnitsToMm(cfg.ZMotion.GetSoftLimitPos());
        double negMm = mu.UnitsToMm(cfg.ZMotion.GetSoftLimitNeg());
        WaveformConfig w = cfg.Waveform;
        double decel = (WaveformMode)w.GetMode() switch
        {
            WaveformMode.Square => w.SqDecelMmS2.Value,
            WaveformMode.Pulse => w.PuDecelMmS2.Value,
            WaveformMode.Prts => w.PrtsDecelMmS2.Value,
            _ => accelLimitMmS2,
        };
        return new MotionLimits(negMm, posMm, accelLimitMmS2)
        {
            EnforceSoftLimit = w.GetEnforceSoftLimit(),
            DecelLimitMmS2 = decel,
            DynAmpK = w.GetStrokeAmpK(),
        };
    }

    public static double DegToRad(double deg) => deg * Math.PI / 180.0;

    // ---- 字符串解析 ----

    /// <summary>"P,v,t;P,v,t;..."（mm, mm/s, s）。</summary>
    public static IReadOnlyList<Waypoint> ParseWaypoints(string s)
    {
        var list = new List<Waypoint>();
        foreach (string row in Split(s, ';'))
        {
            string[] f = row.Split(',');
            if (f.Length < 3) continue;
            if (TryD(f[0], out double p) && TryD(f[1], out double v) && TryD(f[2], out double t))
                list.Add(new Waypoint(p, v, t));
        }
        if (list.Count < 2) throw new FormatException("方波途经点格式应为 \"P,v,t;P,v,t;...\"，且至少 2 个点");
        return list;
    }

    /// <summary>"t1,t2,t3,t4"（s）。</summary>
    public static IReadOnlyList<double> ParseDoubles(string s)
    {
        var list = new List<double>();
        foreach (string tok in Split(s, ','))
            if (TryD(tok, out double d)) list.Add(d);
        if (list.Count == 0) throw new FormatException("脉冲宽度序列格式应为 \"t1,t2,...\"");
        return list;
    }

    /// <summary>"f,A,φ;f,A,φ;..."（Hz, mm/s, 度）。</summary>
    public static IReadOnlyList<SineComponent> ParseComponents(string s)
    {
        var list = new List<SineComponent>();
        foreach (string row in Split(s, ';'))
        {
            string[] f = row.Split(',');
            if (f.Length < 2) continue;
            if (TryD(f[0], out double fk) && TryD(f[1], out double ak))
            {
                double phi = f.Length >= 3 && TryD(f[2], out double pd) ? DegToRad(pd) : 0;
                list.Add(new SineComponent(fk, ak, phi));
            }
        }
        if (list.Count == 0) throw new FormatException("多正弦分量表格式应为 \"f,A,φ;...\"");
        return list;
    }
    
    /// <summary>
    /// 解析 PRTS 显式逐拍速度序列：每个 token 一拍，以逗号/空格/分号/制表符/换行分隔。
    /// token 可为符号档位“+”/“-”/“0”（按速度幅值 V 取 ±V/0；也支持 p/m/P/M），或直接写 mm/s 数值（如 +100/-100/0）。
    /// </summary>
    public static IReadOnlyList<double> ParsePrtsSequence(string s, double V)
    {
        var list = new List<double>();
        foreach (string tok in (s ?? string.Empty).Split(
            new[] { ',', ' ', ';', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            switch (tok)
            {
                case "+": case "p": case "P": list.Add(V); continue;
                case "-": case "m": case "M": list.Add(-V); continue;
                case "0": list.Add(0); continue;
            }
            if (TryD(tok, out double v)) list.Add(v);   // 数值 mm/s（允许带符号，如 +100）
        }
        if (list.Count == 0) throw new FormatException("PRTS 显式速度序列为空或格式错误（用 +/0/- 或每拍 mm/s 数值，逗号/空格分隔）");
        return list;
    }

    /// <summary>把显式分量表中各分量幅值 A 按 s 缩放（f/φ 不变），供“一键缩放至安全”使用。</summary>
    public static string ScaleComponentTable(string table, double s)
    {
        var comps = ParseComponents(table);
        var sb = new StringBuilder();
        foreach (var c in comps)
        {
            if (sb.Length > 0) sb.Append(';');
            sb.Append(c.FreqHz.ToString("0.####", Inv)).Append(',')
              .Append((c.AmpMmS * s).ToString("0.####", Inv)).Append(',')
              .Append((c.PhaseRad * 180.0 / Math.PI).ToString("0.####", Inv));
        }
        return sb.ToString();
    }

    /// <summary>把方波途经点表 "P,v,t;..." 中的位置 P 与速度 v 按 s 等比缩放（停顿 t 不变），供“一键缩放至安全”使用。</summary>
    public static string ScaleWaypointTable(string table, double s)
    {
        var wps = ParseWaypoints(table);
        var sb = new StringBuilder();
        foreach (var w in wps)
        {
            if (sb.Length > 0) sb.Append(';');
            sb.Append((w.PosMm * s).ToString("0.####", Inv)).Append(',')
              .Append((w.SpeedMmS * s).ToString("0.####", Inv)).Append(',')
              .Append(w.DwellSec.ToString("0.####", Inv));
        }
        return sb.ToString();
    }

    /// <summary>把 PRTS 显式逐拍表中的数值速度按 s 缩放；+/0/- 符号拍随速度幅值走，原样保留。</summary>
    public static string ScalePrtsTable(string table, double s)
    {
        var sb = new StringBuilder();
        foreach (string tok in (table ?? string.Empty).Split(
            new[] { ',', ' ', ';', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            if (sb.Length > 0) sb.Append(',');
            if ((tok is "+" or "p" or "P" or "-" or "m" or "M" or "0") || !TryD(tok, out double v)) sb.Append(tok);
            else sb.Append((v * s).ToString("0.####", Inv));
        }
        return sb.ToString();
    }

    private static IEnumerable<string> Split(string s, char sep) =>
        (s ?? string.Empty).Split(sep, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

    private static bool TryD(string t, out double v) =>
        double.TryParse(t.Trim(), NumberStyles.Float, Inv, out v);
}
