namespace MotorControlApp.Waveforms;

/// <summary>离线时域仿真的逐拍采样序列，供轨迹预览绘图。</summary>
public sealed class WaveformSeries
{
    public required double[] TimeSec { get; init; }
    public required double[] VelMmS { get; init; }
    public required double[] PosMm { get; init; }
    public required double DtSec { get; init; }
    /// <summary>false = 仿真撞上限被截断（波形不收敛），图形仅供参考。</summary>
    public required bool Converged { get; init; }
    public required string ModeName { get; init; }
    public MotionLimits? Limits { get; init; }
}

/// <summary>
/// 轨迹采样器：直接复用 WaveformValidator.SimulateCore（同一起点、同一 dt、同一终止条件、
/// 同一含加减速跟随模型的积分回路），区别只是把每一拍的 v 与预测实际位置记录下来用于绘图。
/// 位置反馈波形（方波/脉冲）按校验器给出的保守实际周期采样；校验标签的"行程"还会
/// 额外加上一拍反馈滞后余量，故该处可能比本曲线略宽，属于最坏相位保护。
/// 因此预览图的位置曲线与校验标签的"行程"、真机实测三者同源一致。
/// </summary>
public static class WaveformSampler
{
    public static WaveformSeries Capture(IWaveformGenerator g, double dtSec, MotionLimits? lim)
    {
        var ts = new List<double>();
        var vs = new List<double>();
        var ps = new List<double>();
        double simDt = WaveformValidator.SimDtFor(g, dtSec);
        var core = WaveformValidator.SimulateCore(g, simDt,
            WaveformValidator.AccelFor(lim), WaveformValidator.DecelFor(lim),
            (t, v, pos) => { ts.Add(t); vs.Add(v); ps.Add(pos); });

        return new WaveformSeries
        {
            TimeSec = ts.ToArray(),
            VelMmS = vs.ToArray(),
            PosMm = ps.ToArray(),
            DtSec = simDt,
            Converged = core.Ended,
            ModeName = g.ModeName,
            Limits = lim,
        };
    }
}
