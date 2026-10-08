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
/// 轨迹采样器：与 WaveformValidator.Simulate 相同的积分回路（同一起点、同一 dt、
/// 同一终止条件），区别是把每一拍的 v 与积分位置 p 记录下来用于绘图。
/// </summary>
public static class WaveformSampler
{
    private const double MaxSimSeconds = 600.0;
    private const int MaxTicks = 120_000;

    public static WaveformSeries Capture(IWaveformGenerator g, double dtSec, MotionLimits? lim)
    {
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec));

        g.Reset();
        var ts = new List<double>();
        var vs = new List<double>();
        var ps = new List<double>();
        double t = 0, pos = 0, prevV = 0;
        bool ended = false;
        int count = 0;
        double total = g.TotalDurationSeconds;
        
        while (count < MaxTicks && t <= MaxSimSeconds)
        {
            double v = g.NextVelocity(t, pos);
            // 与 WaveformValidator.Simulate 保持一致：连续型用梯形法（首拍不产生位移），
            // 阶梯型用左端点 pos += v*dt（逐拍恒速的物理模型，得到干净的整数台阶并回零）。
            if (g.ContinuousVelocity)
            {
                if (count > 0)
                    pos += (prevV + v) / 2 * dtSec;
            }
            else
            {
                pos += v * dtSec;
            }
        
            ts.Add(t);
            vs.Add(v);
            ps.Add(pos);
        
            count++;
            prevV = v;
            t += dtSec;

            bool byDuration = !double.IsPositiveInfinity(total) && t >= total;
            if (g.IsFinished(t) || byDuration) { ended = true; break; }
        }

        return new WaveformSeries
        {
            TimeSec = ts.ToArray(),
            VelMmS = vs.ToArray(),
            PosMm = ps.ToArray(),
            DtSec = dtSec,
            Converged = ended,
            ModeName = g.ModeName,
            Limits = lim,
        };
    }
}
