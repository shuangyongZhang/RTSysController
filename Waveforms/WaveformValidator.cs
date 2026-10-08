namespace MotorControlApp.Waveforms;

/// <summary>时域仿真/校验结果。</summary>
public sealed class WaveformSimResult
{
    public double PeakVelocityMmS { get; init; }
    public double MinPosMm { get; init; }
    public double MaxPosMm { get; init; }
    public double PeakAccelMmS2 { get; init; }
    public double MeanVelocityMmS { get; init; }
    public double SimulatedSeconds { get; init; }

    /// <summary>违规原因列表，空表示安全。</summary>
    public IReadOnlyList<string> Violations { get; init; } = Array.Empty<string>();

    /// <summary>建议整体缩放系数（&lt;=1）；把幅值乘它可拉回限速/限行程内。1 表示无需缩放。</summary>
    public double SuggestedScale { get; init; } = 1.0;

    public bool Ok => Violations.Count == 0;
}

/// <summary>
/// 波形时域校验器：按控制周期 dt 全时域仿真整段速度序列并积分位置，
/// 对"叠加后的瞬时峰值"（含相长干涉）而非单分量做限速/限行程/限加速度检查。
/// </summary>
public static class WaveformValidator
{
    /// <summary>仿真最长不超过该秒数（防止方波/循环数设错时无限仿真）。</summary>
    private const double MaxSimSeconds = 600.0;

    public static WaveformSimResult Validate(IWaveformGenerator g, MotionLimits lim, double dtSec)
    {
        return Simulate(g, dtSec, lim);
    }

    /// <summary>核心仿真：积分速度→位置，统计峰值并判限。</summary>
    public static WaveformSimResult Simulate(IWaveformGenerator g, double dtSec, MotionLimits? lim)
    {
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec));

        g.Reset();
        double t = 0, pos = 0, prevV = 0;
        double maxAbsV = 0, minX = 0, maxX = 0, maxA = 0, sumV = 0;
        int count = 0;
        double total = g.TotalDurationSeconds;
        bool ended = false;

        while (count < 5_000_000 && t <= MaxSimSeconds)
        {
            double v = g.NextVelocity(t, pos);
            double a = count == 0 ? 0 : Math.Abs(v - prevV) / dtSec;
            
            // 位置积分按波形类型区分：
            //  连续型（单/多正弦）→ 梯形法且首拍不产生位移，消除左端点求和的系统性直流偏移；
            //  阶梯型（方波/脉冲/PRTS）→ 本拍速度在本拍全程作用，用左端点 pos += v*dt 才符合
            //    “逐拍恒速”物理模型（梯形法会把台阶拐角平均掉、且使阶梯波位置不再回到 0）。
            if (g.ContinuousVelocity)
            {
                if (count > 0)
                    pos += (prevV + v) / 2 * dtSec;
            }
            else
            {
                pos += v * dtSec;
            }

            maxAbsV = Math.Max(maxAbsV, Math.Abs(v));
            maxA = Math.Max(maxA, a);
            if (pos < minX) minX = pos;
            if (pos > maxX) maxX = pos;
            sumV += v;
            count++;
            prevV = v;
            t += dtSec;

            bool byDuration = !double.IsPositiveInfinity(total) && t >= total;
            if (g.IsFinished(t) || byDuration) { ended = true; break; }
        }

        var violations = new List<string>();

        // 仿真撞上限仍未结束 = 波形不收敛（如方波到位判定失效导致极限环）或时长/循环数过大，
        // 此时统计量无效，必须禁止启动而不是误报“校验通过”。
        if (!ended)
            violations.Add($"仿真达上限（{MaxSimSeconds:F0}s）波形仍未结束：不收敛或循环数过大，无法预估，请检查参数");
        double meanV = count > 0 ? sumV / count : 0;

        if (maxAbsV > MotionUnits.VmaxMmS + 1e-6)
            violations.Add($"峰值速度 {maxAbsV:F0} mm/s 超过 vmax {MotionUnits.VmaxMmS:F0} mm/s");

        if (lim is not null)
        {
            // 软限位保护关闭时不判超行程（坚持把运动做完），仅提醒展示用
            if (lim.EnforceSoftLimit && (maxX > lim.PosMm + 1e-6 || minX < lim.NegMm - 1e-6))
                violations.Add($"位置范围 [{minX:F1}, {maxX:F1}] mm 超出软限位 [{lim.NegMm:F1}, {lim.PosMm:F1}] mm（超行程）");

            // 加速度判限仅对连续速度波形有意义；阶梯型（方波/脉冲/PRTS）的 Δv/dt 是指令阶跃假象，
            // 真机由控制器 Accel/Decel 钳制，故不据此报超限。
            if (g.ContinuousVelocity && lim.AccelLimitMmS2 > 0 && maxA > lim.AccelLimitMmS2 + 1e-6)
                violations.Add($"峰值加速度 {maxA:F0} mm/s² 超过上限 {lim.AccelLimitMmS2:F0} mm/s²");
        }

        // 缩放建议：把超限项按比例压回限制内（速度/行程/加速度三项都要求，缺一则对应场景下按钮失效）
        double scale = 1.0;
        if (maxAbsV > 0) scale = Math.Min(scale, MotionUnits.VmaxMmS / maxAbsV);
        if (lim is not null && lim.EnforceSoftLimit)
        {
            // 行程按两侧分别算，避免非对称软限位下用较大侧缩放后仍超较小侧
            if (maxX > lim.PosMm && maxX > 0) scale = Math.Min(scale, lim.PosMm / maxX);
            if (minX < lim.NegMm && minX < 0) scale = Math.Min(scale, lim.NegMm / minX);
        }
        if (lim is not null)
        {
            // 加速度：仅连续波形由波形本身决定（阶梯型由控制器钳制），不缩则“只剩加速度超限”时缩放无效
            if (g.ContinuousVelocity && lim.AccelLimitMmS2 > 0 && maxA > lim.AccelLimitMmS2)
                scale = Math.Min(scale, lim.AccelLimitMmS2 / maxA);
        }
        if (scale < 1.0) scale *= 0.98;   // 留 2% 余量，避免正好卡在判限边界上反复报超限
        scale = Math.Clamp(scale, 0.0, 1.0);

        return new WaveformSimResult
        {
            PeakVelocityMmS = maxAbsV,
            MinPosMm = minX,
            MaxPosMm = maxX,
            PeakAccelMmS2 = maxA,
            MeanVelocityMmS = meanV,
            SimulatedSeconds = t,
            Violations = violations,
            SuggestedScale = scale,
        };
    }
}
