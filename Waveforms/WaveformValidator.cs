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

    /// <summary>建议整体缩放系数（&lt;=1）；把幅值乘它可拉回限速/限行程内。1 表示无需缩放，0 表示缩放无法解决。</summary>
    public double SuggestedScale { get; init; } = 1.0;

    public bool Ok => Violations.Count == 0;
}

/// <summary>
/// 波形时域校验器：按控制周期 dt 全时域仿真整段速度序列，并内嵌一阶"轴跟随模型"——
/// 实际速度每拍最多向指令逼近 accelLimit×dt（与真机 Vmove+Speed 按 ACC 斜坡逼近的行为一致），
/// 行程由实际速度积分。因此启动瞬态欠账（如 ψ=±90° 首拍速度阶跃）与端点换向超程
/// 都被计入预测行程，评估结果与真机实测同量级，而不是理想跟随的下界。
/// 对"叠加后的瞬时峰值"（含相长干涉）做限速/限行程/限加速度检查。
/// </summary>
public static class WaveformValidator
{
    /// <summary>仿真最长不超过该秒数（防止方波/循环数设错时无限仿真）。</summary>
    private const double MaxSimSeconds = 600.0;

    /// <summary>加速度上限未设（<=0）时的跟随模型兜底值，与 ZMotionDeviceController.StartWaveform 的回退一致。</summary>
    internal const double DefaultAccelMmS2 = 5000.0;

    /// <summary>跟随模型使用的加速度限：优先取界面"加速度上限"，未设时兜底 5000（真机同样按此下发）。</summary>
    internal static double AccelFor(MotionLimits? lim) =>
        lim is { AccelLimitMmS2: > 0 } ? lim.AccelLimitMmS2 : DefaultAccelMmS2;

    public static WaveformSimResult Validate(IWaveformGenerator g, MotionLimits lim, double dtSec)
    {
        return Simulate(g, dtSec, lim);
    }

    /// <summary>仿真核心统计量（不含判限）。</summary>
    internal readonly struct SimCoreResult
    {
        public double PeakV { get; init; }
        public double MinPos { get; init; }
        public double MaxPos { get; init; }
        public double PeakA { get; init; }
        public double MeanV { get; init; }
        public double EndT { get; init; }
        /// <summary>false = 撞仿真上限仍未结束（不收敛）。</summary>
        public bool Ended { get; init; }
    }

    /// <summary>
    /// 核心仿真（校验与轨迹预览共用）：指令速度 → 一阶加速度跟随 → 梯形积分实际位置。
    /// onTick(t, vCmd, posMm) 逐拍回调（预览采样用），null 为纯统计。
    /// </summary>
    internal static SimCoreResult SimulateCore(IWaveformGenerator g, double dtSec, double accelMmS2,
        Action<double, double, double>? onTick)
    {
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec));

        g.Reset();
        double t = 0, pos = 0, prevV = 0, vAct = 0;
        double maxAbsV = 0, minX = 0, maxX = 0, maxA = 0, sumV = 0;
        int count = 0;
        double total = g.TotalDurationSeconds;
        bool ended = false;
        double maxDv = accelMmS2 * dtSec;   // 一拍允许的最大速度变化（加速度钳制）

        while (count < 5_000_000 && t <= MaxSimSeconds)
        {
            double v = g.NextVelocity(t, pos);
            double a = count == 0 ? 0 : Math.Abs(v - prevV) / dtSec;

            // 一阶跟随模型：真机控制器以 ACC/DEC 斜坡逼近指令速度，实际速度每拍变化 ≤ maxDv。
            // 另加一拍指令传输延迟（本拍指令经网络+插补+驱动响应约下一拍才生效，保守取 prevV，
            // 首拍取 0）——启动速度阶跃的欠账因此更接近真机实测的永久性中心偏移。
            // 位置由实际速度梯形积分——启动瞬态欠账、端点换向超程都会如实进入预测行程。
            double vTarget = count == 0 ? 0 : prevV;
            double dv = vTarget - vAct;
            if (dv > maxDv) dv = maxDv;
            else if (dv < -maxDv) dv = -maxDv;
            double vActNew = vAct + dv;
            pos += (vAct + vActNew) / 2 * dtSec;
            vAct = vActNew;

            maxAbsV = Math.Max(maxAbsV, Math.Abs(v));
            maxA = Math.Max(maxA, a);
            if (pos < minX) minX = pos;
            if (pos > maxX) maxX = pos;
            sumV += v;
            count++;
            prevV = v;
            onTick?.Invoke(t, v, pos);
            t += dtSec;

            bool byDuration = !double.IsPositiveInfinity(total) && t >= total;
            if (g.IsFinished(t) || byDuration) { ended = true; break; }
        }

        return new SimCoreResult
        {
            PeakV = maxAbsV,
            MinPos = minX,
            MaxPos = maxX,
            PeakA = maxA,
            MeanV = count > 0 ? sumV / count : 0,
            EndT = t,
            Ended = ended,
        };
    }

    /// <summary>
    /// 动态放大展开：预测包络中心不变、半幅×K。快速换向（如 2Hz 正弦）下真机存在机械/伺服
    /// 动态超调，实测振幅会大于指令振幅——速率受限的跟随模型原理上只能缩小振幅、无法预测放大，
    /// 故用自校准得到的系数 K 把包络撑开罩住实测。K≈1 时原样返回。
    /// </summary>
    private static SimCoreResult Amplify(SimCoreResult c, MotionLimits? lim)
    {
        double k = lim?.DynAmpK ?? 1.0;
        if (k <= 1.0 + 1e-9) return c;
        double ctr = (c.MinPos + c.MaxPos) / 2, half = (c.MaxPos - c.MinPos) / 2 * k;
        return new SimCoreResult
        {
            PeakV = c.PeakV, PeakA = c.PeakA, MeanV = c.MeanV, EndT = c.EndT, Ended = c.Ended,
            MinPos = ctr - half, MaxPos = ctr + half,
        };
    }

    /// <summary>按统计量判限（Simulate 与缩放搜索共用同一套标准，保证"缩放后必过校验"）。</summary>
    private static List<string> CheckViolations(SimCoreResult c, IWaveformGenerator g, MotionLimits? lim)
    {
        var violations = new List<string>();

        // 仿真撞上限仍未结束 = 波形不收敛（如方波到位判定失效导致极限环）或时长/循环数过大，
        // 此时统计量无效，必须禁止启动而不是误报"校验通过"。
        if (!c.Ended)
            violations.Add($"仿真达上限（{MaxSimSeconds:F0}s）波形仍未结束：不收敛或循环数过大，无法预估，请检查参数");

        if (c.PeakV > MotionUnits.VmaxMmS + 1e-6)
            violations.Add($"峰值速度 {c.PeakV:F0} mm/s 超过 vmax {MotionUnits.VmaxMmS:F0} mm/s");

        if (lim is not null)
        {
            // 软限位保护关闭时不判超行程（坚持把运动做完），仅提醒展示用。
            // 判限时预留 5% 安全余量：跟随模型与真机（驱动带宽/滤波/总线延迟）必有残差，
            // 不把残差压到限位边上赌，宁可一键缩放多压一点。
            if (lim.EnforceSoftLimit)
            {
                const double MarginFrac = 0.05;
                double posAllow = lim.PosMm > 0 ? lim.PosMm * (1 - MarginFrac) : lim.PosMm;
                double negAllow = lim.NegMm < 0 ? lim.NegMm * (1 - MarginFrac) : lim.NegMm;
                if (c.MaxPos > posAllow + 1e-6 || c.MinPos < negAllow - 1e-6)
                    violations.Add($"预测行程 [{c.MinPos:F1}, {c.MaxPos:F1}] mm 超出安全行程 [{negAllow:F1}, {posAllow:F1}] mm（限位含 5% 余量，已计入加减速跟随误差与动态放大系数）");
            }

            // 指令加速度判限仅对连续速度波形有意义；阶梯型（方波/脉冲/PRTS）的 Δv/dt 是指令阶跃假象，
            // 真机由控制器 Accel/Decel 钳制（跟随模型已把该钳制计入行程预测），故不据此报超限。
            if (g.ContinuousVelocity && lim.AccelLimitMmS2 > 0 && c.PeakA > lim.AccelLimitMmS2 + 1e-6)
                violations.Add($"峰值加速度 {c.PeakA:F0} mm/s² 超过上限 {lim.AccelLimitMmS2:F0} mm/s²");
        }
        return violations;
    }

    public static WaveformSimResult Simulate(IWaveformGenerator g, double dtSec, MotionLimits? lim)
    {
        var core = Amplify(SimulateCore(g, dtSec, AccelFor(lim), null), lim);
        var violations = CheckViolations(core, g, lim);
        double scale = SuggestScale(g, dtSec, lim, core, violations);

        return new WaveformSimResult
        {
            PeakVelocityMmS = core.PeakV,
            MinPosMm = core.MinPos,
            MaxPosMm = core.MaxPos,
            PeakAccelMmS2 = core.PeakA,
            MeanVelocityMmS = core.MeanV,
            SimulatedSeconds = core.EndT,
            Violations = violations,
            SuggestedScale = scale,
        };
    }

    /// <summary>
    /// 缩放建议：不再用线性比例估算（跟随模型下超程随幅值非线性变化，线性缩了仍可能超），
    /// 而是对"指令速度整体乘 s"做二分搜索，找满足全部判限的最大 s（跟随模型内嵌于每次仿真）。
    /// 返回 0 表示连极小幅值都无法满足（如方波途经点本身在限位外），缩放解决不了。
    /// </summary>
    private static double SuggestScale(IWaveformGenerator g, double dtSec, MotionLimits? lim,
        SimCoreResult core, IReadOnlyList<string> violations)
    {
        if (violations.Count == 0) return 1.0;
        if (!core.Ended) return 0.0;   // 不收敛/超长，缩放无意义

        double accel = AccelFor(lim);
        var scaled = new ScaledGenerator(g);

        // 下限探测：极小幅值仍超限 → 缩放救不了（典型：方波途经点位置在限位外）
        scaled.Scale = 0.05;
        if (CheckViolations(Amplify(SimulateCore(scaled, dtSec, accel, null), lim), scaled, lim).Count > 0)
            return 0.0;

        double lo = 0.05, hi = 1.0;
        for (int i = 0; i < 12; i++)
        {
            double mid = (lo + hi) / 2;
            scaled.Scale = mid;
            bool ok = CheckViolations(Amplify(SimulateCore(scaled, dtSec, accel, null), lim), scaled, lim).Count == 0;
            if (ok) lo = mid; else hi = mid;
        }
        return Math.Clamp(lo * 0.98, 0.0, 1.0);   // 留 2% 余量，避免正好卡在判限边界
    }

    /// <summary>
    /// 时空等比缩放包装器：输出速度乘 s、回馈位置除以 s——对时间驱动模式（正弦/多正弦/PRTS，
    /// 忽略 curPos）等价于幅值缩 s；对位置目标驱动模式（方波途经点/脉冲峰位 Pk=v·ti/2）能把
    /// 目标位置也同步缩到 s 倍，与“一键缩放”对各自参数表的缩放语义一致。
    /// </summary>
    private sealed class ScaledGenerator : IWaveformGenerator
    {
        private readonly IWaveformGenerator _inner;
        public double Scale { get; set; } = 1.0;

        public ScaledGenerator(IWaveformGenerator inner) => _inner = inner;

        public string ModeName => _inner.ModeName;
        public bool ContinuousVelocity => _inner.ContinuousVelocity;
        public double TotalDurationSeconds => _inner.TotalDurationSeconds;
        public void Reset() => _inner.Reset();
        public double NextVelocity(double tSec, double curPosMm) => _inner.NextVelocity(tSec, curPosMm / Scale) * Scale;
        public bool IsFinished(double tSec) => _inner.IsFinished(tSec);
    }
}
