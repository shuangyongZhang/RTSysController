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
/// 实际速度每拍向指令逼近：|v| 增大按加速度上限、减小按运行期减速度（方波/脉冲/PRTS 各自页签），
/// 跨零换向先用减速度停到 0 再加速（与真机 Vmove+Speed/Single_Cancel 按 ACC/DEC 斜坡逼近的行为一致），
/// 行程由实际速度积分。因此启动瞬态欠账（如 ψ=±90° 首拍速度阶跃）与端点换向超程
/// 都被计入预测行程，评估结果与真机实测同量级，而不是理想跟随的下界。
/// 对"叠加后的瞬时峰值"（含相长干涉）做限速/限行程/限加速度检查。
/// 位置反馈闭环波形（方波/脉冲）的真实回调周期是 Windows 定时器网格（15.625 ms）的整数倍，
/// 故改用保守拍长仿真，并在包络两侧各加一拍满速行程的反馈滞后余量。
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

    /// <summary>跟随模型使用的减速度限：优先取运行期减速度（方波/脉冲/PRTS 各自页签），未设时回落到加速度上限（与真机一致）。</summary>
    internal static double DecelFor(MotionLimits? lim) =>
        lim is { DecelLimitMmS2: > 0 } ? lim.DecelLimitMmS2 : AccelFor(lim);

    /// <summary>Windows 默认定时器网格 15.625 ms：PeriodicTimer 的实际回调落在它的整数倍附近。</summary>
    private const double TimerGridSec = 1.0 / 64.0;

    /// <summary>网格之上的调度抖动余量（实测 10~100 ms 配置的 p90 均在 1.07 倍网格内）。</summary>
    private const double RealTickGuard = 1.10;

    /// <summary>
    /// 校验/预览用的仿真周期。位置反馈波形（方波/脉冲）的到达判定按真机实际回调周期采样，
    /// 而实际周期是 15.625 ms 定时器网格的整数倍且带抖动（配置 10 ms 实测中位数 ≈15.5 ms）；
    /// 故按网格向上取整再留 10% 余量，得到不低估超程的保守周期。
    /// 时间驱动波形（正弦/多正弦/PRTS）的指令只由时间决定，与原 dt 一致。
    /// </summary>
    internal static double SimDtFor(IWaveformGenerator g, double dtSec) =>
        g.PositionFeedback
            ? Math.Max(dtSec, Math.Ceiling(dtSec / TimerGridSec) * TimerGridSec * RealTickGuard)
            : dtSec;

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
    /// 核心仿真（校验与轨迹预览共用）：指令速度 → 一阶加减速跟随 → 梯形积分实际位置。
    /// onTick(t, vCmd, posMm) 逐拍回调（预览采样用），null 为纯统计。
    /// </summary>
    internal static SimCoreResult SimulateCore(IWaveformGenerator g, double dtSec, double accelMmS2,
        double decelMmS2, Action<double, double, double>? onTick)
    {
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec));
        if (accelMmS2 <= 0) accelMmS2 = DefaultAccelMmS2;
        if (decelMmS2 <= 0) decelMmS2 = accelMmS2;   // 与运行期“减速度 <=0 回退加速度上限”一致

        g.Reset();
        double t = 0, pos = 0, prevV = 0, vAct = 0;
        double maxAbsV = 0, minX = 0, maxX = 0, maxA = 0, sumV = 0;
        int count = 0;
        double total = g.TotalDurationSeconds;
        bool ended = false;

        while (count < 5_000_000 && t <= MaxSimSeconds)
        {
            double v = g.NextVelocity(t, pos);
            double a = count == 0 ? 0 : Math.Abs(v - prevV) / dtSec;

            // 一阶跟随模型：真机控制器以 ACC/DEC 斜坡逼近指令速度（|v| 增大用 ACC、减小用 DEC，
            // 换向先停到 0 再反向起速）。另加一拍指令传输延迟（本拍指令经网络+插补+驱动响应约
            // 下一拍才生效，保守取 prevV，首拍取 0）——启动速度阶跃的欠账因此更接近真机实测的
            // 永久性中心偏移。位置由实际速度梯形积分——启动瞬态欠账、端点换向超程都会如实进入预测行程。
            double vTarget = count == 0 ? 0 : prevV;
            double vActNew = SlewOneTick(vAct, vTarget, accelMmS2, decelMmS2, dtSec);
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
    /// 控制器速度斜坡的一拍推进：同向变速时 |v| 增大按 ACC、减小按 DEC；跨零换向先用 DEC
    /// 减到 0、剩余时间再按 ACC 加速到目标（对应 Vmove+SetSpeed 与 Single_Cancel 的减速行为）。
    /// accel==decel 时与旧的对称钳制完全等价，正弦等原模式的预测结果不变。
    /// </summary>
    private static double SlewOneTick(double vAct, double vTarget, double accelMmS2, double decelMmS2, double dtSec)
    {
        double absAct = Math.Abs(vAct), absTgt = Math.Abs(vTarget);
        double maxAcc = accelMmS2 * dtSec, maxDec = decelMmS2 * dtSec;

        if (absAct < 1e-12)
            return Math.Sign(vTarget) * Math.Min(absTgt, maxAcc);

        if (Math.Sign(vAct) == Math.Sign(vTarget))
        {
            if (absTgt >= absAct)
                return vAct + Math.Sign(vTarget) * Math.Min(absTgt - absAct, maxAcc);
            return vAct - Math.Sign(vAct) * Math.Min(absAct - absTgt, maxDec);
        }

        // 目标为 0 或反向：本拍先按减速度往 0 靠，能停住则剩余时间再按加速度起速
        if (absAct > maxDec)
            return vAct - Math.Sign(vAct) * maxDec;
        double remainSec = dtSec - absAct / decelMmS2;
        return Math.Sign(vTarget) * Math.Min(absTgt, accelMmS2 * remainSec);
    }

    /// <summary>
    /// 动态放大展开：预测包络中心不变、半幅×K。快速换向（如 2Hz 正弦）下真机存在机械/伺服
    /// 动态超调，实测振幅会大于指令振幅——速率受限的跟随模型原理上只能缩小振幅、无法预测放大，
    /// 故用自校准得到的系数 K 把包络撑开罩住实测。仅连续速度模式（单正弦/多正弦）生效，
    /// 方波/脉冲/PRTS 的 K 不参与（界面也隐藏该行）。K≈1 时原样返回。
    /// </summary>
    private static SimCoreResult Amplify(SimCoreResult c, IWaveformGenerator g, MotionLimits? lim)
    {
        double k = g.ContinuousVelocity ? (lim?.DynAmpK ?? 1.0) : 1.0;
        if (k <= 1.0 + 1e-9) return c;
        double ctr = (c.MinPos + c.MaxPos) / 2, half = (c.MaxPos - c.MinPos) / 2 * k;
        return new SimCoreResult
        {
            PeakV = c.PeakV, PeakA = c.PeakA, MeanV = c.MeanV, EndT = c.EndT, Ended = c.Ended,
            MinPos = ctr - half, MaxPos = ctr + half,
        };
    }

    /// <summary>
    /// 位置反馈波形（方波/脉冲）的反馈位置只在实际拍点采样，到达判定最坏会滞后一个拍；
    /// 包络向两侧各放宽 v_peak×T（一拍满速行程），覆盖"目标恰好落在两拍之间"的相位，
    /// 而不是赌仿真起点恰好落在有利相位。时间驱动波形不加。
    /// </summary>
    private static SimCoreResult WidenForFeedback(SimCoreResult c, IWaveformGenerator g, double simDtSec)
    {
        if (!g.PositionFeedback) return c;
        double margin = c.PeakV * simDtSec;
        return new SimCoreResult
        {
            PeakV = c.PeakV, PeakA = c.PeakA, MeanV = c.MeanV, EndT = c.EndT, Ended = c.Ended,
            MinPos = c.MinPos - margin, MaxPos = c.MaxPos + margin,
        };
    }

    /// <summary>校验/缩放搜索共用的完整预测包络（跟随仿真 → 动态放大 → 反馈采样余量）。</summary>
    private static SimCoreResult Predict(IWaveformGenerator g, double simDtSec, MotionLimits? lim) =>
        WidenForFeedback(Amplify(SimulateCore(g, simDtSec, AccelFor(lim), DecelFor(lim), null), g, lim), g, simDtSec);

    /// <summary>判限用的峰值加速度：生成器给出解析值时优先用它，否则用逐拍统计值。</summary>
    private static double PeakAccelFor(IWaveformGenerator g, SimCoreResult c) =>
        g.AnalyticalPeakAccelMmS2 > 0 ? g.AnalyticalPeakAccelMmS2 : c.PeakA;

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
            // 行程判限：任何限制不通过都禁止启动，与软限位保护开关无关。
            // 开关仅决定运行中碰限位是否停、以及判限是否预留 5% 安全余量：
            // 开启时按限位窗缩 5% 判限（跟随模型与真机必有残差，不把残差压到限位边上赌）；
            // 关闭时按原始限位窗判（运行中限位处不停，更不能超窗启动）。
            const double MarginFrac = 0.05;
            double posAllow = !lim.EnforceSoftLimit || lim.PosMm <= 0 ? lim.PosMm : lim.PosMm * (1 - MarginFrac);
            double negAllow = !lim.EnforceSoftLimit || lim.NegMm >= 0 ? lim.NegMm : lim.NegMm * (1 - MarginFrac);
            if (c.MaxPos > posAllow + 1e-6 || c.MinPos < negAllow - 1e-6)
                violations.Add(lim.EnforceSoftLimit
                    ? $"预测行程 [{c.MinPos:F1}, {c.MaxPos:F1}] mm 超出安全行程 [{negAllow:F1}, {posAllow:F1}] mm（限位含 5% 余量，已计入加减速跟随误差与动态放大系数）"
                    : $"预测行程 [{c.MinPos:F1}, {c.MaxPos:F1}] mm 超出限位窗 [{negAllow:F1}, {posAllow:F1}] mm，禁止启动（软限位保护已关闭，运行中限位处不停，请修改参数使行程限位窗内）");

            // 指令加速度判限仅对连续速度波形有意义；阶梯型（方波/脉冲/PRTS）的 Δv/dt 是指令阶跃假象，
            // 真机由控制器 Accel/Decel 钳制（跟随模型已把该钳制计入行程预测），故不据此报超限。
            double peakA = PeakAccelFor(g, c);
            if (g.AccelCheckMeaningful && lim.AccelLimitMmS2 > 0 && peakA > lim.AccelLimitMmS2 + 1e-6)
                violations.Add($"峰值加速度 {peakA:F0} mm/s² 超过上限 {lim.AccelLimitMmS2:F0} mm/s²");
        }
        return violations;
    }

    public static WaveformSimResult Simulate(IWaveformGenerator g, double dtSec, MotionLimits? lim)
    {
        double simDt = SimDtFor(g, dtSec);
        var core = Predict(g, simDt, lim);
        var violations = CheckViolations(core, g, lim);

        // 奈奎斯特：仿真与真机都按 dt 逐拍采样指令速度，频率超过 1/(2dt) 会发生混叠，
        // 峰值速度/行程被低估且与真机不一致；缩放幅值无法解决，故直接禁止启动。
        double fMax = g.MaxFrequencyHz;
        double fNyquist = 0.5 / dtSec;
        bool aliased = fMax > fNyquist + 1e-9;
        if (aliased)
            violations.Insert(0,
                $"最高频率分量 {fMax:F2} Hz 超过奈奎斯特频率 {fNyquist:F2} Hz（控制周期 {dtSec * 1000:0.###} ms），采样会混叠，预测峰值不可信；请降低频率或减小控制周期");

        double scale = aliased ? 0.0 : SuggestScale(g, simDt, lim, core, violations);

        return new WaveformSimResult
        {
            PeakVelocityMmS = core.PeakV,
            MinPosMm = core.MinPos,
            MaxPosMm = core.MaxPos,
            PeakAccelMmS2 = PeakAccelFor(g, core),
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
    private static double SuggestScale(IWaveformGenerator g, double simDtSec, MotionLimits? lim,
        SimCoreResult core, IReadOnlyList<string> violations)
    {
        if (violations.Count == 0) return 1.0;
        if (!core.Ended) return 0.0;   // 不收敛/超长，缩放无意义

        var scaled = new ScaledGenerator(g);

        // 下限探测：极小幅值仍超限 → 缩放救不了（典型：方波途经点位置在限位外）
        scaled.Scale = 0.05;
        if (CheckViolations(Predict(scaled, simDtSec, lim), scaled, lim).Count > 0)
            return 0.0;

        double lo = 0.05, hi = 1.0;
        for (int i = 0; i < 12; i++)
        {
            double mid = (lo + hi) / 2;
            scaled.Scale = mid;
            bool ok = CheckViolations(Predict(scaled, simDtSec, lim), scaled, lim).Count == 0;
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
        public bool AccelCheckMeaningful => _inner.AccelCheckMeaningful;
        public bool PositionFeedback => _inner.PositionFeedback;
        public bool IsSettling => _inner.IsSettling;
        public double AnalyticalPeakAccelMmS2 => _inner.AnalyticalPeakAccelMmS2 * Scale;
        public double MaxFrequencyHz => _inner.MaxFrequencyHz;
        public double TotalDurationSeconds => _inner.TotalDurationSeconds;
        public void Reset() => _inner.Reset();
        public double NextVelocity(double tSec, double curPosMm) => _inner.NextVelocity(tSec, curPosMm / Scale) * Scale;
        public bool IsFinished(double tSec) => _inner.IsFinished(tSec);
    }
}
