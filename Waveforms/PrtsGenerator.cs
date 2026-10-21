namespace MotorControlApp.Waveforms;

/// <summary>
/// 伪随机转速控制（PRTS / MPRTS 思路）：在 +V / 0 / -V 三种速度态间按伪随机顺序切换
/// （每个状态连续保持 K 拍），再积分生成平台位置轨迹。
/// 生成速度表时按控制器加减速上限（取加速度/减速度较小者的 90%）做逐拍斜坡，
/// 位置用梯形积分，并用刹车距离 v²/(2a) 做边界前瞻：规划位置与加速度受限的实际位置一致，
/// 不会像阶跃指令那样每次换向留下跟随欠账并累积成漂移。
/// 固定种子离线生成整条序列，保证：① 实际位置波动限制在 ±S 内；② 周期末位置归零；
/// ③ 每次运行完全复现。末端先按同一斜坡停稳，再低速回零。
/// </summary>
public sealed class PrtsGenerator : IWaveformGenerator
{
    private const double DefaultAccelMmS2 = 5000.0;
    /// <summary>规划加速度取控制器加减速上限的比例：剩余 10% 吸收一拍指令延迟、死区与跟随残差。</summary>
    private const double PlanAccelRatio = 0.9;
    /// <summary>内部位置规划边界相对用户 S 的比例：保证实际位置不越过 ±S 与校验窗。</summary>
    private const double PlanBoundRatio = 0.9;
    private const double ZeroTolMm = 0.2;

    private readonly double[] _schedule;   // 每拍速度 (mm/s)
    private readonly double _dt;
    private readonly double _duration;
    private readonly double _stateSpeedMmS;

    /// <summary>生成速度表时使用的规划加速度 (mm/s²) = 0.9 × min(加速度上限, 减速度上限)。</summary>
    public double PlanningAccelMmS2 { get; }

    /// <summary>从 +V 斜坡反转到 −V 所需拍数；K 小于该值时，状态时间内到不了反向全速，更谈不上驻留。</summary>
    public int FullReversalTicks
    {
        get
        {
            double ticks = 2.0 * _stateSpeedMmS / (PlanningAccelMmS2 * _dt);
            return ticks >= int.MaxValue ? int.MaxValue : Math.Max(1, (int)Math.Ceiling(ticks));
        }
    }

    public PrtsGenerator(double stateSpeedMmS, int picksPerStateK, double boundMmS,
        double durationSec, int seed, double dtSec, double accelMmS2 = 0, double decelMmS2 = 0)
    {
        if (stateSpeedMmS <= 0) throw new ArgumentOutOfRangeException(nameof(stateSpeedMmS), "PRTS 速度幅值必须 > 0");
        if (picksPerStateK < 1) throw new ArgumentOutOfRangeException(nameof(picksPerStateK), "K 必须 >= 1");
        if (boundMmS <= 0) throw new ArgumentOutOfRangeException(nameof(boundMmS), "位置波动 S 必须 > 0");
        if (durationSec <= 0) throw new ArgumentOutOfRangeException(nameof(durationSec), "时长必须 > 0");
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec), "控制周期必须 > 0");

        _dt = dtSec;
        double V = Math.Min(stateSpeedMmS, MotionUnits.VmaxMmS);
        _stateSpeedMmS = V;
        double S = boundMmS;
        if (V * _dt > S)
            throw new ArgumentException(
                $"位置波动 S={S:G6} mm 小于一拍全速位移 V·dt={V * _dt:G6} mm：全速态无法在不越界的情况下保持，请增大 S 或减小 V/控制周期",
                nameof(boundMmS));

        double aAcc = accelMmS2 > 0 ? accelMmS2 : DefaultAccelMmS2;
        double aDec = decelMmS2 > 0 ? decelMmS2 : aAcc;
        double aPlan = PlanAccelRatio * Math.Min(aAcc, aDec);
        if (!(aPlan > 0)) aPlan = PlanAccelRatio * DefaultAccelMmS2;
        PlanningAccelMmS2 = aPlan;
        double maxDv = aPlan * _dt;
        double bound = PlanBoundRatio * S;
        int steps = Math.Max(1, (int)Math.Round(durationSec / _dt));
        int kHold = Math.Min(picksPerStateK, steps);   // 超过总拍数的保持时间没有意义，也避免可行域模拟过长

        var rnd = new Random(seed);
        var list = new List<double>(steps + 64);
        double pos = 0, vOut = 0, target = 0;
        int hold = 0;

        while (list.Count < steps)
        {
            if (hold <= 0)
            {
                (target, hold) = PickState(rnd, V, pos, vOut, _dt, kHold, bound, aPlan, maxDv);
                if (hold < 1) hold = 1;
            }

            double vNew = Slew(vOut, target, maxDv);
            double pNew = pos + (vOut + vNew) / 2 * _dt;

            // PickState 已按刹车包络筛选；这里兜住浮点/极端参数，保证任何时刻都还能在边界内停住。
            if (BrakeEnvelope(pNew, vNew, aPlan) > bound + 1e-9)
            {
                double vStop = Slew(vOut, 0, maxDv);
                double pStop = pos + (vOut + vStop) / 2 * _dt;
                if (BrakeEnvelope(pStop, vStop, aPlan) <= bound + 1e-9)
                {
                    vNew = vStop;
                    pNew = pStop;
                    target = 0;
                }
                else
                {
                    target = pos > 0 ? -V : pos < 0 ? V : 0;
                    vNew = Slew(vOut, target, maxDv);
                    pNew = pos + (vOut + vNew) / 2 * _dt;
                }
            }

            list.Add(vNew);
            pos = pNew;
            vOut = vNew;
            hold--;
        }

        // 末端先按同一斜坡停稳（刹车包络保证仍在内部边界内）
        int guard = 0;
        while (Math.Abs(vOut) > 0.5 && guard++ < 20000)
        {
            double vNew = Slew(vOut, 0, maxDv);
            pos += (vOut + vNew) / 2 * _dt;
            list.Add(vNew);
            vOut = vNew;
        }
        if (Math.Abs(vOut) <= 0.5) vOut = 0;

        // 低速回零：速度上限 50 mm/s，接近 0 时按刹车速度收敛；过零后交给收尾段减速。
        guard = 0;
        while (Math.Abs(pos) > ZeroTolMm && guard++ < 50000)
        {
            double vDes = -Math.Sign(pos) * Math.Min(50.0, Math.Sqrt(2 * 0.8 * aPlan * Math.Abs(pos)));
            double vNew = Slew(vOut, vDes, maxDv);
            double pNew = pos + (vOut + vNew) / 2 * _dt;

            if (Math.Sign(pNew) != Math.Sign(pos) && Math.Abs(pNew) < 0.5)
            {
                // 过零这一拍仍按斜坡下发，交由下面的收尾段逐拍减速到 0；
                // 不能直接写 0，否则会留下最后一拍 >maxDv 的速度阶跃。
                list.Add(vNew);
                pos = pNew;
                vOut = vNew;
                break;
            }

            list.Add(vNew);
            pos = pNew;
            vOut = vNew;
        }

        // 残余速度按同一斜坡收尾，随后留静止段让实际轴停稳
        guard = 0;
        while (Math.Abs(vOut) > 0.5 && guard++ < 2000)
        {
            double vNew = Slew(vOut, 0, maxDv);
            pos += (vOut + vNew) / 2 * _dt;
            list.Add(vNew);
            vOut = vNew;
        }

        for (int i = 0; i < 20; i++) list.Add(0);

        _schedule = list.ToArray();
        _duration = _schedule.Length * _dt;
    }

    /// <summary>
    /// 由“显式速度序列”直接构造：表内每个 token 是控制周期 dt 的采样（不做斜坡、前瞻约束、不自动回零），
    /// 用于 100% 复现任意手绘/指定的三态形状。回放按墙钟时间取表，位置为序列的积分，是否超行程由校验器判限。
    /// </summary>
    public static PrtsGenerator CreateFromSequence(IReadOnlyList<double> scheduleMmS, double dtSec)
    {
        ArgumentNullException.ThrowIfNull(scheduleMmS);
        if (scheduleMmS.Count == 0) throw new ArgumentException("显式速度序列不能为空", nameof(scheduleMmS));
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec), "控制周期必须 > 0");
        return new PrtsGenerator(scheduleMmS.ToArray(), dtSec);
    }

    // 显式序列用的私有构造：直接持有逐拍速度表，时长 = 拍数 × dt。
    private PrtsGenerator(double[] schedule, double dtSec)
    {
        _schedule = schedule;
        _dt = dtSec;
        _duration = schedule.Length * dtSec;
        double maxAbsV = 0;
        foreach (double v in schedule) maxAbsV = Math.Max(maxAbsV, Math.Abs(v));
        _stateSpeedMmS = maxAbsV;
        PlanningAccelMmS2 = PlanAccelRatio * DefaultAccelMmS2;
    }

    /// <summary>
    /// 选下一个速度态：洗牌 +V/−V/0，优先选能在刹车包络内完整保持 K 拍的态；
    /// 边界附近放不下完整 K 拍时缩短非零态的保持拍数；仍不可行则朝内侧走一拍。
    /// </summary>
    private static (double target, int hold) PickState(Random rnd, double V, double pos, double v,
        double dt, int kHold, double bound, double aPlan, double maxDv)
    {
        int[] order = { 1, -1, 0 };
        for (int i = order.Length - 1; i > 0; i--)
        {
            int j = rnd.Next(i + 1);
            (order[i], order[j]) = (order[j], order[i]);
        }

        // 与旧逻辑一致：只要存在能完整保持 K 拍的 ±V 态，就在洗牌顺序里选第一个可行态（0 也参与）；
        // 若 ±V 都放不下完整 K 拍，则跳过 0，避免位置静止锁死整条序列。
        bool anyNonZeroFull = false;
        foreach (int s in order)
        {
            if (s != 0 && Feasible(pos, v, s * V, kHold, dt, bound, aPlan, maxDv))
            {
                anyNonZeroFull = true;
                break;
            }
        }
        if (anyNonZeroFull)
        {
            foreach (int s in order)
                if (Feasible(pos, v, s * V, kHold, dt, bound, aPlan, maxDv))
                    return (s * V, kHold);
        }

        foreach (int s in order)
        {
            if (s == 0) continue;
            int h = MaxFeasibleTicks(pos, v, s * V, kHold, dt, bound, aPlan, maxDv);
            if (h >= 1) return (s * V, h);
        }

        // 构造时已保证 V·dt ≤ S；刹车包络成立时朝内侧一拍一定可行。
        double inward = pos > 0 ? -V : pos < 0 ? V : (rnd.Next(2) == 0 ? V : -V);
        return (inward, 1);
    }

    /// <summary>按目标态连续仿真 ticks 拍，检查每一拍都满足 |p| + v²/(2a) ≤ bound。</summary>
    private static bool Feasible(double pos, double v, double target, int ticks,
        double dt, double bound, double aPlan, double maxDv)
    {
        double p = pos, vv = v;
        for (int i = 0; i < ticks; i++)
        {
            double vn = Slew(vv, target, maxDv);
            p += (vv + vn) / 2 * dt;
            vv = vn;
            if (BrakeEnvelope(p, vv, aPlan) > bound + 1e-9) return false;
        }
        return true;
    }

    /// <summary>固定目标速度下最多能连续保持几拍仍满足刹车包络；0 表示一拍都放不下。</summary>
    private static int MaxFeasibleTicks(double pos, double v, double target, int maxTicks,
        double dt, double bound, double aPlan, double maxDv)
    {
        double p = pos, vv = v;
        int last = 0;
        for (int i = 0; i < maxTicks; i++)
        {
            double vn = Slew(vv, target, maxDv);
            p += (vv + vn) / 2 * dt;
            vv = vn;
            if (BrakeEnvelope(p, vv, aPlan) > bound + 1e-9) break;
            last = i + 1;
        }
        return last;
    }

    /// <summary>每拍速度变化不超过 maxDv（= 规划加速度 × dt）。</summary>
    private static double Slew(double v, double target, double maxDv) =>
        v + Math.Clamp(target - v, -maxDv, maxDv);

    /// <summary>刹车包络：当前位置 + 以规划加速度刹停所需距离。</summary>
    private static double BrakeEnvelope(double pos, double v, double aPlan) =>
        Math.Abs(pos) + v * v / (2 * aPlan);

    public string ModeName => "伪随机PRTS";

    public double TotalDurationSeconds => _duration;

    public void Reset() { }

    public double NextVelocity(double tSec, double curPosMm)
    {
        // 表内 token 是控制周期 dt 的采样，但回放必须按墙钟时间推进：
        // 定时器实际回调间隔（Windows 默认约 14.6ms）不等于配置的控制周期，
        // 用调用次数当拍数会把每个 token 的保持时间拉长，行程与时长同步放大。
        // 按 t/dt 取表并做段内线性插值：在 t=i·dt 处正好返回第 i 个 token，
        // 与校验器的逐拍采样完全一致；真机在任意时刻调用得到该连续重建值。
        double x = Math.Max(0, tSec) / _dt;
        int i = (int)x;
        if (i >= _schedule.Length) return 0;
        if (i == _schedule.Length - 1) return _schedule[i];
        double frac = x - i;
        return _schedule[i] + (_schedule[i + 1] - _schedule[i]) * frac;
    }

    public bool IsFinished(double tSec) => tSec >= _duration;
}
