namespace MotorControlApp.Waveforms;

/// <summary>
/// 伪随机转速控制（PRTS / MPRTS 思路）：在 +V / 0 / -V 三种速度态间按伪随机顺序切换
/// （每个状态连续保持 K 拍），再积分生成平台位置轨迹。
/// 用固定随机种子离线生成整条速度序列，保证：① 位置波动限制在 ±S 内；② 周期末位置归零；
/// ③ 每次运行完全复现。序列构造时逐步积分并对边界做前瞻约束，末端追加回零段。
/// </summary>
public sealed class PrtsGenerator : IWaveformGenerator
{
    private readonly double[] _schedule;   // 每拍速度 (mm/s)
    private readonly double _dt;
    private readonly double _duration;
    private int _tick;                     // 内部拍计数器（不依赖墙钟时间，消除 Timer 抖动致索引跳/重）

    public PrtsGenerator(double stateSpeedMmS, int picksPerStateK, double boundMmS,
        double durationSec, int seed, double dtSec)
    {
        if (stateSpeedMmS <= 0) throw new ArgumentOutOfRangeException(nameof(stateSpeedMmS), "PRTS 速度幅值必须 > 0");
        if (picksPerStateK < 1) throw new ArgumentOutOfRangeException(nameof(picksPerStateK), "K 必须 >= 1");
        if (boundMmS <= 0) throw new ArgumentOutOfRangeException(nameof(boundMmS), "位置波动 S 必须 > 0");
        if (durationSec <= 0) throw new ArgumentOutOfRangeException(nameof(durationSec), "时长必须 > 0");
        if (dtSec <= 0) throw new ArgumentOutOfRangeException(nameof(dtSec), "控制周期必须 > 0");

        _dt = dtSec;
        double V = Math.Min(stateSpeedMmS, MotionUnits.VmaxMmS);
        double S = boundMmS;
        double lookahead = 0.95 * S;                 // 给边界留刹车余量
        int steps = Math.Max(1, (int)Math.Round(durationSec / _dt));

        var rnd = new Random(seed);
        var list = new List<double>(steps + 64);
        double pos = 0;
        int held = 0;
        double curV = 0;

        for (int i = 0; i < steps; i++)
        {
            if (held == 0)
            {
                curV = PickState(rnd, V, pos, _dt, picksPerStateK, lookahead);
                held = picksPerStateK;
            }

            double newPos = pos + curV * _dt;
            if (Math.Abs(newPos) > S)               // 兜底：即将越界则强制回拉
            {
                curV = -Math.Sign(pos) * V;
                newPos = pos + curV * _dt;
                held = 1;
            }

            list.Add(curV);
            pos = newPos;
            held--;
        }

        // 末端追加回零段，保证周期末 x≈0
        // 容差必须 >= 一拍全速位移，否则位置在 ±V*dt 间来回振荡永不收敛
        double posTol = Math.Max(0.5, V * _dt);
        int guard = 0;
        while (Math.Abs(pos) > posTol && guard++ < 4096)
        {
            // 若剩余距离不足一拍全速位移，直接停住避免越零振荡
            double stepDist = V * _dt;
            double v;
            if (Math.Abs(pos) <= stepDist)
                v = 0;
            else
                v = Math.Sign(pos) > 0 ? -V : V;
            pos += v * _dt;
            list.Add(v);
        }
        if (list.Count > 0) list[^1] = 0;            // 收尾一拍静止
        list.Add(0);

        _schedule = list.ToArray();
        _duration = _schedule.Length * _dt;
    }
    
    /// <summary>
    /// 由“显式速度序列”直接构造：逐拍原样下发（不做前瞻约束、不自动回零），
    /// 用于 100% 复现任意手绘/指定的三态形状。位置为序列的积分，是否超行程由校验器判限。
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
    }

    private static double PickState(Random rnd, double V, double pos, double dt, int kHold, double limit)
    {
        // 候选顺序随机（+V / -V / 0），选第一个能在此后 kHold 拍内不越界的
        int[] order = { 1, -1, 0 };
        // Fisher-Yates 洗牌
        for (int i = order.Length - 1; i > 0; i--)
        {
            int j = rnd.Next(i + 1);
            (order[i], order[j]) = (order[j], order[i]);
        }

        foreach (int s in order)
        {
            double v = s * V;
            double projected = pos + v * dt * kHold;
            if (Math.Abs(projected) <= limit)
                return v;
        }
        // 都不行：朝 0 方向回拉
        return -Math.Sign(pos) * V;
    }

    public string ModeName => "伪随机PRTS";

    public double TotalDurationSeconds => _duration;

    public void Reset() => _tick = 0;

    public double NextVelocity(double tSec, double curPosMm)
    {
        // 用内部拍计数器而非墙钟 t/dt：实时循环中 PeriodicTimer 有 OS 抖动，
        // (int)(t/dt) 可能重复或跳过某些拍，导致显式序列不能逐拍精确下发。
        int tick = _tick++;
        if (tick < 0) tick = 0;
        if (tick >= _schedule.Length) return 0;
        return _schedule[tick];
    }

    public bool IsFinished(double tSec) => _tick >= _schedule.Length || tSec >= _duration;
}
