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
        const double posTol = 0.5;                  // mm
        int guard = 0;
        while (Math.Abs(pos) > posTol && guard++ < 4096)
        {
            double v = Math.Sign(pos) > 0 ? -V : V;
            pos += v * _dt;
            list.Add(v);
        }
        if (list.Count > 0) list[^1] = 0;            // 收尾一拍静止
        list.Add(0);

        _schedule = list.ToArray();
        _duration = _schedule.Length * _dt;
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

    public void Reset() { /* 序列已离线确定，无内部状态 */ }

    public double NextVelocity(double tSec, double curPosMm)
    {
        int tick = (int)(tSec / _dt);
        if (tick < 0) tick = 0;
        if (tick >= _schedule.Length) return 0;
        return _schedule[tick];
    }

    public bool IsFinished(double tSec) => tSec >= _duration;
}
