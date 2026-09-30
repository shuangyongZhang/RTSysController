namespace MotorControlApp.Waveforms;

/// <summary>单个正弦分量：频率(Hz)、幅值(mm/s)、初相(rad)。</summary>
public readonly record struct SineComponent(double FreqHz, double AmpMmS, double PhaseRad);

/// <summary>多正弦分量的相位生成策略。</summary>
public enum MsPhaseMode
{
    /// <summary>全部 0 相位（最容易相长干涉，峰值≈ΣAk）。</summary>
    Zero = 0,
    /// <summary>奇偶交替 0/π。</summary>
    Alternate = 1,
    /// <summary>按随机种子生成 0~2π（更接近平凡频谱的随机相位合成）。</summary>
    Random = 2,
}

/// <summary>
/// 多正弦叠加转速控制（傅里叶级数合成）：n(t) = Σ(k=1..N) Ak·sin(2π·fk·t + φk)，
/// 用 16~32 个频率成分合成较复杂但确定、可复现的连续周期波形。
/// fk 取基频 f0 整数倍 → 合成为周期信号、均值≈0、位置有界不漂移。
/// 峰值速度/行程由叠加后的瞬时值决定（可能相长干涉），须靠 SynthesisPreview 全时域校验。
/// </summary>
public sealed class MultiSineGenerator : IWaveformGenerator
{
    private readonly SineComponent[] _comps;
    private readonly double _duration;

    public MultiSineGenerator(IReadOnlyList<SineComponent> components, double durationSec)
    {
        ArgumentNullException.ThrowIfNull(components);
        if (components.Count == 0) throw new ArgumentException("至少需要一个正弦分量", nameof(components));
        if (durationSec <= 0) throw new ArgumentOutOfRangeException(nameof(durationSec), "总时长 T 必须 > 0");

        _comps = components.ToArray();
        _duration = durationSec;
    }

    /// <summary>
    /// 频谱规则法展开成 N 个分量：fk=k·f0，Ak=A1/k^p，相位按 <paramref name="phaseMode"/> 生成。
    /// A1 为基频幅值(mm/s)。
    /// </summary>
    public static MultiSineGenerator CreateFromRule(double baseFreqHz, int count, double fundamentalAmpMmS,
        double decayP, MsPhaseMode phaseMode, int seed, double durationSec)
    {
        if (baseFreqHz <= 0) throw new ArgumentOutOfRangeException(nameof(baseFreqHz), "基频必须 > 0");
        if (count < 1) throw new ArgumentOutOfRangeException(nameof(count), "成分数必须 >= 1");
        count = Math.Min(count, 64);   // 上限保护

        var rnd = new Random(seed);
        var list = new List<SineComponent>(count);
        for (int k = 1; k <= count; k++)
        {
            double fk = baseFreqHz * k;
            double ak = fundamentalAmpMmS / Math.Pow(k, decayP);
            double phi = phaseMode switch
            {
                MsPhaseMode.Alternate => (k % 2 == 0) ? Math.PI : 0.0,
                MsPhaseMode.Random => rnd.NextDouble() * 2 * Math.PI,
                _ => 0.0,
            };
            list.Add(new SineComponent(fk, ak, phi));
        }
        return new MultiSineGenerator(list, durationSec);
    }

    /// <summary>分量列表（供 SynthesisPreview / UI 只读展示）。</summary>
    public IReadOnlyList<SineComponent> Components => _comps;

    /// <summary>所有分量幅值之和 ΣAk（同相叠加时的理论速度峰值上界）。</summary>
    public double SumAmplitudesMmS => _comps.Sum(c => Math.Abs(c.AmpMmS));

    public string ModeName => "多正弦叠加";

    /// <summary>多正弦为连续分量叠加，加速度由波形本身决定，需判限。</summary>
    public bool ContinuousVelocity => true;

    public double TotalDurationSeconds => _duration;

    public void Reset() { /* 纯时间驱动，无内部状态 */ }

    public double NextVelocity(double tSec, double curPosMm)
    {
        double v = 0;
        for (int i = 0; i < _comps.Length; i++)
        {
            var c = _comps[i];
            v += c.AmpMmS * Math.Sin(2 * Math.PI * c.FreqHz * tSec + c.PhaseRad);
        }
        return v;
    }

    public bool IsFinished(double tSec) => tSec >= _duration;
}
