namespace MotorControlApp.Waveforms;

/// <summary>
/// 单正弦转速控制（直线往复）：负载线速度按正弦 v(t)=An·sin(2πf·t+ψ)+n0 变化，
/// 在行程内往返跑 T·f 个周期。速度幅值 An 与位置幅值 S 由 An=2πf·S 耦合，
/// 峰值加速度 amax=(2πf)²·S。周期间可插入 dwell 秒的零速间歇（0=无间歇）。
/// </summary>
public sealed class SineGenerator : IWaveformGenerator
{
    private readonly double _omega;      // 2πf (rad/s)
    private readonly double _an;         // 速度峰值 An (mm/s)
    private readonly double _bias;       // 直流偏置 n0 (mm/s)
    private readonly double _phase;      // 初相 ψ (rad)
    private readonly double _period;     // 1/f (s)
    private readonly double _dwell;      // 周期间歇 (s)
    private readonly double _duration;   // 总时长 T (s)

    public SineGenerator(double freqHz, double strokeMm, double biasMmS, double phaseRad,
                         double durationSec, double dwellSec)
    {
        if (freqHz <= 0) throw new ArgumentOutOfRangeException(nameof(freqHz), "频率必须 > 0");
        if (strokeMm <= 0) throw new ArgumentOutOfRangeException(nameof(strokeMm), "行程幅值 S 必须 > 0");
        if (durationSec <= 0) throw new ArgumentOutOfRangeException(nameof(durationSec), "总时长 T 必须 > 0");

        _omega = 2 * Math.PI * freqHz;
        _an = _omega * strokeMm;                 // An = 2πf·S
        _bias = biasMmS;
        _phase = phaseRad;
        _period = 1.0 / freqHz;
        _dwell = Math.Max(0, dwellSec);
        _duration = durationSec;

        StrokeMm = strokeMm;
        FreqHz = freqHz;
        PeakAccelMmS2 = _omega * _omega * strokeMm;   // amax = (2πf)²·S
    }

    /// <summary>速度峰值 An (mm/s)。</summary>
    public double AmplitudeMmS => _an;

    /// <summary>位置幅值 S (mm)。</summary>
    public double StrokeMm { get; }

    /// <summary>频率 f (Hz)。</summary>
    public double FreqHz { get; }

    /// <summary>峰值加速度 amax (mm/s²)。</summary>
    public double PeakAccelMmS2 { get; }

    public string ModeName => "单正弦";

    /// <summary>正弦速度连续可导，加速度由波形本身决定，需判限。</summary>
    public bool ContinuousVelocity => true;

    public double TotalDurationSeconds => _duration;

    public void Reset() { /* 纯时间驱动，无内部状态 */ }

    public double NextVelocity(double tSec, double curPosMm)
    {
        // 每个"周期块"= 正弦段(_period) + 间歇段(_dwell)
        double block = _period + _dwell;
        double inBlock = block > 0 ? tSec % block : 0;
        if (_dwell > 0 && inBlock >= _period)
            return 0;                                // 间歇段：零速
        return _bias + _an * Math.Sin(_omega * inBlock + _phase);
    }

    public bool IsFinished(double tSec) => tSec >= _duration;
}
