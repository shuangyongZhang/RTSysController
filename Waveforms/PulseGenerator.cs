namespace MotorControlApp.Waveforms;

/// <summary>
/// 脉冲转速控制（闭环·位置反馈）：从基位(0)产生一次短时冲到峰值再退回基位。
/// 每个脉冲宽度 ti 对应名义峰值行程 Pk = v1·ti/2；用当前 Dpos 反馈判到达：
/// 先朝 +Pk 冲出（|curPos-Pk|&lt;tol 或本拍穿过即转回），再朝基位 0 退回（到位/穿过即转停留），
/// 脉冲间 gap 零速停留。durations 列表按顺序循环 n 次。
///
/// 相比旧的“纯时间驱动”（±v 各固定 ti/2 时长、忽略位置），这里回程按位置判是否回到基位，
/// 从根上消除了“控制器有限加速度使换向冲过头、固定时长回程回不到基线、逐脉冲累积漂移冲破行程限位”
/// 的问题：漂移不再随循环数累积。同时校验器/采样器把自身积分位置回喂给 NextVelocity，
/// 于是“启动前预估”与“真机实时曲线”都走同一套闭环逻辑，二者一致。
/// 注：真机在峰值处仍有一个由加速度决定的有限过冲（≈v²/2a，不累积），贴近软限位时应留一点余量。
/// </summary>
public sealed class PulseGenerator : IWaveformGenerator
{
    private enum Phase { Out, Back, Gap, Done }

    private readonly double _v;
    private readonly double[] _peaks;   // 每个脉冲的名义峰值行程(mm)
    private readonly int _cycles;
    private readonly double _gapSec;
    private readonly double _tolMm;

    private int _pulse;
    private int _cycle;
    private Phase _phase;
    private double _gapStartT;
    private double _prevErr = double.NaN;   // 上一拍误差，用于“穿过目标”判定

    public PulseGenerator(double speedMmS, IReadOnlyList<double> durationsSec, int cycles, double gapSec = 0, double arriveTolMm = 0.5)
    {
        ArgumentNullException.ThrowIfNull(durationsSec);
        if (speedMmS <= 0) throw new ArgumentOutOfRangeException(nameof(speedMmS), "脉冲速度必须 > 0");
        if (cycles < 1) throw new ArgumentOutOfRangeException(nameof(cycles), "循环次数必须 >= 1");
        if (durationsSec.Any(d => d <= 0)) throw new ArgumentException("脉冲宽度必须 > 0", nameof(durationsSec));

        _v = Math.Min(speedMmS, MotionUnits.VmaxMmS);
        _cycles = cycles;
        _gapSec = Math.Max(0, gapSec);
        _tolMm = Math.Max(0.01, arriveTolMm);
        // 名义峰值行程 v·ti/2（与旧纯时间驱动的单脉冲峰值一致）
        _peaks = durationsSec.Select(ti => _v * ti / 2.0).ToArray();
        Reset();
    }

    public string ModeName => "脉冲";

    /// <summary>由循环次数决定结束，无固定总时长（同方波）。</summary>
    public double TotalDurationSeconds => double.PositiveInfinity;

    public void Reset()
    {
        _pulse = 0;
        _cycle = 0;
        _phase = Phase.Out;
        _gapStartT = 0;
        _prevErr = double.NaN;
    }

    public double NextVelocity(double tSec, double curPosMm)
    {
        switch (_phase)
        {
            case Phase.Out:
            {
                double err = _peaks[_pulse] - curPosMm;   // 目标：名义峰值
                if (Arrived(err)) { EnterBack(); return 0; }
                _prevErr = err;
                return Math.Sign(err) * _v;               // 朝峰值冲出
            }

            case Phase.Back:
            {
                double err = 0 - curPosMm;                // 目标：基位 0
                if (Arrived(err)) { EnterGap(tSec); return 0; }
                _prevErr = err;
                return Math.Sign(err) * _v;               // 退回基位
            }

            case Phase.Gap:
                if (_gapSec <= 0 || tSec - _gapStartT >= _gapSec)
                    Advance();
                return 0;

            default:   // Done
                return 0;
        }
    }

    /// <summary>
    /// 到位判据：① 误差在容差内；② 相邻两拍误差变号 = 本拍一步穿过了目标。
    /// 只用①时，若每拍位移 v·dt 大于容差会在目标两侧以 ±v 逐拍振荡（数字极限环），故加穿过判定。
    /// </summary>
    private bool Arrived(double err)
    {
        bool crossed = !double.IsNaN(_prevErr) && err != 0 && Math.Sign(err) != Math.Sign(_prevErr);
        return Math.Abs(err) <= _tolMm || crossed;
    }

    private void EnterBack()
    {
        _phase = Phase.Back;
        _prevErr = double.NaN;
    }

    private void EnterGap(double tSec)
    {
        _phase = Phase.Gap;
        _prevErr = double.NaN;
        _gapStartT = tSec;
    }

    /// <summary>一个脉冲(出→回→停留)结束：推进到下一个脉冲，走完 durations 计一次循环，够 cycles 次则结束。</summary>
    private void Advance()
    {
        _prevErr = double.NaN;
        _pulse++;
        if (_pulse >= _peaks.Length)
        {
            _pulse = 0;
            _cycle++;
            if (_cycle >= _cycles)
            {
                _phase = Phase.Done;
                return;
            }
        }
        _phase = Phase.Out;
    }

    public bool IsFinished(double tSec) => _phase == Phase.Done;
}
