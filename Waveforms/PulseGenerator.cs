namespace MotorControlApp.Waveforms;

/// <summary>
/// 脉冲转速控制（闭环·位置反馈）：从基位(0)产生一次短时冲到峰值再退回基位。
/// 每个脉冲宽度 ti 对应名义峰值行程 Pk = v1·ti/2；用当前 Dpos 反馈判到达：
/// 先朝 +Pk 冲出（|curPos-Pk|&lt;tol 或本拍穿过即转回），再朝基位 0 退回。
/// 回程按“刹车距离”预留：距 0 还远时全速返回，剩余距离进入刹车区后锁存减速，
/// 停稳后先快速爬行、再低速爬行进入零窗并等实际停稳，避免全速冲到 0 时靠控制器减速、停在 0 以下。
/// 脉冲间 gap 零速停留。durations 列表按顺序循环 n 次。
///
/// 位置判到达与刹车距离规划都以运行期减速度 a 为依据；校验器/采样器把自身积分位置回喂给
/// NextVelocity，因此“启动前预估”与“真机实时曲线”走同一套闭环逻辑，二者一致。
/// 注：真机在峰值处仍有一个由加速度决定的有限过冲（≈v²/2a，不累积），贴近软限位时应留一点余量。
/// </summary>
public sealed class PulseGenerator : IWaveformGenerator
{
    private enum Phase { Out, Back, Gap, Done }

    /// <summary>减速度未设（&lt;=0）时的兜底值，与 WaveformValidator / StartWaveform 的回退一致。</summary>
    private const double DefaultDecelMmS2 = 5000.0;

    /// <summary>实际速度低于该值即认为已停稳（mm/s）。</summary>
    private const double SettleSpeedMmS = 0.6;

    private readonly double _v;
    private readonly double[] _peaks;   // 每个脉冲的名义峰值行程(mm)
    private readonly int _cycles;
    private readonly double _gapSec;
    private readonly double _tolMm;
    private readonly double _planDecelMmS2;   // 刹车距离规划值，留 20% 余量
    private readonly double _creepMmS;         // 零窗前的低速爬行速度
    private readonly double _fastCreepMmS;     // 距零较远时的快速爬行速度
    private readonly double _finalTolMm;       // 回零终点容差

    private int _pulse;
    private int _cycle;
    private Phase _phase;
    private double _gapStartT;
    private double _prevErr = double.NaN;   // 上一拍误差，用于“穿过目标”判定
    private bool _braking;                  // 回程已进入最后刹车段（锁存，防止停前再次加速）

    // 实际速度估计：相邻两拍的位置差 / 时间差，用于预算刹车距离
    private double _prevPosMm;
    private double _prevT = double.NaN;
    private double _vEstMmS;
    private double _tickSec = 0.01;
    private double _lastCmdMmS;             // 上一拍下发的速度（控制器下一拍才执行到它）

    public PulseGenerator(double speedMmS, IReadOnlyList<double> durationsSec, int cycles,
        double gapSec = 0, double arriveTolMm = 0.5, double decelMmS2 = 0)
    {
        ArgumentNullException.ThrowIfNull(durationsSec);
        if (speedMmS <= 0) throw new ArgumentOutOfRangeException(nameof(speedMmS), "脉冲速度必须 > 0");
        if (cycles < 1) throw new ArgumentOutOfRangeException(nameof(cycles), "循环次数必须 >= 1");
        if (durationsSec.Any(d => d <= 0)) throw new ArgumentException("脉冲宽度必须 > 0", nameof(durationsSec));

        _v = Math.Min(speedMmS, MotionUnits.VmaxMmS);
        _cycles = cycles;
        _gapSec = Math.Max(0, gapSec);
        _tolMm = Math.Max(0.01, arriveTolMm);
        double decel = decelMmS2 > 0 ? decelMmS2 : DefaultDecelMmS2;
        _planDecelMmS2 = decel * 0.8;
        _creepMmS = Math.Min(_v, Math.Clamp(_v * 0.04, 2.0, 5.0));
        _fastCreepMmS = Math.Min(_v, Math.Max(_creepMmS, Math.Clamp(_v * 0.1, 5.0, 20.0)));
        _finalTolMm = Math.Max(0.02, Math.Min(_tolMm, 0.1));
        // 名义峰值行程 v·ti/2（与旧纯时间驱动的单脉冲峰值一致）
        _peaks = durationsSec.Select(ti => _v * ti / 2.0).ToArray();
        Reset();
    }

    public string ModeName => "脉冲";

    public bool PositionFeedback => true;

    /// <summary>回程提前断速后直到停稳都算停稳段（含两段爬行）；曲线显示时折叠该段。</summary>
    public bool IsSettling => _phase == Phase.Back && _braking;

    /// <summary>由循环次数决定结束，无固定总时长（同方波）。</summary>
    public double TotalDurationSeconds => double.PositiveInfinity;

    public void Reset()
    {
        _pulse = 0;
        _cycle = 0;
        _phase = Phase.Out;
        _gapStartT = 0;
        _prevErr = double.NaN;
        _prevT = double.NaN;
        _prevPosMm = 0;
        _vEstMmS = 0;
        _tickSec = 0.01;
        _lastCmdMmS = 0;
        _braking = false;
    }

    public double NextVelocity(double tSec, double curPosMm)
    {
        UpdateMotionEstimate(tSec, curPosMm);
        double cmd = ComputeVelocity(tSec, curPosMm);
        _lastCmdMmS = cmd;
        return cmd;
    }

    private double ComputeVelocity(double tSec, double curPosMm)
    {
        switch (_phase)
        {
            case Phase.Out:
            {
                double err = _peaks[_pulse] - curPosMm;   // 目标：名义峰值
                if (Arrived(err, TolForPulse(_pulse))) { EnterBack(); return 0; }
                _prevErr = err;
                return Math.Sign(err) * _v;               // 朝峰值冲出
            }

            case Phase.Back:
            {
                double err = 0 - curPosMm;                // 目标：基位 0
                double ad = Math.Abs(err);
                int dir = Math.Sign(err);
                double creep = CreepForTick();

                if (!_braking)
                {
                    // 本次指令下一拍才生效，下一拍仍执行上一拍指令；再下一拍才开始按 DEC 减速。
                    // 同时位置是逐拍离散采样的，阈值要再留一拍行程，避免“这次刚好差一点没进刹车区、
                    // 下一拍已经冲过头”的相位问题。故预留：2·v·dt + v²/(2a)。
                    double vLead = Math.Max(Math.Abs(_vEstMmS), Math.Abs(_lastCmdMmS));
                    double stopDist = 2 * vLead * _tickSec + vLead * vLead / (2 * _planDecelMmS2);
                    if (ad <= stopDist + _finalTolMm)
                        _braking = true;                  // 锁存：从此只减速/爬行，不再全速回冲
                    else
                        return dir * _v;                  // 离 0 还远，全速回程
                }

                // 爬行停稳窗：从低速指令发 0 到真正停住还要走 一拍指令 + v²/(2a)，
                // 窗口要按这段刹车距离预留，否则低速/低减速度下会在 0 两侧来回爬。
                double slowCapture = _finalTolMm * 0.5 + creep * _tickSec
                    + creep * creep / (2 * _planDecelMmS2);
                if (ad <= slowCapture)
                {
                    if (ad <= _finalTolMm)
                    {
                        // 已进容差：发 0 等控制器停稳，速度归零才算到达
                        if (Math.Abs(_vEstMmS) <= SettleSpeedMmS) EnterGap(tSec);
                        return 0;
                    }
                    // 在停稳窗内但还没进容差：已经停住就用慢速补一点，还在动就先发 0 等停稳
                    if (Math.Abs(_vEstMmS) <= SettleSpeedMmS) return dir * creep;
                    return 0;
                }

                // 刹车段：速度未降到快速爬行档前只发 0（控制器按 DEC 停），
                // 停车后先快速爬行到零窗附近，再换低速爬行，避免高速模式下长时间慢慢蹭回 0。
                double fastCapture = _finalTolMm + _fastCreepMmS * _tickSec * 2
                    + _fastCreepMmS * _fastCreepMmS / (2 * _planDecelMmS2);
                if (ad > fastCapture)
                {
                    if (Math.Abs(_vEstMmS) <= _fastCreepMmS) return dir * _fastCreepMmS;
                    return 0;
                }
                if (Math.Abs(_vEstMmS) > creep) return 0;
                return dir * creep;
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
    /// 回零爬行速度：一拍爬行位移超过回零容差时（控制周期较大，20ms 周期即触发），
    /// ±creep 会在 0 两侧逐拍来回、永远进不了容差；此时压到半容差/拍以内。
    /// 小周期下不超过容差，保持原速度不变。
    /// </summary>
    private double CreepForTick() =>
        _creepMmS * _tickSec > _finalTolMm ? _finalTolMm / (2 * _tickSec) : _creepMmS;

    /// <summary>
    /// 用相邻两拍的位置差估计实际速度（一拍轻滤波抑制位置量化噪声）。
    /// 回程刹车距离与“是否停稳”都靠它判断；时间差同时对拍长做估计，供离散刹车限速使用。
    /// </summary>
    private void UpdateMotionEstimate(double tSec, double curPosMm)
    {
        if (!double.IsNaN(_prevT))
        {
            double dt = tSec - _prevT;
            if (dt > 1e-4)
            {
                _tickSec = Math.Clamp(dt, 0.001, 0.1);
                double raw = (curPosMm - _prevPosMm) / dt;
                _vEstMmS += 0.6 * (raw - _vEstMmS);
            }
        }
        _prevT = tSec;
        _prevPosMm = curPosMm;
    }

    /// <summary>
    /// 到位判据：① 误差在容差内；② 相邻两拍误差变号 = 本拍一步穿过了目标。
    /// 只用①时，若每拍位移 v·dt 大于容差会在目标两侧以 ±v 逐拍振荡（数字极限环），故加穿过判定。
    /// </summary>
    private bool Arrived(double err, double tol)
    {
        bool crossed = !double.IsNaN(_prevErr) && err != 0 && Math.Sign(err) != Math.Sign(_prevErr);
        return Math.Abs(err) <= tol || crossed;
    }

    /// <summary>
    /// 每个脉冲的有效到位容差：名义峰值小于默认容差时按峰值等比缩小。
    /// 否则极窄脉冲（Pk &lt; 0.5mm）在起点就满足 |Pk-0| ≤ tol，首拍即判到达 → 整个脉冲零运动。
    /// Pk 正常时仍用默认容差，行为不变。
    /// </summary>
    private double TolForPulse(int pulse) => _peaks[pulse] < _tolMm ? _peaks[pulse] * 0.25 : _tolMm;

    private void EnterBack()
    {
        _phase = Phase.Back;
        _prevErr = double.NaN;
        _braking = false;
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
        _braking = false;
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
