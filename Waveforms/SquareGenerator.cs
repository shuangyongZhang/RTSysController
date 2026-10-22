namespace MotorControlApp.Waveforms;

/// <summary>方波序列的一个途经点：目标位置(mm)、逼近速度(mm/s)、到位后停留(s)。</summary>
public readonly record struct Waypoint(double PosMm, double SpeedMmS, double DwellSec);

/// <summary>
/// 方波转速控制：平台在若干设定位 (A,B,C,D...) 之间交替切换，每段以给定速度 vk 逼近、
/// 到位后停留 tk，整表循环 n 次。速度模式下用当前 Dpos 反馈判到达。
/// 接近途经点时按刹车距离提前降速，停稳后先快速爬行、再低速爬行进入该点容差并等实际停稳，
/// 停稳后才开始计时停留——避免全速冲到目标时靠控制器减速、停在目标之外（0 点途经点尤其明显）。
/// 停留/刹车/爬行都以运行期减速度为依据，校验器回喂自身积分位置，预测与真机同源。
/// </summary>
public sealed class SquareGenerator : IWaveformGenerator
{
    /// <summary>减速度未设（&lt;=0）时的兜底值，与 WaveformValidator / StartWaveform 的回退一致。</summary>
    private const double DefaultDecelMmS2 = 5000.0;

    /// <summary>实际速度低于该值即认为已停稳（mm/s）。</summary>
    private const double SettleSpeedMmS = 0.6;

    private readonly Waypoint[] _wps;
    private readonly int _cycles;
    private readonly double _planDecelMmS2;   // 刹车距离规划值，留 20% 余量
    private readonly double _finalTolMm;      // 到点终点容差

    private int _idx;
    private int _cycle;
    private bool _dwelling;
    private double _dwellStartT;
    private bool _done;
    private bool _braking;                  // 已进入最后刹车段（锁存，防止停前再次加速）

    // 实际速度估计：相邻两拍的位置差 / 时间差，用于预算刹车距离
    private double _prevPosMm;
    private double _prevT = double.NaN;
    private double _vEstMmS;
    private double _tickSec = 0.01;
    private double _lastCmdMmS;             // 上一拍下发的速度（控制器下一拍才执行到它）

    public SquareGenerator(IReadOnlyList<Waypoint> waypoints, int cycles,
        double arriveTolMm = 0.5, double decelMmS2 = 0)
    {
        ArgumentNullException.ThrowIfNull(waypoints);
        if (waypoints.Count < 2) throw new ArgumentException("方波至少需要 2 个途经点", nameof(waypoints));
        if (cycles < 1) throw new ArgumentOutOfRangeException(nameof(cycles), "循环次数必须 >= 1");

        _wps = waypoints.ToArray();
        _cycles = cycles;
        double decel = decelMmS2 > 0 ? decelMmS2 : DefaultDecelMmS2;
        _planDecelMmS2 = decel * 0.8;
        _finalTolMm = Math.Max(0.02, Math.Min(Math.Max(0.01, arriveTolMm), 0.1));
        Reset();
    }

    public string ModeName => "方波";

    public bool PositionFeedback => true;

    /// <summary>提前断速后直到停稳都算停稳段（含两段爬行），停稳进入停留即结束；曲线显示时折叠该段。</summary>
    public bool IsSettling => _braking && !_dwelling;

    /// <summary>由循环次数决定结束，无固定总时长。</summary>
    public double TotalDurationSeconds => double.PositiveInfinity;

    public void Reset()
    {
        _idx = 0;
        _cycle = 0;
        _dwelling = false;
        _dwellStartT = 0;
        _done = false;
        _braking = false;
        _prevT = double.NaN;
        _prevPosMm = 0;
        _vEstMmS = 0;
        _tickSec = 0.01;
        _lastCmdMmS = 0;
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
        if (_done) return 0;

        var wp = _wps[_idx];

        if (_dwelling)
        {
            if (tSec - _dwellStartT >= wp.DwellSec)
                Advance();
            return 0;
        }

        double err = wp.PosMm - curPosMm;                // 目标：当前途经点
        double ad = Math.Abs(err);
        int dir = Math.Sign(err);
        double v = Math.Min(Math.Abs(wp.SpeedMmS), MotionUnits.VmaxMmS);
        double creep = Math.Min(v, Math.Clamp(v * 0.04, 2.0, 5.0));
        // 控制周期较大时，一拍爬行位移 creep·dt 可能超过终点容差：±creep 会在目标两侧
        // 逐拍来回、永远进不了容差（20ms 周期即触发）。此时把爬行速度压到半容差/拍以内；
        // 小周期下 creep·dt 本就小于容差，速度与原来一致。
        if (creep * _tickSec > _finalTolMm)
            creep = _finalTolMm / (2 * _tickSec);
        double fastCreep = Math.Min(v, Math.Max(creep, Math.Clamp(v * 0.1, 5.0, 20.0)));

        if (!_braking)
        {
            // 指令下一拍才生效、位置逐拍离散采样：刹车阈值再留两拍行程（同脉冲回程）。
            // 本段目标速度 v 也计入：从静止起步的短距离段若先按全速起一拍，再刹也回不来。
            double vLead = Math.Max(v, Math.Max(Math.Abs(_vEstMmS), Math.Abs(_lastCmdMmS)));
            double stopDist = 2 * vLead * _tickSec + vLead * vLead / (2 * _planDecelMmS2);
            if (ad <= stopDist + _finalTolMm)
                _braking = true;                  // 锁存：从此只减速/爬行，不再全速冲点
            else
                return dir * v;                   // 离目标还远，按该点速度逼近
        }

        // 爬行停稳窗：发 0 到真正停住还要走一拍指令 + v²/(2a)，窗口按这段刹车距离预留。
        double slowCapture = _finalTolMm * 0.5 + creep * _tickSec
            + creep * creep / (2 * _planDecelMmS2);
        if (ad <= slowCapture)
        {
            if (ad <= _finalTolMm)
            {
                // 已进容差：发 0 等控制器停稳，速度归零才开始计时停留
                if (Math.Abs(_vEstMmS) <= SettleSpeedMmS)
                {
                    _dwelling = true;
                    _dwellStartT = tSec;
                }
                return 0;
            }
            // 在停稳窗内但还没进容差：已经停住就用慢速补一点，还在动就先发 0 等停稳
            if (Math.Abs(_vEstMmS) <= SettleSpeedMmS) return dir * creep;
            return 0;
        }

        // 刹车段：速度未降到快速爬行档前只发 0（控制器按 DEC 停），
        // 停车后先快速爬行到零窗附近，再换低速爬行。
        double fastCapture = _finalTolMm + fastCreep * _tickSec * 2
            + fastCreep * fastCreep / (2 * _planDecelMmS2);
        if (ad > fastCapture)
        {
            if (Math.Abs(_vEstMmS) <= fastCreep) return dir * fastCreep;
            return 0;
        }
        if (Math.Abs(_vEstMmS) > creep) return 0;
        return dir * creep;
    }

    /// <summary>用相邻两拍的位置差估计实际速度（一拍轻滤波抑制位置量化噪声）。</summary>
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

    private void Advance()
    {
        _dwelling = false;
        _braking = false;
        _idx++;
        if (_idx >= _wps.Length)
        {
            _idx = 0;
            _cycle++;
            if (_cycle >= _cycles)
                _done = true;
        }
    }

    public bool IsFinished(double tSec) => _done;
}
