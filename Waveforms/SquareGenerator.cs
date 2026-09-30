namespace MotorControlApp.Waveforms;

/// <summary>方波序列的一个途经点：目标位置(mm)、逼近速度(mm/s)、到位后停留(s)。</summary>
public readonly record struct Waypoint(double PosMm, double SpeedMmS, double DwellSec);

/// <summary>
/// 方波转速控制：平台在若干设定位 (A,B,C,D...) 之间交替切换，每段以给定速度 vk 逼近、
/// 到位后停留 tk，整表循环 n 次。速度模式下用当前 Dpos 反馈判到达（|curPos-P|&lt;tol 后转停留）。
/// </summary>
public sealed class SquareGenerator : IWaveformGenerator
{
    private readonly Waypoint[] _wps;
    private readonly int _cycles;
    private readonly double _tolMm;

    private int _idx;
    private int _cycle;
    private bool _dwelling;
    private double _dwellStartT;
    private double _prevErr = double.NaN;   // 上一拍误差，用于“穿过目标”判定
    private bool _done;

    public SquareGenerator(IReadOnlyList<Waypoint> waypoints, int cycles, double arriveTolMm = 0.5)
    {
        ArgumentNullException.ThrowIfNull(waypoints);
        if (waypoints.Count < 2) throw new ArgumentException("方波至少需要 2 个途经点", nameof(waypoints));
        if (cycles < 1) throw new ArgumentOutOfRangeException(nameof(cycles), "循环次数必须 >= 1");

        _wps = waypoints.ToArray();
        _cycles = cycles;
        _tolMm = Math.Max(0.01, arriveTolMm);
        Reset();
    }

    public string ModeName => "方波";

    /// <summary>由循环次数决定结束，无固定总时长。</summary>
    public double TotalDurationSeconds => double.PositiveInfinity;

    public void Reset()
    {
        _idx = 0;
        _cycle = 0;
        _dwelling = false;
        _dwellStartT = 0;
        _prevErr = double.NaN;
        _done = false;
    }

    public double NextVelocity(double tSec, double curPosMm)
    {
        if (_done) return 0;

        var wp = _wps[_idx];

        if (_dwelling)
        {
            if (tSec - _dwellStartT >= wp.DwellSec)
                Advance();
            return 0;
        }

        double err = wp.PosMm - curPosMm;

        // 到位判据：① 误差在容差内；② 相邻两拍误差变号 = 本拍一步穿过了目标。
        // 若只用①，当每拍位移 v·dt 大于容差时（如 200mm/s×10ms=2mm > 0.5mm）会在目标
        // 两侧以 ±v 逐拍振荡（数字极限环），永远到不了下一点，故必须加穿过判定。
        bool crossed = !double.IsNaN(_prevErr) && err != 0 && Math.Sign(err) != Math.Sign(_prevErr);
        if (Math.Abs(err) <= _tolMm || crossed)
        {
            _dwelling = true;
            _dwellStartT = tSec;
            _prevErr = double.NaN;
            return 0;
        }
        _prevErr = err;

        double dir = Math.Sign(err);
        return dir * Math.Min(Math.Abs(wp.SpeedMmS), MotionUnits.VmaxMmS);
    }

    private void Advance()
    {
        _dwelling = false;
        _prevErr = double.NaN;
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
