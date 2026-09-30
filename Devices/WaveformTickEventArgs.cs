namespace MotorControlApp.Devices;

/// <summary>
/// 波形运行每拍上报：当前时间、目标线速度、积分/反馈位置（均为 mm 物理域）。
/// 供 UI 实时读数与画曲线。可能在后台控制线程触发。
/// </summary>
public sealed class WaveformTickEventArgs : EventArgs
{
    public WaveformTickEventArgs(double timeSec, double velMmS, double posMm)
    {
        TimeSec = timeSec;
        VelMmS = velMmS;
        PosMm = posMm;
    }

    public double TimeSec { get; }
    public double VelMmS { get; }
    public double PosMm { get; }
}
