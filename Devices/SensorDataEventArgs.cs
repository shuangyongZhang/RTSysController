namespace MotorControlApp.Devices;

/// <summary>
/// 传感器数据上报事件参数，Values 固定为 4 路传感器的值。
/// </summary>
public sealed class SensorDataEventArgs : EventArgs
{
    public IReadOnlyList<double> Values { get; }

    public SensorDataEventArgs(IReadOnlyList<double> values)
    {
        // 拷贝一份，避免外部持有后台线程正在修改的数组
        Values = values.ToArray();
    }
}
