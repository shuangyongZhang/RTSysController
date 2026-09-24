namespace MotorControlApp.Devices;

/// <summary>
/// 硬件设备抽象层。UI 只依赖此接口，不关心底层是串口、TCP 还是模拟器。
/// 拿到真实硬件协议后，新增一个实现类（如 SerialDeviceController）即可，
/// 再把 config.json 里 Connection.UseSimulator 改为 false 完成切换。
/// </summary>
public interface IDeviceController : IDisposable
{
    /// <summary>当前是否已连接设备。</summary>
    bool IsConnected { get; }

    /// <summary>4 路传感器数据实时上报（注意：可能在后台线程触发）。</summary>
    event EventHandler<SensorDataEventArgs>? SensorDataReceived;


    /// <summary>按当前配置连接设备（连接参数均来自 config.json，无端口入参）。返回是否连接成功。</summary>
    Task<bool> ConnectAsync(CancellationToken cancellationToken = default);

    /// <summary>主动断开连接。</summary>
    void Disconnect();

    /// <summary>电机前进。</summary>
    void Forward();

    /// <summary>电机后退。</summary>
    void Backward();

    /// <summary>关闭/停止电机。</summary>
    void Stop();
}