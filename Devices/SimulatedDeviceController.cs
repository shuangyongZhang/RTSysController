namespace MotorControlApp.Devices;

/// <summary>
/// 模拟设备实现：硬件接口未到位前用于联调 UI 与业务逻辑。
/// - 后台周期产生 4 路 0~100 的模拟传感器数据；电机正反转会让数据产生趋势变化
/// TODO（拿到真实协议后）：新建 SerialDeviceController，按协议组帧/解析，替换本类。
/// </summary>
public sealed class SimulatedDeviceController : IDeviceController
{
    private readonly int _updateIntervalMs;
    private readonly Random _random = new();
    private readonly double[] _values = new double[4];

    private CancellationTokenSource? _cts;
    private volatile int _motorState; // 0=停止 1=前进 -1=后退
    private volatile bool _connected;

    public SimulatedDeviceController(int updateIntervalMs)
    {
        _updateIntervalMs = Math.Max(50, updateIntervalMs);
        for (int i = 0; i < _values.Length; i++)
        {
            _values[i] = 50.0;
        }
    }

    public bool IsConnected => _connected;

    /// <summary>模拟在线从站数量：3 个（电机伺服 + AD 模块 + 一个备用）。</summary>
    public int SlaveCount => 3;

    public event EventHandler<SensorDataEventArgs>? SensorDataReceived;

    public event EventHandler? Disconnected;

    public Task<bool> ConnectAsync(CancellationToken cancellationToken = default)
    {
        // 模拟连接耗时
        Thread.Sleep(100);
        cancellationToken.ThrowIfCancellationRequested();

        _connected = true;
        _motorState = 0;
        _cts = new CancellationTokenSource();
        _ = Task.Run(() => SensorLoopAsync(_cts.Token));

        return Task.FromResult(true);
    }

    public void Disconnect()
    {
        if (!_connected)
        {
            return;
        }

        _connected = false;
        _motorState = 0;
        _cts?.Cancel();
        Disconnected?.Invoke(this, EventArgs.Empty);
    }

    public void Forward()
    {
        EnsureConnected();
        _motorState = 1;
    }

    public void Backward()
    {
        EnsureConnected();
        _motorState = -1;
    }

    public void Stop()
    {
        EnsureConnected();
        _motorState = 0;
    }

    public void Dispose()
    {
        _cts?.Cancel();
        _cts?.Dispose();
        _cts = null;
        GC.SuppressFinalize(this);
    }

    private void EnsureConnected()
    {
        if (!_connected)
        {
            throw new InvalidOperationException("设备未连接");
        }
    }

    private async Task SensorLoopAsync(CancellationToken token)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(_updateIntervalMs));

        try
        {
            while (await timer.WaitForNextTickAsync(token).ConfigureAwait(false))
            {
                for (int i = 0; i < _values.Length; i++)
                {
                    // 电机状态给模拟值增加一点趋势，便于肉眼区分正/反/停
                    double bias = _motorState * (i % 2 == 0 ? 0.25 : -0.2);
                    _values[i] += _random.NextDouble() * 4.0 - 2.0 + bias;
                    _values[i] = Math.Clamp(_values[i], 0.0, 100.0);
                }

                SensorDataReceived?.Invoke(this, new SensorDataEventArgs(_values));
            }
        }
        catch (OperationCanceledException)
        {
            // 正常停止
        }
    }
}
