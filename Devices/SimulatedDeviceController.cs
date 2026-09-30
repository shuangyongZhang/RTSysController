namespace MotorControlApp.Devices;

using MotorControlApp.Waveforms;

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
    private volatile bool _homing;

    // ---- 波形模拟：积分速度→假位置，验证曲线与校验逻辑 ----
    private CancellationTokenSource? _wfCts;
    private volatile bool _wfRunning;
    private double _simPosMm;

    public SimulatedDeviceController(int updateIntervalMs)
    {
        _updateIntervalMs = Math.Max(5, updateIntervalMs);
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

    public bool IsWaveformRunning => _wfRunning;

    public event EventHandler<WaveformTickEventArgs>? WaveformTick;

    public event EventHandler<WaveformStopReason>? WaveformStopped;

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
        StopWaveform();
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

    public bool IsHoming => _homing;

    /// <summary>模拟双限位对中找零：模拟各阶段耗时后归零；无真实限位，返回 null。</summary>
    public HomingResult? Home()
    {
        EnsureConnected();
        if (_wfRunning) throw new InvalidOperationException("波形运行中，请先停止波形再找零");
        if (_homing) throw new InvalidOperationException("找零正在进行中，请稍候");
        _homing = true;
        try
        {
            Thread.Sleep(500);      // 模拟撞负限位 + 退出
            Thread.Sleep(500);      // 模拟撞正限位 + 退出
            Thread.Sleep(300);      // 模拟回到中点
            _motorState = 0;
            _simPosMm = 0;          // 中点即零点
            return null;            // 模拟器无实测限位值
        }
        finally
        {
            _homing = false;
        }
    }

    public void Dispose()
    {
        _cts?.Cancel();
        _cts?.Dispose();
        _cts = null;
        StopWaveform();
        GC.SuppressFinalize(this);
    }

    private void EnsureConnected()
    {
        if (!_connected)
        {
            throw new InvalidOperationException("设备未连接");
        }
    }

    // ==================== 运动波形（模拟）====================

    public void StartWaveform(WaveformRuntime runtime)
    {
        EnsureConnected();
        if (_wfRunning) throw new InvalidOperationException("波形已在运行中");
        ArgumentNullException.ThrowIfNull(runtime);

        runtime.Generator.Reset();
        _simPosMm = 0;
        _wfCts = new CancellationTokenSource();
        _wfRunning = true;
        _ = Task.Run(() => WaveformLoopAsync(runtime, _wfCts.Token));
    }

    private async Task WaveformLoopAsync(WaveformRuntime rt, CancellationToken ct)
    {
        var gen = rt.Generator;
        var sw = System.Diagnostics.Stopwatch.StartNew();
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(rt.DtSec));
        WaveformStopReason reason = WaveformStopReason.Completed;
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await timer.WaitForNextTickAsync(ct).ConfigureAwait(false);
                double t = sw.Elapsed.TotalSeconds;

                double v = gen.NextVelocity(t, _simPosMm);
                _simPosMm += v * rt.DtSec;             // 积分生成假位置

                WaveformTick?.Invoke(this, new WaveformTickEventArgs(t, v, _simPosMm));

                if (gen.IsFinished(t)) break;
            }
        }
        catch (OperationCanceledException) { reason = WaveformStopReason.UserStopped; }
        finally
        {
            _wfRunning = false;
            WaveformStopped?.Invoke(this, reason);
        }
    }

    public void StopWaveform()
    {
        _wfCts?.Cancel();
        _wfCts?.Dispose();
        _wfCts = null;
        _wfRunning = false;
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
