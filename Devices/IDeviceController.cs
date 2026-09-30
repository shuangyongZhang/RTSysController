namespace MotorControlApp.Devices;

using MotorControlApp.Waveforms;

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

    // ==================== 找零点（回零）====================

    /// <summary>是否正在执行找零（回零）运动。</summary>
    bool IsHoming { get; }

    /// <summary>
    /// 找零点（双硬限位对中）：无原点开关时，向正/负硬限位各撞一次取两端触发位，中点作为机械零点并置 Dpos=0。
    /// 以机械限位为绝对基准，可抵消撞限位后打滑/丢步造成的零点漂移。
    /// 阻塞直到完成或超时——上层必须放到后台线程调用。返回实测的限位/中点 mm 值（模拟器返回 null）。
    /// </summary>
    HomingResult? Home();

    // ==================== 运动波形（五种可复现轨迹） ====================

    /// <summary>波形是否正在运行。</summary>
    bool IsWaveformRunning { get; }

    /// <summary>波形每拍上报（时间/速度/位置）。</summary>
    event EventHandler<WaveformTickEventArgs>? WaveformTick;

    /// <summary>波形停止时上报停止原因（正常完成 / 用户停止 / 触碰软限位），供 UI 复位“启动波形”按钮与提示。可能在后台线程触发。</summary>
    event EventHandler<WaveformStopReason>? WaveformStopped;

    /// <summary>启动波形轨迹：以 CSV 速度模式逐拍下发生成器产生的带符号速度。调用前须已通过时域校验。</summary>
    void StartWaveform(WaveformRuntime runtime);

    /// <summary>停止波形轨迹（减速停止）。</summary>
    void StopWaveform();
}

/// <summary>
/// 双硬限位对中找零的实测结果。三个值均为“相对找零前旧零点”的 mm（因此 CenterMm 直接反映旧零点漂移量）。
/// 找零完成后该中点已被置为新零点（Dpos=0）。
/// </summary>
/// <param name="NegLimitMm">负限位触发点（相对旧零点，典型为负值）。</param>
/// <param name="PosLimitMm">正限位触发点（相对旧零点，典型为正值）。</param>
/// <param name="CenterMm">两端中点（=新零点）相对旧零点的偏移，即此次纠正掉的漂移量。</param>
public sealed record HomingResult(float NegLimitMm, float PosLimitMm, float CenterMm)
{
    /// <summary>全行程（mm）= 正限位 - 负限位。</summary>
    public float FullTravelMm => PosLimitMm - NegLimitMm;

    /// <summary>半行程（mm）。</summary>
    public float HalfTravelMm => (PosLimitMm - NegLimitMm) / 2f;

    /// <summary>限位不对称度（mm）：中点到两端距离之差，越接近 0 说明两端限位越对称。</summary>
    public float AsymmetryMm => (PosLimitMm - CenterMm) - (CenterMm - NegLimitMm);
}

/// <summary>
/// 轴状态快照（界面状态灯轮询用）：使能 / 正负软限位 / 正负硬限位 / 伺服报警。
/// 硬限位取 AXISSTATUS 实时位（bit4 正 0x10、bit5 负 0x20）；软限位取 bit9(0x200)/bit10(0x400) 并按 Dpos 对比限位值兜底。
/// </summary>
public sealed record AxisStatusInfo(
    bool Enabled,
    bool PosSoftLimit,
    bool NegSoftLimit,
    bool PosHardLimit,
    bool NegHardLimit,
    bool ServoAlarm);

/// <summary>波形停止原因，供 UI 区分复位与提示。</summary>
public enum WaveformStopReason
{
    /// <summary>波形按计划跑完（正常结束）。</summary>
    Completed,
    /// <summary>用户点击停止。</summary>
    UserStopped,
    /// <summary>开启软限位保护时触碰软限位而中止。</summary>
    SoftLimit,
}