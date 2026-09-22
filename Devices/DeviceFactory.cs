using MotorControlApp.Configuration;

namespace MotorControlApp.Devices;

/// <summary>
/// 根据 config.json 中 UseSimulator 决定：模拟器 / 正运动 ZMotion 控制卡。
/// </summary>
public static class DeviceFactory
{
    public static IDeviceController Create(AppConfig config)
    {
        if (config.Connection.UseSimulator.Value)
            return new SimulatedDeviceController(config.Sensor.UpdateIntervalMs.Value);

        return new ZMotionDeviceController(config);
    }
}
