namespace MotorControlApp.Configuration;

/// <summary>
/// 对应外部 config.json 的根对象，新增可调参数时在此扩展即可。
/// </summary>
public sealed class AppConfig
{
    public ConnectionConfig Connection { get; set; } = new();

    public ZMotionConfig ZMotion { get; set; } = new();

    public SensorConfig Sensor { get; set; } = new();

    public UiConfig Ui { get; set; } = new();
}

/// <summary>连接参数。</summary>
public sealed class ConnectionConfig
{
    /// <summary>true=模拟器；false=正运动控制卡。</summary>
    public CfgValue<bool> UseSimulator { get; set; } = new() { Value = true };

    /// <summary>连接类型：LOCAL / Ethernet / PCI / Serial。</summary>
    public CfgValue<string> Type { get; set; } = new() { Value = "LOCAL" };

    /// <summary>连接目标：LOCAL1 / 192.168.0.11 / PCI1 / COM3。</summary>
    public CfgValue<string> Target { get; set; } = new() { Value = "LOCAL1" };

    /// <summary>连接超时（毫秒）。</summary>
    public CfgValue<int> TimeoutMs { get; set; } = new() { Value = 3000 };
}

/// <summary>正运动控制卡参数（ZMotion / zauxdll.dll）。</summary>
public sealed class ZMotionConfig
{
    /// <summary>轴号。</summary>
    public CfgValue<int> AxisNumber { get; set; } = new() { Value = 0 };

    /// <summary>脉冲当量：1 脉冲 = 多少物理单位（mm/脉冲），SetUnits 参数。</summary>
    public CfgValue<float> Units { get; set; } = new() { Value = 1.0f };

    /// <summary>最低速度（mm/s），SetLspeed 参数。</summary>
    public CfgValue<float> Lspeed { get; set; } = new() { Value = 10.0f };

    /// <summary>运行速度（mm/s），SetSpeed 参数。</summary>
    public CfgValue<float> Speed { get; set; } = new() { Value = 100.0f };

    /// <summary>加速度（mm/s²），SetAccel 参数。</summary>
    public CfgValue<float> Accel { get; set; } = new() { Value = 500.0f };

    /// <summary>减速度（mm/s²），SetDecel 参数。</summary>
    public CfgValue<float> Decel { get; set; } = new() { Value = 500.0f };

    /// <summary>S 曲线时间（ms），SetSramp 参数；填 0 关闭 S 曲线。</summary>
    public CfgValue<float> Sramp { get; set; } = new() { Value = 0.0f };

    /// <summary>正向软限位坐标（物理单位 mm），SetPosLimit 参数。default +500000（相当于不生效）。</summary>
    public CfgValue<float> SoftLimitPos { get; set; } = new() { Value = 500000.0f };

    /// <summary>负向软限位坐标（物理单位 mm），SetNegPosLimit 参数。default -500000（相当于不生效）。</summary>
    public CfgValue<float> SoftLimitNeg { get; set; } = new() { Value = -500000.0f };

    /// <summary>
    /// 轴类型 ATYPE。
    /// 1 = 本地脉冲/步进轴；
    /// 65 = EtherCAT 伺服位置模式 CSP；
    /// 66 = EtherCAT 伺服速度模式 CSV；
    /// 67 = EtherCAT 伺服力矩模式 CST。
    /// EtherCAT 总线驱动器必须设 65/66/67，不能用 1！
    /// </summary>
    public CfgValue<int> AtType { get; set; } = new() { Value = 65 };

    // ---- 便捷取值 ----
    public int GetAxisNumber() => AxisNumber.Value;
    public int GetAtType() => AtType.Value;
    public float GetUnits() => Units.Value;
    public float GetLspeed() => Lspeed.Value;
    public float GetSpeed() => Speed.Value;
    public float GetAccel() => Accel.Value;
    public float GetDecel() => Decel.Value;
    public float GetSramp() => Sramp.Value;
    public float GetSoftLimitPos() => SoftLimitPos.Value;
    public float GetSoftLimitNeg() => SoftLimitNeg.Value;
}

/// <summary>传感器参数。</summary>
public sealed class SensorConfig
{
    /// <summary>轮询间隔（毫秒）。</summary>
    public CfgValue<int> UpdateIntervalMs { get; set; } = new() { Value = 200 };

    /// <summary>
    /// 全局 AIN 起始编号。
    /// PAC 本体一般从 0 开始；EtherCAT 总线模块如果配了 NODE_AIO，
    /// 起始编号会排在本体之后或按 RTSys 配置分配。
    /// 在诊断里用 NODE_AIO 检查结果校准这个值。
    /// </summary>
    public CfgValue<int> AinStart { get; set; } = new() { Value = 0 };

    /// <summary>实际采集几路 AIN（对应 UI 上显示的通道数）。</summary>
    public CfgValue<int> AinCount { get; set; } = new() { Value = 4 };

    /// <summary>换算系数：1 ADC ≈ 多少克（传感器灵敏度）。默认 15.26 g/ADC。</summary>
    public CfgValue<double> AdcPerGram { get; set; } = new() { Value = 15.26 };

    /// <summary>重力加速度 g（m/s²），用于 kg→N 换算。默认 10。</summary>
    public CfgValue<double> Gravity { get; set; } = new() { Value = 10.0 };

    /// <summary>K。</summary>
    public CfgValue<double> K { get; set; } = new() { Value = 50.0 };

    /// <summary>
    /// 4 路传感器归零校准值（raw 原始码）。
    /// 点击"归零校准"时记录当前 4 路 raw，显示时减去该值。
    /// 持久化到 config.json，下次启动自动加载。
    /// </summary>
    public CfgValue<int[]> ZeroOffsets { get; set; } = new() { Value = new int[4] };

    public int GetUpdateIntervalMs() => UpdateIntervalMs.Value;
    public int GetAinStart() => AinStart.Value;
    public int GetAinCount() => AinCount.Value;
    public double GetAdcPerGram() => AdcPerGram.Value;
    public double GetGravity() => Gravity.Value;
    public double GetK() => K.Value;

    /// <summary>读取 4 路归零偏移（长度不足时自动补齐为 4）。</summary>
    public int[] GetZeroOffsets()
    {
        int[] src = ZeroOffsets.Value ?? Array.Empty<int>();
        var dst = new int[4];
        Array.Copy(src, dst, Math.Min(src.Length, 4));
        return dst;
    }

    /// <summary>写回 4 路归零偏移到 config（不自动落盘，由调用方 TrySave）。</summary>
    public void SetZeroOffsets(int[] offsets)
    {
        var dst = new int[4];
        if (offsets != null) Array.Copy(offsets, dst, Math.Min(offsets.Length, 4));
        ZeroOffsets.Value = dst;
    }
}

public sealed class UiConfig
{
    public CfgValue<double> TipDisplaySeconds { get; set; } = new() { Value = 3.0 };
    public double GetTipDisplaySeconds() => TipDisplaySeconds.Value;
}
