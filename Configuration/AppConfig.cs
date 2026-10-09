namespace MotorControlApp.Configuration;

/// <summary>
/// 对应外部 config.json 的根对象，新增可调参数时在此扩展即可。
/// </summary>
public sealed class AppConfig
{
    public ConnectionConfig Connection { get; set; } = new();

    public ZMotionConfig ZMotion { get; set; } = new();

    public SensorConfig Sensor { get; set; } = new();

    public WaveformConfig Waveform { get; set; } = new();

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

    /// <summary>脉冲当量：每 1 用户单位 = 多少脉冲（SetUnits 参数），=1 即 1 units = 1 pulse。</summary>
    public CfgValue<float> Units { get; set; } = new() { Value = 1.0f };

    /// <summary>丝杆导程（cm，标定常量，UI 不直接编辑）：电机转一圈平台走多少 cm。MotionUnits 内部×10 换算为 mm。</summary>
    public CfgValue<float> Lead { get; set; } = new() { Value = 28.2857f };

    /// <summary>齿轮比分母：电机转一圈 = 多少脉冲。</summary>
    public CfgValue<int> GearDenominator { get; set; } = new() { Value = 100000 };

    /// <summary>最低速度（内部存 user units；UI 以 mm/s 显示），SetLspeed 参数。</summary>
    public CfgValue<float> Lspeed { get; set; } = new() { Value = 10.0f };

    /// <summary>运行速度（内部存 user units；UI 以 mm/s 显示），SetSpeed 参数。</summary>
    public CfgValue<float> Speed { get; set; } = new() { Value = 100.0f };

    /// <summary>加速度（内部存 user units；UI 以 mm/s² 显示），SetAccel 参数。</summary>
    public CfgValue<float> Accel { get; set; } = new() { Value = 500.0f };

    /// <summary>减速度（内部存 user units；UI 以 mm/s² 显示），SetDecel 参数。</summary>
    public CfgValue<float> Decel { get; set; } = new() { Value = 500.0f };

    /// <summary>S 曲线时间（ms），SetSramp 参数；填 0 关闭 S 曲线。</summary>
    public CfgValue<float> Sramp { get; set; } = new() { Value = 0.0f };

    /// <summary>正向软限位（内部存 user units；UI 以 mm 显示），SetPosLimit 参数。default +500000（相当于不生效）。</summary>
    public CfgValue<float> SoftLimitPos { get; set; } = new() { Value = 500000.0f };

    /// <summary>负向软限位（内部存 user units；UI 以 mm 显示），SetNegPosLimit 参数。default -500000（相当于不生效）。</summary>
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

    /// <summary>
    /// 找零方式：双硬限位对中。因本设备无原点开关，改为“向负限位走→触发→退出→向正限位走→触发→退出→取两端中点=零点”。
    /// 机械限位是绝对基准，故能抵消硬限位撞碰后皮带打滑/丢步造成的零点漂移。
    /// 下列参数均以物理量（mm / mm·s⁻¹）表达，代码内部换算为 user units。
    /// </summary>
    /// <summary>找零低速（精确）搜寻速度（mm/s）。撞限位精确段与回中末段用它，越慢接触越轻柔、触发点越准。默认 20mm/s。</summary>
    public CfgValue<float> HomeSeekSpeedMmS { get; set; } = new() { Value = 20.0f };

    /// <summary>找零快速巡航速度（mm/s）：远离限位的空行程用它快跑，大幅缩短找零时间；接近限位/回中末段再降到 HomeSeekSpeedMmS 低速精确触发，兼顾速度与接触轻柔（不额外加剧皮带打滑）。默认 60。</summary>
    public CfgValue<float> HomeCruiseSpeedMmS { get; set; } = new() { Value = 60.0f };

    /// <summary>找零巡航加减速（mm/s²）。高速段需要更大加减速以缩短爬坡。默认 200。</summary>
    public CfgValue<float> HomeCruiseAccelMmS2 { get; set; } = new() { Value = 200.0f };

    /// <summary>回中末段切换为低速的提前距离（mm）：距目标小于此值改用 HomeSeekSpeedMmS 精确停靠。默认 5。</summary>
    public CfgValue<float> HomeCreepZoneMm { get; set; } = new() { Value = 5.0f };

    /// <summary>找零低速段加减速（mm/s²），低速小加减速降低冲击。默认 50。</summary>
    public CfgValue<float> HomeSeekAccelMmS2 { get; set; } = new() { Value = 50.0f };

    /// <summary>退出限位开关后再多退的余量（mm），脱离开关回差。默认 2mm。</summary>
    public CfgValue<float> HomeBackoffMm { get; set; } = new() { Value = 2.0f };

    /// <summary>回到中点的到位容差（mm）。默认 0.5mm。</summary>
    public CfgValue<float> HomeCenterTolMm { get; set; } = new() { Value = 0.5f };

    /// <summary>找零单阶段（撞限位/退出/回中）超时（毫秒），超时急停报错，防卡死。默认 30000。</summary>
    public CfgValue<int> HomePhaseTimeoutMs { get; set; } = new() { Value = 30000 };

    /// <summary>单向最大允许行程（mm）：朝一个方向走这么多仍未触发限位即判定“限位没接/未映射”并急停。应 > 半行程(约9cm)。默认 200。</summary>
    public CfgValue<float> HomeMaxTravelMm { get; set; } = new() { Value = 200.0f };

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

/// <summary>运动波形参数（五种可复现轨迹模式）。</summary>
public sealed class WaveformConfig
{
    /// <summary>模式：0=单正弦 1=多正弦叠加 2=方波 3=脉冲 4=伪随机PRTS。</summary>
    public CfgValue<int> Mode { get; set; } = new() { Value = 0 };

    /// <summary>控制回路周期 dt（毫秒），逐拍下发速度的时间步长。</summary>
    public CfgValue<int> ControlPeriodMs { get; set; } = new() { Value = 10 };

    /// <summary>峰值加速度上限（mm/s²），0=不校验。</summary>
    public CfgValue<double> AccelLimitMmS2 { get; set; } = new() { Value = 5000 };

    /// <summary>
    /// 行程动态放大系数 K：快速换向（如 2Hz 正弦）激发出的机械/伺服动态超调使实测振幅大于指令振幅，
    /// 加速度跟随模型原理上无法预测。校验时把预测行程包络以中心不变、半幅×K 展开，罩住实测；
    /// 1=不放大，由自校准提示（实测振幅÷预测振幅）标定。与加速度上限无关，不随频率变化做精确建模。
    /// </summary>
    public CfgValue<double> StrokeAmpK { get; set; } = new() { Value = 1.0 };

    /// <summary>
    /// 软限位保护开关：true=碰到软限位立即停止运动并提示；
    /// false=忽略软限位，波形坚持把整个运动做完（运行期临时放开软限位、跳过超行程校验，硬限位仍有效）。
    /// </summary>
    public CfgValue<bool> EnforceSoftLimit { get; set; } = new() { Value = true };

    /// <summary>
    /// 逐周期位置校正：正弦类速度模式是开环位置积分，驱动器/机构存在恒定的速度偏置时
    /// 中心会随时间线性漂移（高速更明显）。开启后波形环以指令速度积分出参考位置，
    /// 按与 Dpos 反馈的误差叠加一个低速校正项（等效给速度环外加低带宽位置环），
    /// 把漂移压在一个周期内不跨周期累积。仅对连续波形（正弦/多正弦）生效，
    /// 方波/脉冲本身按位置反馈自对齐。跑完后自校准会报告实测漂移速率。
    /// </summary>
    public CfgValue<bool> PosCorrEnabled { get; set; } = new() { Value = false };

    /// <summary>校正环增益 Kp (1/s)：校正速度 = Kp×(参考位置−实际位置)。小=回拉温和不干扰动力学，大=收敛快。默认 0.2（≈5s 收敛）。</summary>
    public CfgValue<double> PosCorrKpPerS { get; set; } = new() { Value = 0.2 };

    /// <summary>校正速度上限 (mm/s)：限制校正项最大回拉速度，避免严重干扰波形形状。默认 10。</summary>
    public CfgValue<double> PosCorrMaxMmS { get; set; } = new() { Value = 10 };

    // ---- 单正弦 ----
    public CfgValue<double> SineFreqHz { get; set; } = new() { Value = 0.5 };
    public CfgValue<double> SineStrokeMm { get; set; } = new() { Value = 50 };
    public CfgValue<double> SineBiasMmS { get; set; } = new() { Value = 0 };
    public CfgValue<double> SinePhaseDeg { get; set; } = new() { Value = 0 };
    public CfgValue<double> SineDurationS { get; set; } = new() { Value = 20 };
    public CfgValue<double> SineDwellS { get; set; } = new() { Value = 0 };

    // ---- 多正弦叠加 ----
    public CfgValue<double> MsBaseFreqHz { get; set; } = new() { Value = 0.2 };
    public CfgValue<int> MsCount { get; set; } = new() { Value = 16 };
    public CfgValue<double> MsFundAmpMmS { get; set; } = new() { Value = 100 };
    public CfgValue<double> MsDecayP { get; set; } = new() { Value = 1.0 };
    public CfgValue<int> MsPhaseMode { get; set; } = new() { Value = 0 };
    public CfgValue<int> MsSeed { get; set; } = new() { Value = 12345 };
    public CfgValue<double> MsDurationS { get; set; } = new() { Value = 30 };
    /// <summary>显式分量表 "f,A,φ;..."（Hz, mm/s, 度），非空则覆盖规则法；清空回到规则生成。</summary>
    public CfgValue<string> MsTable { get; set; } = new() { Value = "0.2,30,0;0.6,15,45;1.0,8,90;1.4,5,30" };

    // ---- 方波 ----
    /// <summary>途经点 "P,v,t;..."：目标位置 P(mm)、逼近速度 v(mm/s)、到点后停顿时间 t(s)。整表循环 SqCycles 次，终点停在最后一个途经点。</summary>
    public CfgValue<string> SqWaypoints { get; set; } = new() { Value = "0,200,1;100,200,1" };
    /// <summary>整表循环次数 n（≥1）：每循环依次经过所有途经点各一次。</summary>
    public CfgValue<int> SqCycles { get; set; } = new() { Value = 3 };
    /// <summary>方波运行期控制器减速度上限 (mm/s²)：到点/换向/停止时按此值减速。</summary>
    public CfgValue<double> SqDecelMmS2 { get; set; } = new() { Value = 5000 };

    // ---- 脉冲 ----
    public CfgValue<double> PuSpeedMmS { get; set; } = new() { Value = 300 };
    /// <summary>脉冲宽度序列 "t1,t2,t3,t4"（s）。</summary>
    public CfgValue<string> PuDurations { get; set; } = new() { Value = "0.5,0.3,0.2,0.1" };
    public CfgValue<int> PuCycles { get; set; } = new() { Value = 3 };
    public CfgValue<double> PuGapS { get; set; } = new() { Value = 0.5 };
    /// <summary>脉冲运行期控制器减速度上限 (mm/s²)：每脉冲回基位/换向时按此值减速。</summary>
    public CfgValue<double> PuDecelMmS2 { get; set; } = new() { Value = 5000 };

    // ---- 伪随机 PRTS ----
    public CfgValue<double> PrtsV { get; set; } = new() { Value = 200 };
    public CfgValue<int> PrtsK { get; set; } = new() { Value = 5 };
    public CfgValue<double> PrtsS { get; set; } = new() { Value = 80 };
    public CfgValue<double> PrtsDurationS { get; set; } = new() { Value = 60 };
    public CfgValue<int> PrtsSeed { get; set; } = new() { Value = 2024 };
    /// <summary>PRTS 运行期控制器减速度上限 (mm/s²)：速度态切换/回零时按此值减速。</summary>
    public CfgValue<double> PrtsDecelMmS2 { get; set; } = new() { Value = 5000 };
    /// <summary>显式逐拍速度序列（每拍一个 token：+/0/- 或 mm/s 数值，逗号/空格分隔）；非空则覆盖规则生成，逐拍原样下发（不做前瞻/回零）。</summary>
    public CfgValue<string> PrtsTable { get; set; } = new() { Value = "" };

    // ---- 便捷取值 ----
    public int GetMode() => Mode.Value;
    public double GetDtSec() => Math.Max(0.001, ControlPeriodMs.Value / 1000.0);
    public double GetAccelLimit() => AccelLimitMmS2.Value;
    public double GetStrokeAmpK() => Math.Clamp(StrokeAmpK.Value, 1.0, 3.0);
    public bool GetEnforceSoftLimit() => EnforceSoftLimit.Value;
    public bool GetPosCorrEnabled() => PosCorrEnabled.Value;
    public double GetPosCorrKp() => Math.Clamp(PosCorrKpPerS.Value, 0.01, 2.0);
    public double GetPosCorrMaxMmS() => Math.Clamp(PosCorrMaxMmS.Value, 1.0, 50.0);
}

public sealed class UiConfig
{
    public CfgValue<double> TipDisplaySeconds { get; set; } = new() { Value = 3.0 };
    public double GetTipDisplaySeconds() => TipDisplaySeconds.Value;
}
