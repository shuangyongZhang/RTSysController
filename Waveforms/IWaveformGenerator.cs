namespace MotorControlApp.Waveforms;

/// <summary>
/// 波形轨迹生成器统一抽象：每控制拍产生一个"带符号的负载线速度指令"（mm/s）。
/// 五种运动模式（单正弦 / 多正弦叠加 / 方波 / 脉冲 / 伪随机 PRTS）各实现一个生成器。
/// 控制器以 CSV 速度模式逐拍下发生成的速度；位置仅用于到达判定、行程软限位与启动前校验。
/// 生成器应设计为纯计算 + 内部状态，便于单元测试与启动前时域仿真校验。
/// </summary>
public interface IWaveformGenerator
{
    /// <summary>整段运行总时长（秒）；返回 <see cref="double.PositiveInfinity"/> 表示由 <see cref="IsFinished"/> 决定结束。</summary>
    double TotalDurationSeconds { get; }

    /// <summary>模式名称，供 UI 提示。</summary>
    string ModeName { get; }

    /// <summary>
    /// 速度指令是否连续（无瞬时跳变）。连续波形（正弦/多正弦）的加速度由波形本身决定，需判限；
    /// 阶梯型波形（方波/脉冲/PRTS）速度是分段恒定的瞬时跳变，Δv/dt 无物理意义——实际加速度由
    /// 控制器 Accel/Decel 斜坡钳制，故这类波形跳过加速度判限。默认 false。
    /// </summary>
    bool ContinuousVelocity => false;

    /// <summary>
    /// 加速度判限是否有意义。默认与 <see cref="ContinuousVelocity"/> 相同。
    /// 单正弦插入间歇段后，周期末是设计内的零速阶跃（由控制器减速斜坡钳制），
    /// 不按阶跃沿 An/dt 判限，改由 <see cref="AnalyticalPeakAccelMmS2"/> 判正弦本体。
    /// </summary>
    bool AccelCheckMeaningful => ContinuousVelocity;

    /// <summary>
    /// 波形本体的解析峰值加速度（mm/s²）；0 表示无解析值，按仿真逐拍 Δv/dt 统计。
    /// 用于含设计内速度阶跃的连续波形：只判波形本体，不把阶跃沿算作加速度。
    /// </summary>
    double AnalyticalPeakAccelMmS2 => 0;

    /// <summary>
    /// 最高频率分量 (Hz)，供校验器做奈奎斯特检查（防止采样混叠后低估峰值）。
    /// 0 表示无频率成分（方波/脉冲/PRTS 等阶梯型）。
    /// </summary>
    double MaxFrequencyHz => 0;

    /// <summary>
    /// 是否为位置反馈闭环（方波/脉冲）：速度指令由采样到的当前位置决定，采样周期越长到达判定越晚、
    /// 换向过冲越大。校验器/采样器对这类波形按物理定时器精度取保守仿真周期。默认 false。
    /// </summary>
    bool PositionFeedback => false;

    /// <summary>复位内部状态（回到起点/相位/序列头），每次启动前调用。</summary>
    void Reset();

    /// <summary>
    /// 返回本拍目标线速度（mm/s，带符号）。
    /// </summary>
    /// <param name="tSec">自启动以来的累计时间（秒）。</param>
    /// <param name="curPosMm">当前负载位置（mm），供方波/脉冲按位置反馈判到达；连续波形可忽略。</param>
    double NextVelocity(double tSec, double curPosMm);

    /// <summary>波形是否已跑完（到时或完成设定循环次数）。</summary>
    bool IsFinished(double tSec);
}
