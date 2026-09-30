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
