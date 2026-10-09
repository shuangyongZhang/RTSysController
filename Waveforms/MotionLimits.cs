namespace MotorControlApp.Waveforms;

/// <summary>波形运行/校验用的物理限制（均在 mm 域）。AccelLimitMmS2 &lt;= 0 表示不校验加速度。</summary>
public sealed class MotionLimits
{
    public MotionLimits(double negMm, double posMm, double accelLimitMmS2 = 0)
    {
        NegMm = negMm;
        PosMm = posMm;
        AccelLimitMmS2 = accelLimitMmS2;
    }

    /// <summary>负向软限位（mm），如 -100。</summary>
    public double NegMm { get; }

    /// <summary>正向软限位（mm），如 +100。</summary>
    public double PosMm { get; }

    /// <summary>峰值加速度上限（mm/s²），&lt;=0 不校验。</summary>
    public double AccelLimitMmS2 { get; }
    
    /// <summary>
    /// 运行期减速度上限（mm/s²）：阶梯型模式（方波/脉冲/PRTS）可在各自页签单独设置。
    /// &lt;=0 时运行期回退使用 <see cref="AccelLimitMmS2"/>。
    /// </summary>
    public double DecelLimitMmS2 { get; init; }

    /// <summary>是否强制软限位：false 时校验器跳过超行程判定（由 UI 按开关设置）。</summary>
    public bool EnforceSoftLimit { get; init; } = true;

    /// <summary>
    /// 行程动态放大系数 K（≥1）：快速换向激发出的机械/伺服动态超调使实测振幅大于指令振幅，
    /// 校验时把预测包络中心不变、半幅×K 展开。1=不放大（跟随模型本身不含此效应）。
    /// </summary>
    public double DynAmpK { get; init; } = 1.0;
}
