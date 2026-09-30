namespace MotorControlApp.Waveforms;

/// <summary>
/// 多正弦专用"合成预览校验"：给定 16~32 个分量后，全时域仿真合成波形，
/// 实时算出合成后峰值速度、积分行程范围、峰值加速度，并判断是否超限（不是逐分量判限，
/// 而是叠加后含相长干涉的瞬时值判限）。UI 参数一改就调用它即时提示，不等启动。
/// </summary>
public static class SynthesisPreview
{
    public static WaveformSimResult Preview(IReadOnlyList<SineComponent> components,
        double durationSec, double dtSec, MotionLimits lim)
    {
        var gen = new MultiSineGenerator(components, durationSec);
        return WaveformValidator.Simulate(gen, dtSec, lim);
    }

    public static WaveformSimResult PreviewFromRule(double baseFreqHz, int count, double fundamentalAmpMmS,
        double decayP, MsPhaseMode phaseMode, int seed, double durationSec, double dtSec, MotionLimits lim)
    {
        var gen = MultiSineGenerator.CreateFromRule(baseFreqHz, count, fundamentalAmpMmS, decayP, phaseMode, seed, durationSec);
        return WaveformValidator.Simulate(gen, dtSec, lim);
    }

    /// <summary>把每个分量幅值乘以 scale，用于"一键缩放至安全"。</summary>
    public static IReadOnlyList<SineComponent> Scale(IReadOnlyList<SineComponent> components, double scale)
        => components.Select(c => c with { AmpMmS = c.AmpMmS * scale }).ToArray();
}
