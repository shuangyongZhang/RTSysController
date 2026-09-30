using MotorControlApp.Configuration;

namespace MotorControlApp.Waveforms;

/// <summary>
/// 运动单位换算与波形物理常量。
/// 波形引擎与 UI 一律在"负载线速度域"（mm / mm/s / mm/s²）里计算，
/// 减速比 1:40 与丝杆导程只体现在与 ZMotion 控制卡 user units 的换算里。
/// </summary>
public sealed class MotionUnits
{
    /// <summary>直线最大线速度（mm/s），对应文档 vmax = 1 m/s。</summary>
    public const double VmaxMmS = 1000.0;

    /// <summary>直线最大行程（mm），对应文档 Smax = 200。</summary>
    public const double SmaxMm = 200.0;

    // 换算系数：1 cm = 10 mm；1 mm = 0.1 cm。
    private const double MmPerCm = 10.0;

    private readonly float _units;
    private readonly float _lead;
    private readonly int _gearDen;

    public MotionUnits(ZMotionConfig z)
    {
        ArgumentNullException.ThrowIfNull(z);
        _units = z.Units.Value;
        _lead = z.Lead.Value;
        _gearDen = z.GearDenominator.Value;
    }

    /// <summary>mm（UI 物理量）→ 控制卡 user units（位置）。</summary>
    public double MmToUnits(double mm) => mm / MmPerCm * _gearDen / _lead / _units;

    /// <summary>控制卡 user units（位置/速度）→ mm。</summary>
    public double UnitsToMm(float u) => u * _units * _lead / _gearDen * MmPerCm;

    /// <summary>mm/s（UI 物理量）→ 控制卡 user units/s（速度）。</summary>
    public float MmSToUnitsPS(double mmPerS) => (float)(mmPerS / MmPerCm * _gearDen / _lead / _units);

    /// <summary>mm/s²（UI 物理量）→ 控制卡 user units/s²（加速度）。</summary>
    public float MmS2ToUnitsPS2(double mmPerS2) => (float)(mmPerS2 / MmPerCm * _gearDen / _lead / _units);
}
