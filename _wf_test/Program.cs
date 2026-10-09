using MotorControlApp.Waveforms;

static void Run(string name, IWaveformGenerator g, double dt, MotionLimits lim)
{
    var r = WaveformValidator.Simulate(g, dt, lim);
    Console.WriteLine($"[{name}] stroke=[{r.MinPosMm:F1},{r.MaxPosMm:F1}] ok={r.Ok} scale={r.SuggestedScale:F3}");
    foreach (var v in r.Violations) Console.WriteLine("    ! " + v);
}

// 1) 用户 0.5Hz 验证场景基准：K=1 时预测 [-27.8,27.8]（与界面一致）
Run("psi-110 S27.85 a4500 K1", new SineGenerator(2, 27.852, 0, -110 * Math.PI / 180, 20, 0),
    0.01, new MotionLimits(-50, 50, 4500) { EnforceSoftLimit = true });

// 2) 同参数 K=1.4：包络中心不变、半幅×1.4 → [≈-38.9, 38.9]，仍 <47.5 判限 → 通过但余量真实反映超调
Run("psi-110 S27.85 a4500 K1.4", new SineGenerator(2, 27.852, 0, -110 * Math.PI / 180, 20, 0),
    0.01, new MotionLimits(-50, 50, 4500) { EnforceSoftLimit = true, DynAmpK = 1.4 });

// 3) 大参数 + K=1.4：应报超限，缩放二分搜索应给出放大后仍可通过的系数
var limK = new MotionLimits(-50, 50, 7000) { EnforceSoftLimit = true, DynAmpK = 1.4 };
Run("psi-100 S40 a7000 K1.4", new SineGenerator(2, 40, 0, -100 * Math.PI / 180, 20, 0), 0.01, limK);

// 4) 缩放闭环：S×scale 后重新校验（K=1.4）应通过
{
    var r = WaveformValidator.Simulate(new SineGenerator(2, 40, 0, -100 * Math.PI / 180, 20, 0), 0.01, limK);
    double s = r.SuggestedScale;
    if (s > 0 && s < 1)
        Run($"scaled S={40 * s:F1} K1.4", new SineGenerator(2, 40 * s, 0, -100 * Math.PI / 180, 20, 0), 0.01, limK);
}

// 5) 方波 K=1 回归不变
Run("square+-10 K1", new SquareGenerator(new[] { new Waypoint(10, 200, 0.2), new Waypoint(-10, 200, 0.2) }, 2),
    0.01, new MotionLimits(-50, 50, 7000) { EnforceSoftLimit = true });
