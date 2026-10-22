using MotorControlApp.Waveforms;

const double dt = 0.01;
const double aReal = 4500, S = 80, dur = 60;
const int seed = 2024;

// 判据：若欠账本质是"指令滞后一拍/一个下发周期"的位置债，则 最大误差 ≈ V×下发间隔，随 V 与间隔线性；
// 若是"每次换向滚存的长债"，则 时间轴后段(75%)应明显大于前段(25%)、且末端均值偏置不为 0。
Console.WriteLine("V   下发间隔  25%时刻误差  75%时刻误差  后半段平均误差  最大误差 | 预测 V×间隔");
foreach (double V in new[] { 50.0, 100.0, 200.0, 300.0 })
    foreach (double grid in new[] { 0.010, 0.015625, 0.020 })
    {
        var g = new PrtsGenerator(V, 5, S, dur, seed, dt, aReal, aReal);
        int nFine = (int)Math.Round(g.TotalDurationSeconds / 0.001);

        var refCurve = new double[nFine + 1];
        double p = 0, pv = g.NextVelocity(0, 0);
        for (int i = 1; i <= nFine; i++)
        {
            double vf = g.NextVelocity(i * 0.001, 0);
            refCurve[i] = p += (pv + vf) / 2 * 0.001;
            pv = vf;
        }

        double pos = 0, va = 0, t = 0, maxErr = 0, e25 = 0, e75 = 0, sumLate = 0; int cntLate = 0;
        double half = g.TotalDurationSeconds / 2;
        while (t < g.TotalDurationSeconds)
        {
            double cmd = g.NextVelocity(t, 0);
            double van = va + Math.Clamp(cmd - va, -aReal * grid, aReal * grid);
            pos += (va + van) / 2 * grid;
            va = van; t += grid;
            double err = pos - refCurve[(int)Math.Min(nFine, t / 0.001)];
            maxErr = Math.Max(maxErr, Math.Abs(err));
            double frac = t / g.TotalDurationSeconds;
            if (frac >= 0.25 && frac < 0.25 + grid) e25 = err;
            if (frac >= 0.75 && frac < 0.75 + grid) e75 = err;
            if (frac >= 0.5) { sumLate += err; cntLate++; }
        }
        Console.WriteLine($"{V,5:F0} {grid * 1000,8:F3}    {e25,8:F2}    {e75,8:F2}      {sumLate / cntLate,8:F2}     {maxErr,7:F2}  | {V * grid,7:F2}");
    }

Console.WriteLine();
Console.WriteLine("把规划 dt 对齐真实回调网格（下发间隔固定 15.625 ms，V=200，K=5）：");
Console.WriteLine("  dt(ms)  25%误差  75%误差  后半段平均  最大误差");
foreach (double d in new[] { 0.010, 0.0125, 0.015, 0.015625, 0.016, 0.020 })
{
    var g = new PrtsGenerator(200, 5, S, dur, seed, d, aReal, aReal);
    double step = 0.0005;
    int nFine = (int)Math.Round(g.TotalDurationSeconds / step);
    var refCurve = new double[nFine + 1];
    double p = 0, pv = g.NextVelocity(0, 0);
    for (int i = 1; i <= nFine; i++)
    {
        double vf = g.NextVelocity(i * step, 0);
        refCurve[i] = p += (pv + vf) / 2 * step; pv = vf;
    }
    double pos = 0, va = 0, t = 0, maxErr = 0, e25 = 0, e75 = 0, sumLate = 0; int cntLate = 0;
    double tot = g.TotalDurationSeconds;
    while (t < tot)
    {
        double cmd = g.NextVelocity(t, 0);
        double van = va + Math.Clamp(cmd - va, -aReal * 0.015625, aReal * 0.015625);
        pos += (va + van) / 2 * 0.015625; va = van; t += 0.015625;
        double err = pos - refCurve[(int)Math.Min(nFine, t / step)];
        maxErr = Math.Max(maxErr, Math.Abs(err));
        double frac = t / tot;
        if (frac >= 0.25 && frac < 0.25 + 0.015625) e25 = err;
        if (frac >= 0.75 && frac < 0.75 + 0.015625) e75 = err;
        if (frac >= 0.5) { sumLate += err; cntLate++; }
    }
    Console.WriteLine($"{d * 1000,8:F3}  {e25,7:F2}  {e75,7:F2}   {sumLate / cntLate,8:F2}   {maxErr,7:F2}");
}

Console.WriteLine();
Console.WriteLine("节拍抖动敏感性（真实回调 14.6~16.3 ms 区间，V=200，K=5）：");
Console.WriteLine("  dt对grid   14.600ms      15.625ms      16.300ms   (最大误差 / 后半段平均)");
foreach (double d in new[] { 0.010, 0.015625 })
{
    var sb = new System.Text.StringBuilder($"{d * 1000,8:F3}  ");
    foreach (double grid in new[] { 0.0146, 0.015625, 0.0163 })
    {
        var g = new PrtsGenerator(200, 5, S, dur, seed, d, aReal, aReal);
        double step = 0.0005;
        int nFine = (int)Math.Round(g.TotalDurationSeconds / step);
        var refCurve = new double[nFine + 1];
        double p = 0, pv = g.NextVelocity(0, 0);
        for (int i = 1; i <= nFine; i++) { double vf = g.NextVelocity(i * step, 0); refCurve[i] = p += (pv + vf) / 2 * step; pv = vf; }
        double pos = 0, va = 0, t = 0, maxErr = 0, sumLate = 0; int cntLate = 0;
        double tot = g.TotalDurationSeconds;
        while (t < tot)
        {
            double cmd = g.NextVelocity(t, 0);
            double van = va + Math.Clamp(cmd - va, -aReal * grid, aReal * grid);
            pos += (va + van) / 2 * grid; va = van; t += grid;
            double err = pos - refCurve[(int)Math.Min(nFine, t / step)];
            maxErr = Math.Max(maxErr, Math.Abs(err));
            if (t / tot >= 0.5) { sumLate += err; cntLate++; }
        }
        sb.Append($"  {maxErr,6:F2}/{sumLate / cntLate,6:F2}");
    }
    Console.WriteLine(sb.ToString());
}
