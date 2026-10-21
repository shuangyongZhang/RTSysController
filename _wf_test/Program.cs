using System.Diagnostics;

Console.WriteLine($"ticks/ms={Stopwatch.Frequency / 1000.0:F3}  IsHighRes={Stopwatch.IsHighResolution}");

foreach (double reqMs in new[] { 5.0, 10.0, 10.0, 12.5, 16.0, 20.0, 30.0 })
{
    using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(reqMs));
    var sw = Stopwatch.StartNew();
    var intervals = new List<double>();
    double prev = 0;
    for (int i = 0; i < 27; i++)
    {
        if (!await timer.WaitForNextTickAsync()) break;
        double now = sw.Elapsed.TotalSeconds;
        if (i >= 2) intervals.Add((now - prev) * 1000);
        prev = now;
    }
    intervals.Sort();
    Console.WriteLine($"req={reqMs,5:F1}ms  median={intervals[intervals.Count / 2],7:F3}ms  min={intervals[0],7:F3}  p90={intervals[(int)(intervals.Count * 0.9)],7:F3}  max={intervals[^1],7:F3}  total={sw.Elapsed.TotalMilliseconds,8:F1}");
}
