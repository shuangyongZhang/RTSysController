namespace MotorControlApp;

/// <summary>
/// 轻量错误日志：追加写入 exe 同目录 app.log。
/// 记录失败绝不抛异常，避免日志本身影响主流程。
/// </summary>
internal static class AppLog
{
    private static readonly object Gate = new();

    public static string FilePath => System.IO.Path.Combine(AppContext.BaseDirectory, "app.log");

    public static void Error(string scope, Exception? ex)
    {
        try
        {
            string line = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} [ERROR] {scope}: {ex}\r\n";
            lock (Gate) System.IO.File.AppendAllText(FilePath, line);
        }
        catch
        {
        }
    }
}
