using System.Text.Encodings.Web;
using System.Text.Json;

namespace MotorControlApp.Configuration;

/// <summary>
/// 负责从程序外部（exe 同目录 config.json）读取 / 写回配置。
/// 修改 json 文件后重启程序即可生效，无需重新编译。
/// </summary>
public static class ConfigService
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        WriteIndented = true,
        Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
        PropertyNameCaseInsensitive = true
    };

    public static string ConfigPath => Path.Combine(AppContext.BaseDirectory, "config.json");

    public static AppConfig Load()
    {
        try
        {
            if (File.Exists(ConfigPath))
            {
                string json = File.ReadAllText(ConfigPath);
                AppConfig? config = JsonSerializer.Deserialize<AppConfig>(json);
                if (config is not null)
                {
                    return config;
                }
            }
        }
        catch (Exception ex)
        {
            // 配置文件损坏时不阻塞程序启动：备份坏文件后回退默认配置
            TryBackupCorruptedFile();
            Console.WriteLine($"配置读取失败，已使用默认配置：{ex.Message}");
        }

        AppConfig fallback = new();
        TrySave(fallback);
        return fallback;
    }

    public static void TrySave(AppConfig config)
    {
        try
        {
            File.WriteAllText(ConfigPath, JsonSerializer.Serialize(config, JsonOptions));
        }
        catch
        {
            // 配置落盘失败不影响程序运行
        }
    }

    private static void TryBackupCorruptedFile()
    {
        try
        {
            if (File.Exists(ConfigPath))
            {
                string backup = ConfigPath + $".bad.{DateTime.Now:yyyyMMddHHmmss}";
                File.Move(ConfigPath, backup);
            }
        }
        catch
        {
            // 忽略备份失败
        }
    }
}
