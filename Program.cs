using MotorControlApp.Configuration;
using MotorControlApp.Forms;

namespace MotorControlApp;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();

        // 启动时从 exe 同目录读取 config.json；文件缺失或损坏时自动生成一份默认配置
        AppConfig config = ConfigService.Load();

        Application.Run(new MainForm(config));
    }
}
