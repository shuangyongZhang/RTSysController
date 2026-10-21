using MotorControlApp.Configuration;
using MotorControlApp.Forms;

namespace MotorControlApp;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();

        Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
        Application.ThreadException += (_, e) =>
        {
            AppLog.Error("UI线程", e.Exception);
            MessageBox.Show(e.Exception.Message, "程序错误（详情已写入 app.log）", MessageBoxButtons.OK, MessageBoxIcon.Error);
        };
        AppDomain.CurrentDomain.UnhandledException += (_, e) =>
            AppLog.Error("非UI线程", e.ExceptionObject as Exception);

        // 启动时从 exe 同目录读取 config.json；文件缺失或损坏时自动生成一份默认配置
        AppConfig config = ConfigService.Load();

        Application.Run(new MainForm(config));
    }
}
