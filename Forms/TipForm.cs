namespace MotorControlApp.Forms;

/// <summary>
/// 轻量 Tips 弹窗：无边框、置顶、按配置时长（默认3秒）自动消失。
/// 成功=绿色，失败=红色。宽度固定，高度随错误原因文字长度自动扩展并自动换行。
/// </summary>
public sealed class TipForm : Form
{
    private const int TipWidth = 320;
    private const int TipMinHeight = 60;
    private const int TipMaxHeight = 170;

    private readonly System.Windows.Forms.Timer _timer;

    private TipForm(string message, bool success, int displayMs)
    {
        FormBorderStyle = FormBorderStyle.None;
        StartPosition = FormStartPosition.Manual;
        ShowInTaskbar = false;
        TopMost = true;
        BackColor = success
            ? Color.FromArgb(76, 175, 80)
            : Color.FromArgb(229, 57, 53);

        var font = new Font("Microsoft YaHei UI", 10F, FontStyle.Bold);

        // 按文字量计算弹窗高度，保证较长的失败原因也能完整显示
        int height;
        using (Graphics g = CreateGraphics())
        {
            SizeF measured = g.MeasureString(message, font, TipWidth - 24);
            height = (int)Math.Ceiling(measured.Height) + 22;
        }

        height = Math.Clamp(height, TipMinHeight, TipMaxHeight);
        Size = new Size(TipWidth, height);

        var label = new Label
        {
            Text = message,
            Dock = DockStyle.Fill,
            ForeColor = Color.White,
            Font = font,
            TextAlign = ContentAlignment.MiddleCenter,
            Padding = new Padding(10, 4, 10, 4)
        };
        Controls.Add(label);

        _timer = new System.Windows.Forms.Timer { Interval = Math.Max(500, displayMs) };
        _timer.Tick += (_, _) =>
        {
            _timer.Stop();
            Close();
        };
    }

    protected override void OnShown(EventArgs e)
    {
        base.OnShown(e);
        _timer.Start();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _timer.Dispose();
        }
        base.Dispose(disposing);
    }

    /// <summary>
    /// 在宿主窗口顶部居中位置显示一条 Tips。
    /// </summary>
    public static void Show(IWin32Window owner, string message, bool success, int displayMs = 3000)
    {
        var tip = new TipForm(message, success, displayMs);

        if (owner is Form host && !host.IsDisposed)
        {
            int x = host.Left + (host.Width - tip.Width) / 2;
            int y = host.Top + 56;
            tip.Location = new Point(x, y);
        }
        else
        {
            tip.StartPosition = FormStartPosition.CenterScreen;
        }

        tip.Show(owner);
    }
}
