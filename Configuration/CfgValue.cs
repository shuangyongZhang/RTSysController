namespace MotorControlApp.Configuration;

/// <summary>
/// 携带中文说明的配置值包装。
/// config.json 中每个字段写成 { "Des": "<中文解释>", "Value": <实际值> }，
/// 程序只读取 Value，Des 供现场/维护人员阅读理解配置用途。
/// </summary>
/// <typeparam name="T">配置值类型。</typeparam>
public sealed class CfgValue<T>
{
    /// <summary>该配置项的中文说明（仅做文档用途，程序不读取）。</summary>
    public string? Des { get; set; }

    /// <summary>该配置项的实际取值。</summary>
    public T Value { get; set; } = default!;
}