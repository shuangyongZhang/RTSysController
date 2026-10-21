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

    /// <summary>该配置项的实际取值。缺省按类型给出安全默认值，避免 JSON 缺 "Value" 键时得到 null。</summary>
    public T Value { get; set; } = CreateDefault();

    private static T CreateDefault()
    {
        if (typeof(T) == typeof(string)) return (T)(object)string.Empty;
        if (typeof(T).IsArray) return (T)(object)Array.CreateInstance(typeof(T).GetElementType()!, 0);
        return default!;
    }
}