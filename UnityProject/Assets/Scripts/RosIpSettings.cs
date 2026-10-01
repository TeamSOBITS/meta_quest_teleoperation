using UnityEngine;

/// <summary>
/// The ROS PC's IP as typed by the user, shared by the robot selection screen and the robot
/// screens. Saved in PlayerPrefs so it survives app restarts.
/// </summary>
public static class RosIpSettings
{
    const string PrefsKey = "RosIPAddress";
    public const int Port = 10000;

    // Saved IP, or `fallback` when none has been typed yet.
    public static string Load(string fallback)
    {
        string ip = PlayerPrefs.GetString(PrefsKey, "");
        return string.IsNullOrEmpty(ip) ? fallback : ip;
    }

    public static bool HasSaved => !string.IsNullOrEmpty(PlayerPrefs.GetString(PrefsKey, ""));

    public static void Save(string ip)
    {
        PlayerPrefs.SetString(PrefsKey, ip);
        PlayerPrefs.Save();  // flush now; the app may be killed from the Quest menu without a clean quit
    }
}

/// <summary>
/// Quest system keyboard for typing text. Call <see cref="Poll"/> every frame; it returns the
/// typed text once the user confirms a changed, non-empty value, otherwise null.
/// Where no system keyboard exists (the Unity Editor), <see cref="Open"/> can confirm a
/// fallback value straight away so flows stay testable.
/// </summary>
public class TextKeyboard
{
    readonly TouchScreenKeyboardType _type;
    TouchScreenKeyboard _keyboard;
    string _current, _pending;
    bool _allowEmpty, _seenVisible;
    float _openedAt;
    // A new keyboard can report the previous one's Done status until it shows up; results are
    // only taken once it has been visible (or after this long, if it never appears).
    const float ShowTimeoutSeconds = 2f;

    public TextKeyboard(TouchScreenKeyboardType type = TouchScreenKeyboardType.Default) => _type = type;

    public bool IsOpen => _keyboard != null;
    // What the user is typing while the keyboard is open.
    public string Text => _keyboard?.text;

    // `current` is the value being replaced (confirming it unchanged counts as cancel);
    // the keyboard starts empty unless `initialText` is given.
    // allowEmpty: confirming an empty field returns "" (e.g. "no namespace") instead of cancelling.
    public void Open(string current, string fallbackWithoutKeyboard = null, string initialText = "", bool allowEmpty = false)
    {
        _current = current;
        _allowEmpty = allowEmpty;
        if (!TouchScreenKeyboard.isSupported && fallbackWithoutKeyboard != null)
        {
            _pending = fallbackWithoutKeyboard;
            return;
        }
        TouchScreenKeyboard.hideInput = false;
        _keyboard = TouchScreenKeyboard.Open(initialText ?? "", _type, false, false, false, false);
        _seenVisible = false;
        _openedAt = Time.unscaledTime;
    }

    public string Poll()
    {
        if (_pending != null)
        {
            string p = _pending;
            _pending = null;
            return p;
        }
        if (_keyboard == null) return null;
        if (_keyboard.status == TouchScreenKeyboard.Status.Visible) { _seenVisible = true; return null; }
        if (!_seenVisible && Time.unscaledTime - _openedAt < ShowTimeoutSeconds) return null;   // not shown yet

        string typed = _keyboard.text;
        bool confirmed = _seenVisible && _keyboard.status == TouchScreenKeyboard.Status.Done;
        Debug.Log($"[TextKeyboard] closed: {_keyboard.status}, seen {_seenVisible}, text \"{typed}\"");
        _keyboard = null;
        if (!confirmed) return null;
        typed ??= "";
        if (typed.Length == 0 && !_allowEmpty) return null;
        return typed != _current ? typed : null;
    }
}

/// <summary>Keyboard with the numbers-and-punctuation layout, for typing an IP.</summary>
public class IpKeyboard : TextKeyboard
{
    public IpKeyboard() : base(TouchScreenKeyboardType.NumbersAndPunctuation) { }
}
