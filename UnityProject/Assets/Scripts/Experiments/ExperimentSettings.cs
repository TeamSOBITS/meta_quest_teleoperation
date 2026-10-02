using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Registry of the experiment toggles shown in the <see cref="ExperimentsPanel"/>. Each toggle is
/// saved as PlayerPrefs "Exp/&lt;key&gt;". A feature reads <see cref="IsOn"/> (and listens to
/// <see cref="Changed"/>); removing a feature = delete its file and its Register line in
/// <see cref="RegisterAll"/>. See docs/experiments/2026-10-02-teleop-experiments.md.
/// </summary>
public static class ExperimentSettings
{
    public const string Rtt = "rtt", Status = "status", HandCams = "handcams", Targets = "targets",
                        BaseVel = "basevel", HeadLock = "headlock";

    class Entry { public string key, label; public bool defaultOn; }

    static readonly List<Entry> _entries = new List<Entry>();

    // Raised when a toggle changes: (key, on).
    public static event Action<string, bool> Changed;

    public static void Register(string key, string label, bool defaultOn)
    {
        var existing = _entries.Find(e => e.key == key);
        if (existing != null) { existing.label = label; existing.defaultOn = defaultOn; return; }
        _entries.Add(new Entry { key = key, label = label, defaultOn = defaultOn });
    }

    // The whole programme; features implemented later only read their key.
    public static void RegisterAll()
    {
        Register(Rtt, "Round-trip latency", true);
        Register(Status, "Status strip", true);
        Register(HandCams, "Hand cams in first person", true);
        Register(Targets, "Arm target markers", true);
        Register(BaseVel, "Base velocity arrow", true);
        Register(HeadLock, "Image follows my head", false);
    }

    public static bool IsOn(string key)
    {
        var e = _entries.Find(x => x.key == key);
        return PlayerPrefs.GetInt("Exp/" + key, e != null && e.defaultOn ? 1 : 0) == 1;
    }

    public static void Set(string key, bool on)
    {
        if (IsOn(key) == on) return;
        PlayerPrefs.SetInt("Exp/" + key, on ? 1 : 0);
        PlayerPrefs.Save();
        Debug.Log($"[Experiments] {key} -> {(on ? "on" : "off")}");
        Changed?.Invoke(key, on);
    }

    public static IEnumerable<(string key, string label, bool on)> All
    {
        get
        {
            foreach (var e in _entries.ToArray()) yield return (e.key, e.label, IsOn(e.key));
        }
    }
}
