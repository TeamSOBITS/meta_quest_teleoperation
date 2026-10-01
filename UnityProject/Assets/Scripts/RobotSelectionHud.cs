using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

/// <summary>
/// Robot selection screen, built from the robot profiles with the same HUD toolkit as the
/// robot screens (head-locked, same distance, fonts and cards):
///
///                         Choose a robot
///        ROS IP  192.168.11.20                      [ Edit ]
///     +----------------------+   +----------------------+
///     |       [picture]      |   |       [picture]      |
///     |  SOBIT HOME          |   |  SOBIT LIGHT         |
///     |  /sobit_home · 3 cam |   |  /sobit_light · 4 cam|
///     +----------------------+   +----------------------+
///              Point at a robot and pull the trigger
///
/// The robot chosen last time carries a "Last used" tag; a green dot after the name means the
/// robot is online (it has topics in its namespace, see RobotPresence), a grey one that it is not.
/// Each card is a button: pointing at it darkens it, pulling the trigger opens that robot.
/// The ROS IP row only sets the IP; the ROS connection is opened (and its state shown) by the
/// robot screen. (A reachability ping was dropped: UnityEngine.Ping does not work on Quest.)
/// </summary>
public class RobotSelectionHud : MonoBehaviour
{
    // One card per profile, in this order.
    public RobotProfile[] robots;

    // IP shown before the user has typed one (the robot scenes' ROSConnection default).
    public string defaultIp = "127.0.0.1";

    // Sizes in mm on a canvas at HudUi.ReferenceDistance (same scale as the robot screens).
    const float CardWidthMm = 1500f, CardPaddingMm = 60f, CardGapMm = 160f, RadiusMm = 60f;
    const float IpRowWidthMm = 3000f, IpRowHeightMm = 260f, EditWidthMm = 420f;
    const float TitleFontScale = 1.5f, BackdropPaddingMm = 110f;
    public const string LastRobotKey = "LastRobot";
    const string HintText = "Point at a robot and pull the trigger";
    const float MessageSeconds = 4f, RemoveConfirmSeconds = 3f;
    const string AddRobotTitle = "Add robot";
    static readonly Color CardColor = new Color(0.21f, 0.23f, 0.28f, 1f);
    const int MaxColumns = 3;
    // Vertical position of the whole screen's centre relative to eye level (metres).
    const float CentreY = 0.1f;

    readonly IpKeyboard _keyboard = new IpKeyboard();
    readonly TextKeyboard _nameKeyboard = new TextKeyboard();
    readonly List<RobotProfile> _all = new List<RobotProfile>();
    Transform _head;
    GameObject _screen;
    TextMeshProUGUI _hint;
    float _hintUntil;
    string _ip;
    TextMeshProUGUI _ipLabel, _addName;
    RobotPresence _presence;
    bool _opening;
    readonly Dictionary<RobotProfile, Image> _dots = new Dictionary<RobotProfile, Image>();
    static readonly Color OfflineDotColor = new Color(1f, 1f, 1f, 0.18f);

    void Start()
    {
        _ip = RosIpSettings.Load(defaultIp);
        StudioEnvironment.Apply(Camera.main);
        _head = Camera.main != null ? Camera.main.transform : transform;
        Build();
        _presence = RobotPresence.Create(_all, _ip);
#if UNITY_ANDROID && !UNITY_EDITOR
        ApplyLaunchExtras();
#endif
    }

#if UNITY_ANDROID && !UNITY_EDITOR
    // Autonomous tests start the app with intent extras, e.g.
    //   am start -n <pkg>/<activity> --es robot SOBIT_HOME --es viewmode firstperson --es capture 1
    // robot = profile asset name (opens it), viewmode = firstperson | blocks, capture = 1 (save a
    // screenshot of the robot screen, see TeleopHud). Read once per app run, so "Back to robots"
    // does not open the robot again.
    static bool _extrasHandled;

    void ApplyLaunchExtras()
    {
        if (_extrasHandled) return;
        _extrasHandled = true;
        string robot = null, viewMode = null, capture = null;
        try
        {
            using (var player = new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
            using (var activity = player.GetStatic<AndroidJavaObject>("currentActivity"))
            using (var intent = activity?.Call<AndroidJavaObject>("getIntent"))
            {
                if (intent != null)
                {
                    robot = intent.Call<string>("getStringExtra", "robot");
                    viewMode = intent.Call<string>("getStringExtra", "viewmode");
                    capture = intent.Call<string>("getStringExtra", "capture");
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"FPV: could not read intent extras: {e.Message}");
            return;
        }
        Debug.Log($"FPV: intent robot={robot} viewmode={viewMode} capture={capture}");

        // DebugCapture must not linger: set only by this launch, cleared when no extra is present.
        if (capture == "1") PlayerPrefs.SetInt("DebugCapture", 1);
        else PlayerPrefs.DeleteKey("DebugCapture");
        if (string.IsNullOrEmpty(robot)) { PlayerPrefs.Save(); return; }

        var profile = _all.Find(r => r.name == robot);
        if (profile == null)
        {
            Debug.LogWarning($"FPV: intent robot '{robot}' not found");
            PlayerPrefs.Save();
            return;
        }
        if (viewMode == FirstPersonView.ModeFirstPerson || viewMode == FirstPersonView.ModeBlocks)
            FirstPersonView.ViewModeOverride = viewMode;   // this robot screen only, not saved
        PlayerPrefs.Save();
        Select(profile);
    }
#endif

    // (Re)build the whole screen: built-in robots, robots added on the headset, "Add robot".
    void Build()
    {
        if (_screen != null) Destroy(_screen);
        _all.Clear();
        _dots.Clear();
        _all.AddRange(robots);
        _all.AddRange(RobotLibrary.LoadAll());

        float body  = HudUi.BodyFontSize  * HudUi.MmPerMetre;
        float title = HudUi.TitleFontSize * HudUi.MmPerMetre;

        // Card size: picture (4:3) + name + meta line.
        float pictureW = CardWidthMm - 2f * CardPaddingMm;
        float pictureH = pictureW * 0.75f;
        float cardH = CardPaddingMm + pictureH + 40f + title * 1.3f + body * 1.4f + CardPaddingMm;

        int n = _all.Count + 1;   // + the "Add robot" card
        int cols = Mathf.Min(n, MaxColumns);
        int rows = Mathf.CeilToInt(n / (float)cols);
        float gridW = cols * CardWidthMm + (cols - 1) * CardGapMm;
        float gridH = rows * cardH + (rows - 1) * CardGapMm;

        float headingH = title * TitleFontScale * 1.4f;
        float pad = BackdropPaddingMm;
        float contentW = Mathf.Max(gridW, IpRowWidthMm);
        float widthMm = contentW + 2f * pad;
        float hintH = body * 2.2f;
        float heightMm = pad + headingH + 60f + IpRowHeightMm + 120f + gridH + 40f + hintH + pad;

        var root = HudUi.CreateCanvas("Robot Selection Screen", _head,
            new Vector3(0f, CentreY, HudUi.ReferenceDistance), new Vector2(widthMm, heightMm), interactive: true);
        _screen = root.gameObject;

        // One dark backdrop behind everything, so text reads on any background.
        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Backdrop", HudUi.PanelColor), RadiusMm * 1.5f).rectTransform);

        // Heading
        float top = pad;
        var heading = HudUi.Label(root, "Heading", "Choose a robot", title * TitleFontScale);
        heading.fontStyle = FontStyles.Bold;
        HudUi.Place(heading.rectTransform, 0f, top, widthMm, headingH);
        top += headingH + 60f;

        // ROS IP row
        float rowLeft = (widthMm - IpRowWidthMm) / 2f;
        var row = HudUi.Round(HudUi.Box(root, "ROS IP", CardColor), RadiusMm);
        HudUi.Place(row.rectTransform, rowLeft, top, IpRowWidthMm, IpRowHeightMm);
        BuildIpRow(row.rectTransform, body, title);
        top += IpRowHeightMm + 120f;

        // Robot cards, then "Add robot"
        for (int i = 0; i < n; i++)
        {
            int r = i / cols, c = i % cols;
            int inRow = Mathf.Min(cols, n - r * cols);
            float rowW = inRow * CardWidthMm + (inRow - 1) * CardGapMm;
            float left = (widthMm - rowW) / 2f + c * (CardWidthMm + CardGapMm);
            var card = i < _all.Count
                ? BuildCard(root, _all[i], pictureW, pictureH, body, title)
                : BuildAddCard(root, pictureW, pictureH, body, title);
            HudUi.Place((RectTransform)card.transform, left, top + r * (cardH + CardGapMm), CardWidthMm, cardH);
        }
        top += gridH + 40f;

        _hint = HudUi.Label(root, "Hint", HintText, body);
        _hint.color = HudUi.MutedText;
        HudUi.Place(_hint.rectTransform, 0f, top, widthMm, hintH);
    }

    // Briefly replace the hint line with a message (e.g. why a name was not accepted).
    void ShowMessage(string text)
    {
        _hint.text = text;
        _hint.color = HudUi.BadColor;
        _hintUntil = Time.time + MessageSeconds;
    }

    Button BuildAddCard(RectTransform parent, float pictureW, float pictureH, float body, float title)
    {
        var bg = HudUi.Round(HudUi.Box(parent, "Card Add robot", CardColor, raycastTarget: true), RadiusMm);
        var button = bg.gameObject.AddComponent<Button>();
        button.targetGraphic = bg;
        button.colors = HudUi.HoverColors;
        button.onClick.AddListener(() => _nameKeyboard.Open("", $"Robot {_all.Count + 1}"));

        float top = CardPaddingMm;
        var frame = HudUi.Round(HudUi.Box(bg.transform, "Picture", new Color(1f, 1f, 1f, 0.06f)), RadiusMm * 0.6f);
        HudUi.Place(frame.rectTransform, CardPaddingMm, top, pictureW, pictureH);
        var plus = HudUi.Label(frame.transform, "Plus", "+", title * 4f);
        plus.color = HudUi.AccentColor;
        HudUi.Stretch(plus.rectTransform);
        top += pictureH + 40f;

        var name = _addName = HudUi.Label(bg.transform, "Name", AddRobotTitle, title, TextAlignmentOptions.Left);
        name.fontStyle = FontStyles.Bold;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        name.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(name.rectTransform, CardPaddingMm, top, pictureW, title * 1.3f);
        top += title * 1.3f;

        var meta = HudUi.Label(bg.transform, "Details", "Name it, then pick its cameras", body, TextAlignmentOptions.Left);
        meta.color = HudUi.MutedText;
        HudUi.Place(meta.rectTransform, CardPaddingMm, top, pictureW, body * 1.4f);
        return button;
    }

    void StartSetup(string displayName)
    {
        displayName = displayName.Trim();
        if (displayName.Length == 0) { ShowMessage("Type a name for the robot"); return; }
        if (RobotLibrary.IsNameTaken(displayName, _all)) { ShowMessage($"A robot named \u201c{displayName}\u201d already exists"); return; }

        var robot = RobotLibrary.CreateNew(displayName);
        RobotProfile.SetupMode = true;
        RobotProfile.Selected = robot;
        StartCoroutine(Open(robot));
    }

    void BuildIpRow(RectTransform row, float body, float title)
    {
        const float pad = 50f, gap = 40f;
        float h = IpRowHeightMm;

        var caption = HudUi.Label(row, "Caption", "ROS IP", body, TextAlignmentOptions.Left);
        caption.color = HudUi.MutedText;
        caption.textWrappingMode = TextWrappingModes.NoWrap;
        float captionW = caption.GetPreferredValues("ROS IP").x;
        HudUi.Place(caption.rectTransform, pad, 0f, captionW, h);

        float editLeft = IpRowWidthMm - pad - EditWidthMm;
        var edit = HudUi.Button(row, "Edit", body, () => _keyboard.Open(_ip));   // starts empty
        HudUi.Place((RectTransform)edit.transform, editLeft, 50f, EditWidthMm, h - 100f);

        float ipLeft = pad + captionW + gap;
        _ipLabel = HudUi.Label(row, "IP", _ip, title, TextAlignmentOptions.Left);
        _ipLabel.textWrappingMode = TextWrappingModes.NoWrap;
        _ipLabel.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(_ipLabel.rectTransform, ipLeft, 0f, editLeft - gap - ipLeft, h);
    }

    Button BuildCard(RectTransform parent, RobotProfile robot, float pictureW, float pictureH, float body, float title)
    {
        var bg = HudUi.Round(HudUi.Box(parent, "Card " + robot.displayName, CardColor, raycastTarget: true), RadiusMm);
        var button = bg.gameObject.AddComponent<Button>();
        button.targetGraphic = bg;
        button.colors = HudUi.HoverColors;
        button.onClick.AddListener(() => Select(robot));

        float top = CardPaddingMm;
        var frame = HudUi.Round(HudUi.Box(bg.transform, "Picture", new Color(0.91f, 0.93f, 0.95f, 1f)), RadiusMm * 0.6f);
        HudUi.Place(frame.rectTransform, CardPaddingMm, top, pictureW, pictureH);
        if (robot.picture == null)
        {
            // Added robots have no picture: show their initials instead.
            frame.color = new Color(HudUi.AccentColor.r * 0.35f, HudUi.AccentColor.g * 0.35f, HudUi.AccentColor.b * 0.35f, 1f);
            var initials = HudUi.Label(frame.transform, "Initials", Initials(robot.displayName), title * 3f);
            initials.fontStyle = FontStyles.Bold;
            initials.color = HudUi.AccentColor;
            HudUi.Stretch(initials.rectTransform);
        }
        else
        {
            var pic = new GameObject("Image", typeof(RectTransform)).AddComponent<RawImage>();
            pic.transform.SetParent(frame.transform, false);
            pic.texture = robot.picture;
            pic.raycastTarget = false;
            // Fit inside the frame, keeping the picture's aspect ratio.
            float aspect = (float)robot.picture.width / robot.picture.height;
            float w = pictureW * 0.9f, h = pictureH * 0.9f;
            if (w / h > aspect) w = h * aspect; else h = w / aspect;
            var prt = pic.rectTransform;
            prt.anchorMin = prt.anchorMax = new Vector2(0.5f, 0.5f);
            prt.sizeDelta = new Vector2(w, h);
        }
        if (robot.name == PlayerPrefs.GetString(LastRobotKey, ""))
        {
            float tagH = body * 1.6f;
            var tag = HudUi.Round(HudUi.Box(frame.transform, "Last used",
                new Color(HudUi.AccentColor.r, HudUi.AccentColor.g, HudUi.AccentColor.b, 0.9f)), tagH / 2f);
            var tagText = HudUi.Label(tag.transform, "Label", "Last used", body * 0.9f);
            tagText.fontStyle = FontStyles.Bold;
            tagText.color = new Color(0.05f, 0.08f, 0.12f, 1f);
            tagText.textWrappingMode = TextWrappingModes.NoWrap;
            HudUi.Stretch(tagText.rectTransform);
            float tagW = tagText.GetPreferredValues("Last used").x + 1.6f * body;
            HudUi.Place(tag.rectTransform, pictureW - tagW - 30f, 30f, tagW, tagH);
        }
        if (robot.isCustom)
            BuildRemoveButton(frame.rectTransform, robot, body);
        top += pictureH + 40f;

        // Name, with a dot at the end of the row: green when the robot is online.
        float dot = body * 0.9f;
        var name = HudUi.Label(bg.transform, "Name", robot.displayName, title, TextAlignmentOptions.Left);
        name.fontStyle = FontStyles.Bold;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        name.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(name.rectTransform, CardPaddingMm, top, pictureW - dot - 30f, title * 1.3f);
        if (RobotPresence.CanCheck(robot))
        {
            var status = HudUi.Round(HudUi.Box(bg.transform, "Online", OfflineDotColor), dot / 2f);
            HudUi.Place(status.rectTransform, CardPaddingMm + pictureW - dot, top + (title * 1.3f - dot) / 2f, dot, dot);
            status.gameObject.SetActive(false);   // shown once the first check is done
            _dots[robot] = status;
        }
        top += title * 1.3f;

        int cams = robot.cameras != null ? robot.cameras.Length : 0;
        string ns = string.IsNullOrEmpty(robot.robotNamespace) ? "no namespace" : "/" + robot.robotNamespace;
        var meta = HudUi.Label(bg.transform, "Details",
            $"{ns}  ·  {cams} camera{(cams == 1 ? "" : "s")}", body, TextAlignmentOptions.Left);
        meta.color = HudUi.MutedText;
        HudUi.Place(meta.rectTransform, CardPaddingMm, top, pictureW, body * 1.4f);
        return button;
    }

    // "Remove" on an added robot's card; a second press within a few seconds confirms.
    void BuildRemoveButton(RectTransform frame, RobotProfile robot, float body)
    {
        var button = HudUi.Button(frame, "Remove", body * 0.9f, null);
        var label = button.GetComponentInChildren<TextMeshProUGUI>();
        HudUi.Place((RectTransform)button.transform, 30f, 30f, 360f, body * 1.7f);
        float armedUntil = -1f;
        button.onClick.AddListener(() =>
        {
            if (Time.time < armedUntil)
            {
                RobotLibrary.Delete(robot);
                if (PlayerPrefs.GetString(LastRobotKey, "") == robot.name) PlayerPrefs.DeleteKey(LastRobotKey);
                Build();
                return;
            }
            armedUntil = Time.time + RemoveConfirmSeconds;
            label.text = "Press again";
            label.color = HudUi.BadColor;
        });
    }

    static string Initials(string name)
    {
        var words = name.Split(new[] { ' ', '_', '-' }, System.StringSplitOptions.RemoveEmptyEntries);
        string s = words.Length >= 2 ? $"{words[0][0]}{words[1][0]}" : name.Length > 0 ? name.Substring(0, Mathf.Min(2, name.Length)) : "?";
        return s.ToUpperInvariant();
    }

    void Select(RobotProfile robot)
    {
        PlayerPrefs.SetString(LastRobotKey, robot.name);
        PlayerPrefs.Save();
        RobotProfile.Selected = robot;
        StartCoroutine(Open(robot));
    }

    // The online check's ROS connection must be closed before the robot screen opens its own.
    System.Collections.IEnumerator Open(RobotProfile robot)
    {
        if (_opening) yield break;
        _opening = true;
        _hint.text = $"Opening {robot.displayName}\u2026";
        _hint.color = HudUi.MutedText;
        _hintUntil = 0f;
        yield return _presence.Close();
        SceneManager.LoadScene(robot.sceneName);
    }

    void Update()
    {
        // The Quest overlay keyboard has no text field, so show what is being typed on the screen.
        _ipLabel.text = _keyboard.IsOpen ? QuestControllerPublisher.TypingDisplay(_keyboard.Text, "Type the ROS PC IP\u2026") : _ip;
        if (_addName != null)
            _addName.text = _nameKeyboard.IsOpen ? QuestControllerPublisher.TypingDisplay(_nameKeyboard.Text, "Type a name\u2026") : AddRobotTitle;

        foreach (var (robot, status) in _dots)
        {
            bool? online = _presence.IsOnline(robot);
            status.gameObject.SetActive(online.HasValue);
            status.color = online == true ? HudUi.GoodColor : OfflineDotColor;
        }

        string newName = _nameKeyboard.Poll();
        if (newName != null) StartSetup(newName);
        if (_hintUntil > 0f && Time.time > _hintUntil)
        {
            _hintUntil = 0f;
            _hint.text = HintText;
            _hint.color = HudUi.MutedText;
        }

        string newIp = _keyboard.Poll();
        if (newIp != null)
        {
            _ip = newIp;
            RosIpSettings.Save(_ip);
            _presence.SetIp(_ip);
        }
    }
}
