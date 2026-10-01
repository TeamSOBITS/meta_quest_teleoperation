using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

/// <summary>
/// Robot selection screen, built from the robot profiles with the same HUD toolkit as the
/// robot screens (head-locked, same distance, fonts and cards):
///
///                         Choose a robot
///        ROS IP  192.168.11.20        ( reachable )   [ Edit ]
///     +----------------------+   +----------------------+
///     |       [picture]      |   |       [picture]      |
///     |  SOBIT HOME          |   |  SOBIT LIGHT         |
///     |  /sobit_home · 3 cam |   |  /sobit_light · 4 cam|
///     +----------------------+   +----------------------+
///
/// Each card is a button: pointing at it darkens it, pulling the trigger opens that robot.
/// The ROS IP row checks whether the ROS endpoint answers on that IP, without opening a
/// ROS connection (the robot screen owns the connection).
/// </summary>
public class RobotSelectionHud : MonoBehaviour
{
    // One card per profile, in this order.
    public RobotProfile[] robots;

    // IP shown before the user has typed one (the robot scenes' ROSConnection default).
    public string defaultIp = "127.0.0.1";

    // Sizes in mm on a canvas at HudUi.ReferenceDistance (same scale as the robot screens).
    const float CardWidthMm = 1500f, CardPaddingMm = 60f, CardGapMm = 160f, RadiusMm = 60f;
    const float IpRowWidthMm = 3000f, IpRowHeightMm = 260f, EditWidthMm = 420f, PillWidthMm = 640f;
    const float TitleFontScale = 1.5f, BackdropPaddingMm = 110f;
    static readonly Color CardColor = new Color(0.21f, 0.23f, 0.28f, 1f);
    const int MaxColumns = 3;
    // Vertical position of the whole screen's centre relative to eye level (metres).
    const float CentreY = 0.1f;

    readonly IpKeyboard _keyboard = new IpKeyboard();
    string _ip;
    TextMeshProUGUI _ipLabel, _pillText;
    Image _pill;
    enum Reach { Checking, Reachable, Unreachable }
    Reach _reach = Reach.Checking;
    int _probeId;

    void Start()
    {
        _ip = RosIpSettings.Load(defaultIp);
        StudioEnvironment.Apply(Camera.main);
        var head = Camera.main != null ? Camera.main.transform : transform;
        Build(head);
        Probe();
    }

    void Build(Transform head)
    {
        float body  = HudUi.BodyFontSize  * HudUi.MmPerMetre;
        float title = HudUi.TitleFontSize * HudUi.MmPerMetre;

        // Card size: picture (4:3) + name + meta line.
        float pictureW = CardWidthMm - 2f * CardPaddingMm;
        float pictureH = pictureW * 0.75f;
        float cardH = CardPaddingMm + pictureH + 40f + title * 1.3f + body * 1.4f + CardPaddingMm;

        int n = Mathf.Max(1, robots.Length);
        int cols = Mathf.Min(n, MaxColumns);
        int rows = Mathf.CeilToInt(n / (float)cols);
        float gridW = cols * CardWidthMm + (cols - 1) * CardGapMm;
        float gridH = rows * cardH + (rows - 1) * CardGapMm;

        float headingH = title * TitleFontScale * 1.4f;
        float pad = BackdropPaddingMm;
        float contentW = Mathf.Max(gridW, IpRowWidthMm);
        float widthMm = contentW + 2f * pad;
        float heightMm = pad + headingH + 60f + IpRowHeightMm + 120f + gridH + pad;

        var root = HudUi.CreateCanvas("Robot Selection Screen", head,
            new Vector3(0f, CentreY, HudUi.ReferenceDistance), new Vector2(widthMm, heightMm), interactive: true);

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

        // Robot cards
        for (int i = 0; i < robots.Length; i++)
        {
            int r = i / cols, c = i % cols;
            int inRow = Mathf.Min(cols, robots.Length - r * cols);
            float rowW = inRow * CardWidthMm + (inRow - 1) * CardGapMm;
            float left = (widthMm - rowW) / 2f + c * (CardWidthMm + CardGapMm);
            var card = BuildCard(root, robots[i], pictureW, pictureH, body, title);
            HudUi.Place((RectTransform)card.transform, left, top + r * (cardH + CardGapMm), CardWidthMm, cardH);
        }
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
        var edit = HudUi.Button(row, "Edit", body, () => _keyboard.Open(_ip));
        HudUi.Place((RectTransform)edit.transform, editLeft, 50f, EditWidthMm, h - 100f);

        float pillH = body * 1.7f;
        float pillLeft = editLeft - gap - PillWidthMm;
        _pill = HudUi.Round(HudUi.Box(row, "Status", Color.clear), pillH / 2f);
        HudUi.Place(_pill.rectTransform, pillLeft, (h - pillH) / 2f, PillWidthMm, pillH);
        _pillText = HudUi.Label(_pill.transform, "Label", "", body);
        HudUi.Stretch(_pillText.rectTransform);

        float ipLeft = pad + captionW + gap;
        _ipLabel = HudUi.Label(row, "IP", _ip, title, TextAlignmentOptions.Left);
        _ipLabel.textWrappingMode = TextWrappingModes.NoWrap;
        _ipLabel.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(_ipLabel.rectTransform, ipLeft, 0f, pillLeft - gap - ipLeft, h);
        ShowReach();
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
        if (robot.picture != null)
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
        top += pictureH + 40f;

        var name = HudUi.Label(bg.transform, "Name", robot.displayName, title, TextAlignmentOptions.Left);
        name.fontStyle = FontStyles.Bold;
        HudUi.Place(name.rectTransform, CardPaddingMm, top, pictureW, title * 1.3f);
        top += title * 1.3f;

        int cams = robot.cameras != null ? robot.cameras.Length : 0;
        var meta = HudUi.Label(bg.transform, "Details",
            $"/{robot.robotNamespace}  ·  {cams} camera{(cams == 1 ? "" : "s")}", body, TextAlignmentOptions.Left);
        meta.color = HudUi.MutedText;
        HudUi.Place(meta.rectTransform, CardPaddingMm, top, pictureW, body * 1.4f);
        return button;
    }

    void Select(RobotProfile robot)
    {
        RobotProfile.Selected = robot;
        SceneManager.LoadScene(robot.sceneName);
    }

    void Update()
    {
        _ipLabel.text = _keyboard.IsOpen ? _keyboard.Text : _ip;

        string newIp = _keyboard.Poll();
        if (newIp != null)
        {
            _ip = newIp;
            RosIpSettings.Save(_ip);
            Probe();
        }
    }

    // Check whether the ROS endpoint answers on the current IP. Only the latest check updates the UI.
    async void Probe()
    {
        int id = ++_probeId;
        _reach = Reach.Checking;
        ShowReach();
        bool ok = await RosIpSettings.ProbeAsync(_ip);
        if (this == null || id != _probeId) return;
        _reach = ok ? Reach.Reachable : Reach.Unreachable;
        ShowReach();
    }

    void ShowReach()
    {
        if (_pill == null) return;
        var c = _reach switch
        {
            Reach.Reachable   => HudUi.GoodColor,
            Reach.Unreachable => HudUi.BadColor,
            _                 => HudUi.MutedText,
        };
        _pill.color = new Color(c.r, c.g, c.b, 0.18f);
        _pillText.color = c;
        _pillText.text = _reach switch
        {
            Reach.Reachable   => "reachable",
            Reach.Unreachable => "not reachable",
            _                 => "checking…",
        };
    }
}
