using TMPro;
using UnityEngine;

/// <summary>
/// Sets up the robot screen around the camera blocks: studio surroundings, the HUD bar
/// (ROS IP + controls), the camera block dragger, controller hints and the optional
/// "lazy follow" mode. Runs after ImageSubscriber so it can use the blocks it created.
///
/// For a robot being added (setup mode) it first shows a "Setting up" card while the
/// cameras are discovered, keeps Joy off (layout mode) and offers Save robot / Cancel.
/// </summary>
[DefaultExecutionOrder(100)]
public class TeleopHud : MonoBehaviour
{
    public QuestControllerPublisher publisher;
    public ImageSubscriber images;

    // Panels follow this transform; defaults to the main camera.
    public Transform hudParent;

    const string LazyFollowKey = "LazyFollow";

    Transform _bar;
    HeadFollower _follower;
    GameObject _waiting;
    TextMeshProUGUI _waitingStatus;

    public bool LazyFollow { get; private set; }

    void Start()
    {
        StudioEnvironment.Apply(Camera.main);

        if (hudParent == null && Camera.main != null)
            hudParent = Camera.main.transform;

        if (images.InSetup)
            publisher.publishJoy = false;  // layout mode: the trigger arranges blocks, nothing reaches a robot

        if (images.IsReady)
            BuildHud();
        else
        {
            ShowWaiting();
            images.Ready += BuildHud;
        }
    }

    void BuildHud()
    {
        images.Ready -= BuildHud;
        if (_waiting != null) Destroy(_waiting);

        _bar = HudBar.Create(hudParent, publisher, images, this).transform;

        var dragger = gameObject.AddComponent<PanelDragger>();
        dragger.publisher = publisher;
        dragger.images = images;

        ControllerHints.Create(hudParent);

        if (PlayerPrefs.GetInt(LazyFollowKey, 0) == 1)
            SetLazyFollow(true);
    }

    // Shown in setup mode until camera topics have been found.
    void ShowWaiting()
    {
        float title = HudUi.TitleFontSize * HudUi.MmPerMetre;
        float body = HudUi.BodyFontSize * HudUi.MmPerMetre;
        const float w = 2600f, h = 760f, pad = 80f;

        var root = HudUi.CreateCanvas("Setup Status", hudParent, new Vector3(0f, 0.2f, HudUi.ReferenceDistance),
            new Vector2(w, h), interactive: true);
        _waiting = root.gameObject;
        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudUi.PanelColor), 60f).rectTransform);

        var heading = HudUi.Label(root, "Heading", $"Setting up {images.Profile.displayName}", title);
        heading.fontStyle = FontStyles.Bold;
        HudUi.Place(heading.rectTransform, pad, pad, w - 2 * pad, title * 1.5f);

        _waitingStatus = HudUi.Label(root, "Status", "", body);
        _waitingStatus.color = HudUi.MutedText;
        HudUi.Place(_waitingStatus.rectTransform, pad, pad + title * 1.6f, w - 2 * pad, body * 3f);

        var cancel = HudUi.Button(root, "Cancel", body, CancelSetup);
        HudUi.Place((RectTransform)cancel.transform, (w - 560f) / 2f, h - pad - 150f, 560f, 150f);
    }

    void Update()
    {
        if (_waitingStatus != null) _waitingStatus.text = images.SetupStatus;
    }

    // Setup mode: keep the discovered cameras, shown/hidden choices and layout as a new robot.
    public void SaveSetup()
    {
        RobotLibrary.Save(images.Profile);
        PlayerPrefs.SetString(RobotSelectionHud.LastRobotKey, images.Profile.name);
        PlayerPrefs.Save();
        RobotProfile.SetupMode = false;
        publisher.BackToRobotSelection();
    }

    // Setup mode: discard the robot being added.
    public void CancelSetup()
    {
        ImageSubscriber.ForgetLayout(images.Profile);
        RobotProfile.SetupMode = false;
        publisher.BackToRobotSelection();
    }

    // Off (default): the HUD is rigidly attached to the head. On: it follows the head with a
    // dead zone and a short delay (see HeadFollower). Panel positions relative to the head
    // are kept either way, so switching never moves a panel within the view.
    public void SetLazyFollow(bool on)
    {
        if (on == LazyFollow) return;
        LazyFollow = on;
        PlayerPrefs.SetInt(LazyFollowKey, on ? 1 : 0);
        PlayerPrefs.Save();

        if (on)
        {
            if (_follower == null) _follower = HeadFollower.Create(hudParent);
            _follower.Snap();
        }
        Transform parent = on ? _follower.transform : hudParent;

        foreach (var panel in images.Panels)
            panel.transform.SetParent(parent, false);
        _bar.SetParent(parent, false);
        images.panelParent = parent;
    }
}
