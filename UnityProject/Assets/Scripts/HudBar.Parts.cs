using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The bar's camera toggles, the action buttons and the buttons for added robots (Find cameras, Joy namespace) with
// their helpers; the rest is in HudBar.cs.
public partial class HudBar
{
    // Middle columns: one toggle per camera (and, for added robots, Find cameras / Joy namespace); the
    // first-person hint takes the row below them.
    void BuildCameraToggles(RectTransform root, Cursor at, ImageSubscriber images)
    {
        float body = HudTheme.BodyFont * HudUi.MmPerMetre;
        for (int i = 0; i < images.Panels.Count; i++)
        {
            var cam = images.Panels[i];
            var t = HudUi.Toggle(root, cam.Label, body, images.IsOn(cam), on => images.SetCameraVisible(cam, on));
            _cameraToggles.Add((cam, t));
            at.CameraSlot(i, out float left, out float top);
            HudUi.Place((RectTransform)t.transform, left, top, at.CameraColW, RowHeightMm);
        }

        if (images.Profile.isCustom)
        {
            at.CameraSlot(images.Panels.Count, out float findLeft, out float findTop);
            HudUi.Place((RectTransform)BuildFindCameras(root, body, images).transform, findLeft, findTop, at.CameraColW, RowHeightMm);
            at.CameraSlot(images.Panels.Count + 1, out float nsLeft, out float nsTop);
            HudUi.Place((RectTransform)BuildNamespaceButton(root, body, images).transform, nsLeft, nsTop, at.CameraColW, RowHeightMm);
        }

        if (images.Profile.HasModel)
        {
            _fpHint = HudUi.Label(root, "First person hint", "You see the robot's arms twice (image + model) \u2014 expected.",
                                  body * 0.85f, TextAlignmentOptions.Left);
            _fpHint.color = HudTheme.Muted;
            HudUi.Place(_fpHint.rectTransform, at.CameraLeft, at.Row(CameraRows(images)), at.CameraWidth, RowHeightMm);
        }
    }

    // Right column: Reset layout, then Back to robots (setup mode: Save robot and Cancel), then Recenter (models).
    void BuildActions(RectTransform root, Cursor at, ImageSubscriber images, TeleopHud hud)
    {
        float body = HudTheme.BodyFont * HudUi.MmPerMetre;
        var reset = HudUi.Button(root, "Reset layout", body, images.ResetLayout);
        HudUi.Place((RectTransform)reset.transform, at.ButtonsLeft, at.Row(0), ButtonColumnMm, RowHeightMm);
        if (images.InSetup)
        {
            var save = HudUi.Button(root, "Save robot", body, hud.SaveSetup);
            save.GetComponent<Image>().color = HudTheme.WithAlpha(HudTheme.Accent * SaveTint, 1f);
            HudUi.Place((RectTransform)save.transform, at.ButtonsLeft, at.Row(1), ButtonColumnMm, RowHeightMm);
            var cancel = HudUi.Button(root, "Cancel", body, hud.CancelSetup);
            HudUi.Place((RectTransform)cancel.transform, at.ButtonsLeft, at.Row(2), ButtonColumnMm, RowHeightMm);
        }
        else
        {
            var back = HudUi.Button(root, "\u2190 Robots", body, _publisher.BackToRobotSelection);
            HudUi.Place((RectTransform)back.transform, at.ButtonsLeft, at.Row(1), ButtonColumnMm, RowHeightMm);
        }
        if (images.Profile.HasModel)
        {
            _recenter = HudUi.Button(root, "Recenter", body, hud.Recenter);
            HudUi.Place((RectTransform)_recenter.transform, at.ButtonsLeft, at.Row(2), ButtonColumnMm, RowHeightMm);
        }
    }

    Button BuildFindCameras(RectTransform root, float body, ImageSubscriber images)
    {
        var find = HudUi.Button(root, FindCamerasText, body, null);
        var findLabel = find.GetComponentInChildren<TextMeshProUGUI>();
        find.onClick.AddListener(() =>
        {
            if (findLabel.text != FindCamerasText) return;   // a search is already running
            findLabel.text = "Searching\u2026";
            images.FindNewCameras(added =>
            {
                // When cameras were added the bar is rebuilt (TeleopHud); otherwise say so briefly.
                if (this == null || added > 0) return;
                findLabel.text = "No new cameras";
                StartCoroutine(ResetLabelLater(findLabel));
            });
        });
        return find;
    }

    // Joy namespace: typed on the Quest keyboard; confirming it empty means no namespace.
    Button BuildNamespaceButton(RectTransform root, float body, ImageSubscriber images)
    {
        var ns = HudUi.Button(root, NamespaceButtonText(images.Profile), body, null);
        _namespaceLabel = ns.GetComponentInChildren<TextMeshProUGUI>();
        ns.onClick.AddListener(() => _namespaceKeyboard.Open(images.Profile.robotNamespace, allowEmpty: true));
        return ns;
    }

    const string FindCamerasText = "Find cameras";

    readonly TextKeyboard _namespaceKeyboard = new TextKeyboard();
    TextMeshProUGUI _namespaceLabel;

    static string NamespaceButtonText(RobotProfile robot)
        => string.IsNullOrEmpty(robot.robotNamespace) ? $"Joy: /{RosNames.Joy}" : $"Joy: /{robot.robotNamespace}/{RosNames.Joy}";

    System.Collections.IEnumerator ResetLabelLater(TextMeshProUGUI label)
    {
        yield return new WaitForSeconds(3f);
        if (label != null) label.text = FindCamerasText;
    }

    // Change the added robot's Joy namespace ("" = none) and keep it.
    void SetNamespace(string ns)
    {
        var robot = _images.Profile;
        _publisher.SetNamespace(ns);
        robot.robotNamespace = _publisher.robotNamespace;
        if (!_images.InSetup) RobotLibrary.Save(robot);
        _namespaceLabel.text = NamespaceButtonText(robot);
        if (_images.InSetup) _joyText.text = SetupChipText(_images);
    }

    static string SetupChipText(ImageSubscriber images)
    {
        string ns = images.Profile.robotNamespace;
        return string.IsNullOrEmpty(ns) ? "SETUP · no namespace" : $"SETUP · /{ns}";
    }
}
