using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

// The head camera's image card of the first-person layout (build, size from camera_info, frames, lens
// undistortion, link health); the fields are in FirstPersonView.cs.
public partial class FirstPersonView
{
    // --- Image quad ---

    // First-person layout: the head camera's image card on the camera frame. Blocks layout: no card,
    // the camera blocks show the images. Safe to call repeatedly.
    public void SetFirstPersonLayout(bool on)
    {
        if (_model == null || on == ImageShown) return;
        if (on)
        {
            BuildQuad();
            UseTexture();
        }
        else DestroyQuad();
        DevLog.Log("FPV", $"image card {(on ? "shown" : "removed")}");
    }

    void DestroyQuad()
    {
        _images.FrameReady -= OnFrame;
        _images.CameraVisibilityChanged -= OnCameraVisibility;
        _images.LabelsChanged -= OnLabels;
        if (_cameraIndex >= 0) _images.ForceDecode(_cameraIndex, false);
        if (_canvasRt != null) Destroy(_canvasRt.gameObject);
        if (_lag != null) Destroy(_lag.gameObject);
        if (_surround != null) Destroy(_surround.gameObject);
        _lag = null; _surround = null;
        _canvasRt = _viewRt = _cardRt = null;
        _card = null; _view = null; _name = null; _badge = null; _rename = null;
        _quadFramed = false;
    }

    // Layout mode: the card's Rename button is available.
    public void SetEditable(bool editable)
    {
        if (_rename != null) _rename.gameObject.SetActive(editable);
    }

    void BuildQuad()
    {
        Transform parent = _camFrame != null ? _camFrame : _model.Root;

        var style = CameraCard.Style.Block(CardScale);
        style.SortingOrder = HudTheme.SortingOrder - 10;   // behind the HUD
        style.WaitingText = "Waiting for head camera";
        style.WaitingFontMm = style.NameFontMm * WaitingFontRatio;
        style.WaitingInsetMm = 0f;
        style.PinView = true;
        _card = CameraCard.Create(parent, "FPV Image", style, null, "Head Camera");
        _canvasRt = _card.Rect;
        _canvasRt.localPosition = new Vector3(0f, 0f, QuadDistance);
        _canvasRt.localRotation = Quaternion.identity;
        _card.Renamed += () =>
        {
            if (_cameraIndex >= 0) _images.BeginRename(_images.Panels[_cameraIndex]);
        };
        _viewRt = _card.ViewRect;
        _view = _card.View;
        _name = _card.NameLabel;
        _cardRt = _card.CardRect;
        _badge = _card.Badge;
        _rename = _card.RenameButton;

        _lag = HeadLagOutline.Create(Head, _card, QuadDistance, style.RadiusMm);
        _surround = Surround.Create(transform);
        ApplyUndistort();
        ApplyQuadSize();
    }

    // Size and offset from the camera intrinsics, or the default field of view until they arrive.
    void ApplyQuadSize()
    {
        if (_viewRt == null) return;
        float widthM, heightM, shiftXM = 0f, shiftYM = 0f;
        if (_hasInfo)
        {
            widthM = (float)(QuadDistance * _infoW / _fx);
            heightM = (float)(QuadDistance * _infoH / _fy);
            shiftXM = (float)(-(_cx - _infoW / 2.0) / _fx * QuadDistance);
            shiftYM = (float)((_cy - _infoH / 2.0) / _fy * QuadDistance);
        }
        else
        {
            widthM = 2f * QuadDistance * Mathf.Tan((_profile.defaultHfov > 0f ? _profile.defaultHfov : RobotProfile.FallbackHfov) / 2f);
            heightM = widthM / _textureAspect;
        }
        _card.Config = _cameraIndex >= 0 ? _images.Panels[_cameraIndex].Config : null;
        _card.SetSize(widthM * HudUi.MmPerMetre, heightM * HudUi.MmPerMetre, new Vector2(shiftXM, shiftYM) * HudUi.MmPerMetre);
    }

    // Lens distortion: the card's image goes through the undistortion material (flips stay in the uvRect).
    void ApplyUndistort()
    {
        if (_view == null || _undistort == null) return;
        _view.material = _undistort;
        DevLog.Log("FPV", "lens distortion corrected (plumb_bob)");
    }

    void UseTexture()
    {
        if (_cameraIndex < 0)
        {
            Debug.LogWarning($"FPV: robot has no first-person camera in the profile (or not shown); no image");
            return;
        }
        var config = _images.Panels[_cameraIndex].Config;
        _textureAspect = config.Aspect;
        _card.SetLabel(_images.Panels[_cameraIndex].Label);
        ApplyQuadSize();
        _images.FrameReady += OnFrame;
        _images.CameraVisibilityChanged += OnCameraVisibility;
        _images.LabelsChanged += OnLabels;
        SetCameraOn(_images.IsOn(_images.Panels[_cameraIndex]));
    }

    // A camera was renamed: the card shows the head camera's name.
    void OnLabels()
    {
        if (this == null || _card == null || _cameraIndex < 0) return;
        _card.SetLabel(_images.Panels[_cameraIndex].Label);
    }

    void OnCameraVisibility(int index, bool on)
    {
        if (this == null || index != _cameraIndex) return;
        SetCameraOn(on);
    }

    // Camera toggled in the bar: show / hide the card and stop / resume decoding its frames.
    void SetCameraOn(bool on)
    {
        CameraOn = on;
        if (_canvasRt != null) _canvasRt.gameObject.SetActive(on);
        _images.ForceDecode(_cameraIndex, on);
        DevLog.Log("FPV", $"head camera {(on ? "on" : "off")}");
    }

    void Update()
    {
        if (_badge != null && CameraOn && _cameraIndex >= 0) _badge.Tick(_images, _cameraIndex);
        if (_card != null && CameraOn && Time.unscaledTime >= _nextLink)
        {
            _nextLink = Time.unscaledTime + CameraBadge.RefreshSeconds;
            _card.SetLink(_health.Current, _health.AgeS, lostLabel: true);
        }
    }

    void OnFrame(int index, Texture2D tex)
    {
        if (this == null || _view == null || index != _cameraIndex || tex == null) return;
        _frames++;
        if (!_quadFramed)
        {
            _quadFramed = true;
            if (!_hasInfo)
            {
                _textureAspect = (float)tex.width / tex.height;
                ApplyQuadSize();
            }
        }
        _card.Config = _images.Panels[_cameraIndex].Config;
        _card.SetTexture(tex);   // raw decoding can replace the instance
    }

    void SubscribeCameraInfo()
    {
        string info = _profile.FirstPersonCamera?.cameraInfoSuffix;
        if (string.IsNullOrEmpty(info)) return;
        string topic = _profile.FullTopic(info);
        ROSConnection.GetOrCreateInstance().Subscribe<CameraInfoMsg>(topic, OnCameraInfo);
    }

    // The first usable message wins.
    void OnCameraInfo(CameraInfoMsg msg)
    {
        if (this == null || !isActiveAndEnabled || _hasInfo || msg == null || msg.K == null || msg.K.Length < 6) return;
        if (msg.K[0] <= 0.0 || msg.K[4] <= 0.0 || msg.width == 0 || msg.height == 0) return;
        _fx = msg.K[0]; _fy = msg.K[4]; _cx = msg.K[2]; _cy = msg.K[5];
        _infoW = msg.width; _infoH = msg.height;
        _hasInfo = true;
        _undistort = Undistort.Create(msg);   // null unless the camera reports a plumb_bob distortion
        ApplyUndistort();
        ApplyQuadSize();
        DevLog.Log("FPV", $"camera_info {msg.width}x{msg.height} fx={_fx:F1} fy={_fy:F1} cx={_cx:F1} cy={_cy:F1} " +
                  (_viewRt != null ? $"quad={_viewRt.sizeDelta.x / 1000f:F3}x{_viewRt.sizeDelta.y / 1000f:F3} m" : "(no image card in the blocks layout)"));
    }
}
