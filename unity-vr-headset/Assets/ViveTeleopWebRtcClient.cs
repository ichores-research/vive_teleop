using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.XR;
using Valve.VR;

public class ViveTeleopWebRtcClient : MonoBehaviour
{
    [Header("Server")]
    public string configUrl = "http://192.168.1.174:8088/config";
    public string fallbackServerUrl = "http://192.168.1.174:8088";
    public bool connectOnStart = true;
    public float iceGatheringTimeoutSeconds = 10f;

    [Header("Video")]
    public bool connectVideo = true;
    public Renderer targetRenderer;
    public string[] texturePropertyNames = { "_BaseMap", "_MainTex" };
    public bool headLockDisplay = true;
    public Transform displayCamera;
    public Vector3 headLockedLocalPosition = new Vector3(0f, 0f, 1.8f);
    public Vector3 headLockedLocalEulerAngles = Vector3.zero;
    public Vector3 headLockedLocalScale = new Vector3(1.6f, 1.2f, 1f);

    [Header("Input")]
    public bool connectInput = true;
    public bool sendHmdPose = true;
    public bool hmdPoseStreamingEnabled;
    public KeyCode toggleHmdPoseKey = KeyCode.S;
    public Transform poseSource;
    public float poseSendRateHz = 30f;
    public bool sendPoseWhenChannelOpens = true;
    public bool sendJoystickWristPose = true;
    public Transform wristPoseSource;
    public XRNode wristXrNode = XRNode.RightHand;
    public bool preferOpenVrControllerTracking = true;
    public Transform wristTrackingOrigin;
    public bool calibrateWristOnFirstSample = true;
    public Vector3 wristRotationOffsetEuler = Vector3.zero;
    public KeyCode recalibrateWristKey = KeyCode.R;
    public float wristPositionScale = 1f;
    public bool requireWristDeadman = true;
    [Range(0f, 1f)]
    public float wristDeadmanTriggerThreshold = 0.15f;

    [Header("6-DoF Recording")]
    public bool recordControllerPoseOnStart;
    public bool toggleRecordingWithMenuButton = true;
    public KeyCode toggleRecordingKey = KeyCode.Space;
    public string recordingFolderName = "ViveTeleopRecordings";
    public int recordingFlushIntervalSamples = 30;

    RTCPeerConnection videoPeer;
    RTCPeerConnection inputPeer;
    RTCDataChannel inputChannel;
    Coroutine webRtcUpdateRoutine;
    Coroutine poseSampleRoutine;
    bool inputChannelOpen;
    bool wristCalibrationReady;
    bool calibrateWristOnNextSample;
    bool robotWristStateReady;
    bool wristCommandActive;
    bool recenterHeadsetPoseOnNextSample;
    bool previousRecordingMenuButton;
    bool recordingActive;
    uint lastOpenVrControllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    int recordingSamplesSinceFlush;
    string recordingPath;
    StreamWriter recordingWriter;
    readonly Dictionary<uint, string> openVrDeviceNames =
        new Dictionary<uint, string>();
    readonly List<InputDevice> xrWristDevices = new List<InputDevice>();
    Vector3 controllerWristAnchorPosition;
    Quaternion controllerWristAnchorRotation = Quaternion.identity;
    Vector3 robotWristAnchorPosition;
    Quaternion robotWristAnchorRotation = Quaternion.identity;
    Vector3 lastRobotWristTargetPosition;
    Quaternion lastRobotWristTargetRotation = Quaternion.identity;
    string robotWristFrame = "base_footprint";

    public bool IsRecording => recordingActive;
    public string RecordingPath => recordingPath;

    IEnumerator Start()
    {
        Application.runInBackground = true;

        if (targetRenderer == null)
        {
            targetRenderer = GetComponent<Renderer>();
        }

        if (poseSource == null && Camera.main != null)
        {
            poseSource = Camera.main.transform;
        }

        if (displayCamera == null && Camera.main != null)
        {
            displayCamera = Camera.main.transform;
        }

        LockDisplayToCamera();

        webRtcUpdateRoutine = StartCoroutine(WebRTC.Update());

        if (poseSendRateHz > 0f)
        {
            poseSampleRoutine = StartCoroutine(SendPoseLoop());
        }

        if (ShouldRecordOnStart())
        {
            StartRecording();
        }

        if (connectOnStart)
        {
            yield return ConnectRoutine();
        }
    }

    void LateUpdate()
    {
        if (Input.GetKeyDown(toggleHmdPoseKey))
        {
            hmdPoseStreamingEnabled = !hmdPoseStreamingEnabled;
            Debug.Log(
                $"ViveTeleop WebRTC: HMD pose streaming {(hmdPoseStreamingEnabled ? "enabled" : "disabled")}."
            );
        }

        if (Input.GetKeyDown(recalibrateWristKey))
        {
            CalibrateWristRotation();
            RecenterHeadsetPose();
        }

        if (Input.GetKeyDown(toggleRecordingKey))
        {
            ToggleRecording();
        }

        LockDisplayToCamera();
    }

    public void Connect()
    {
        StartCoroutine(ConnectRoutine());
    }

    public void Disconnect()
    {
        inputChannelOpen = false;
        wristCommandActive = false;
        inputChannel?.Close();
        inputChannel = null;

        inputPeer?.Close();
        inputPeer?.Dispose();
        inputPeer = null;

        videoPeer?.Close();
        videoPeer?.Dispose();
        videoPeer = null;
    }

    public void CalibrateWristRotation()
    {
        calibrateWristOnNextSample = true;
    }

    public void RecenterHeadsetPose()
    {
        recenterHeadsetPoseOnNextSample = true;
    }

    IEnumerator ConnectRoutine()
    {
        Disconnect();

        ServerConfig serverConfig = null;
        yield return LoadServerConfig(config => serverConfig = config);

        if (serverConfig == null)
        {
            Debug.LogError("ViveTeleop WebRTC: no server config available.");
            yield break;
        }

        if (connectVideo)
        {
            yield return ConnectVideo(serverConfig);
        }

        if (connectInput)
        {
            yield return LoadRobotWristState(serverConfig);
            if (!robotWristStateReady)
            {
                Debug.LogError(
                    "ViveTeleop WebRTC: robot wrist state is unavailable; " +
                    "input is disabled to avoid an uncalibrated arm command.");
                yield break;
            }

            yield return ConnectInput(serverConfig);
        }
    }

    IEnumerator LoadServerConfig(Action<ServerConfig> onLoaded)
    {
        var resolvedConfigUrl = ResolveConfiguredUrl();
        using var request = UnityWebRequest.Get(resolvedConfigUrl);
        yield return request.SendWebRequest();

        if (request.result == UnityWebRequest.Result.Success)
        {
            var config = JsonUtility.FromJson<ServerConfig>(request.downloadHandler.text);
            if (config != null && !string.IsNullOrWhiteSpace(config.serverUrl))
            {
                onLoaded(config);
                yield break;
            }
        }
        else
        {
            Debug.LogWarning(
                $"ViveTeleop WebRTC: config fetch failed from {resolvedConfigUrl}: {request.error}"
            );
        }

        var serverUrl = TrimTrailingSlash(fallbackServerUrl);
        onLoaded(new ServerConfig
        {
            serverUrl = serverUrl,
            offerUrl = $"{serverUrl}/offer",
            inputOfferUrl = $"{serverUrl}/input_offer",
            iceServers = Array.Empty<IceServerConfig>(),
        });
    }

    IEnumerator LoadRobotWristState(ServerConfig serverConfig)
    {
        robotWristStateReady = false;
        wristCalibrationReady = false;
        wristCommandActive = false;

        var stateUrl = $"{TrimTrailingSlash(serverConfig.serverUrl)}/robot_state";
        using var request = UnityWebRequest.Get(stateUrl);
        yield return request.SendWebRequest();

        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError(
                $"ViveTeleop WebRTC: robot state fetch failed from {stateUrl}: " +
                request.error);
            yield break;
        }

        var snapshot =
            JsonUtility.FromJson<RobotStateSnapshot>(request.downloadHandler.text);
        if (snapshot == null ||
            snapshot.wrist == null ||
            snapshot.wrist.position == null ||
            snapshot.wrist.orientation == null)
        {
            Debug.LogError(
                $"ViveTeleop WebRTC: robot state from {stateUrl} is not ready.");
            yield break;
        }

        if (!snapshot.ready)
        {
            Debug.LogWarning(
                "ViveTeleop WebRTC: full robot state is still warming up; " +
                "using the valid wrist transform for arm anchoring.");
        }

        var position = snapshot.wrist.position.ToVector3();
        var rotation = snapshot.wrist.orientation.ToQuaternion();
        if (!IsFinite(position) || !TryNormalize(ref rotation))
        {
            Debug.LogError(
                "ViveTeleop WebRTC: robot state contained an invalid wrist pose.");
            yield break;
        }

        robotWristFrame = string.IsNullOrWhiteSpace(snapshot.wrist.frame)
            ? "base_footprint"
            : snapshot.wrist.frame;
        robotWristAnchorPosition = position;
        robotWristAnchorRotation = rotation;
        lastRobotWristTargetPosition = position;
        lastRobotWristTargetRotation = rotation;
        robotWristStateReady = true;

        Debug.Log(
            $"ViveTeleop WebRTC: loaded robot wrist anchor " +
            $"frame='{robotWristFrame}' xyz=({position.x:F3}, " +
            $"{position.y:F3}, {position.z:F3}).");
    }

    IEnumerator ConnectVideo(ServerConfig serverConfig)
    {
        videoPeer = CreatePeer(serverConfig, "video");
        videoPeer.OnTrack = e =>
        {
            if (e.Track is VideoStreamTrack videoTrack)
            {
                videoTrack.OnVideoReceived += ApplyVideoTexture;
            }
        };

        var transceiver = videoPeer.AddTransceiver(TrackKind.Video);
        transceiver.Direction = RTCRtpTransceiverDirection.RecvOnly;

        yield return ExchangeOffer(videoPeer, serverConfig.VideoOfferUrl, "video");
    }

    IEnumerator ConnectInput(ServerConfig serverConfig)
    {
        inputPeer = CreatePeer(serverConfig, "input");
        inputChannel = inputPeer.CreateDataChannel("input");
        inputChannel.OnOpen = () =>
        {
            inputChannelOpen = true;
            Debug.Log("ViveTeleop WebRTC input channel open.");

            if (sendPoseWhenChannelOpens)
            {
                SendPose();
            }
        };
        inputChannel.OnClose = () =>
        {
            inputChannelOpen = false;
            Debug.Log("ViveTeleop WebRTC input channel closed.");
        };

        yield return ExchangeOffer(inputPeer, serverConfig.InputOfferUrl, "input");
    }

    RTCPeerConnection CreatePeer(ServerConfig serverConfig, string label)
    {
        var iceServers = new List<RTCIceServer>();
        if (serverConfig.iceServers != null)
        {
            foreach (var iceServer in serverConfig.iceServers)
            {
                if (iceServer?.urls == null || iceServer.urls.Length == 0)
                {
                    continue;
                }

                iceServers.Add(new RTCIceServer
                {
                    urls = iceServer.urls,
                    username = iceServer.username,
                    credential = iceServer.credential,
                    credentialType = RTCIceCredentialType.Password,
                });
            }
        }

        RTCPeerConnection peer;
        if (iceServers.Count > 0)
        {
            var configuration = new RTCConfiguration
            {
                iceServers = iceServers.ToArray(),
            };
            peer = new RTCPeerConnection(ref configuration);
        }
        else
        {
            peer = new RTCPeerConnection();
        }

        peer.OnIceConnectionChange = state =>
            Debug.Log($"ViveTeleop WebRTC {label} ICE state: {state}");
        peer.OnConnectionStateChange = state =>
            Debug.Log($"ViveTeleop WebRTC {label} connection state: {state}");
        peer.OnIceGatheringStateChange = state =>
            Debug.Log($"ViveTeleop WebRTC {label} ICE gathering: {state}");

        return peer;
    }

    IEnumerator ExchangeOffer(RTCPeerConnection peer, string offerUrl, string label)
    {
        if (string.IsNullOrWhiteSpace(offerUrl))
        {
            Debug.LogError($"ViveTeleop WebRTC {label}: offer URL is empty.");
            yield break;
        }

        var createOfferOp = peer.CreateOffer();
        yield return createOfferOp;
        if (createOfferOp.IsError)
        {
            Debug.LogError(
                $"ViveTeleop WebRTC {label}: create offer failed: {createOfferOp.Error.message}"
            );
            yield break;
        }

        var offerDescription = createOfferOp.Desc;
        var setLocalOp = peer.SetLocalDescription(ref offerDescription);
        yield return setLocalOp;
        if (setLocalOp.IsError)
        {
            Debug.LogError(
                $"ViveTeleop WebRTC {label}: set local description failed: {setLocalOp.Error.message}"
            );
            yield break;
        }

        yield return WaitForIceGathering(peer, label);

        var offer = new SdpMessage
        {
            sdp = peer.LocalDescription.sdp,
            type = "offer",
        };
        var requestBody = JsonUtility.ToJson(offer);

        Debug.Log($"ViveTeleop WebRTC {label}: POST offer to {offerUrl}");

        using var request = new UnityWebRequest(offerUrl, "POST")
        {
            uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(requestBody)),
            downloadHandler = new DownloadHandlerBuffer(),
        };
        request.SetRequestHeader("Content-Type", "application/json");

        yield return request.SendWebRequest();
        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError(
                $"ViveTeleop WebRTC {label}: offer POST failed: {request.error}\n{request.downloadHandler.text}"
            );
            yield break;
        }

        var answer = JsonUtility.FromJson<SdpMessage>(request.downloadHandler.text);
        if (answer == null || string.IsNullOrWhiteSpace(answer.sdp))
        {
            Debug.LogError(
                $"ViveTeleop WebRTC {label}: invalid answer from {offerUrl}: {request.downloadHandler.text}"
            );
            yield break;
        }

        var answerDescription = new RTCSessionDescription
        {
            sdp = answer.sdp,
            type = ParseSdpType(answer.type),
        };

        var setRemoteOp = peer.SetRemoteDescription(ref answerDescription);
        yield return setRemoteOp;
        if (setRemoteOp.IsError)
        {
            Debug.LogError(
                $"ViveTeleop WebRTC {label}: set remote description failed: {setRemoteOp.Error.message}"
            );
            yield break;
        }

        Debug.Log($"ViveTeleop WebRTC {label}: connected to {offerUrl}");
    }

    IEnumerator WaitForIceGathering(RTCPeerConnection peer, string label)
    {
        var start = Time.realtimeSinceStartup;
        while (peer.GatheringState != RTCIceGatheringState.Complete)
        {
            if (Time.realtimeSinceStartup - start > iceGatheringTimeoutSeconds)
            {
                Debug.LogWarning(
                    $"ViveTeleop WebRTC {label}: ICE gathering timed out; sending partial offer."
                );
                yield break;
            }

            yield return null;
        }
    }

    IEnumerator SendPoseLoop()
    {
        var wait = new WaitForSeconds(1f / Mathf.Max(1f, poseSendRateHz));
        while (true)
        {
            SendPose();
            yield return wait;
        }
    }

    public void SendPose()
    {
        var payload = new PosePayload
        {
            type = "unity_teleop_pose",
            timestamp = Time.realtimeSinceStartup,
            headsetRecenter = recenterHeadsetPoseOnNextSample,
        };

        if (sendHmdPose && hmdPoseStreamingEnabled)
        {
            var hmdSource = poseSource;
            if (hmdSource == null && Camera.main != null)
            {
                hmdSource = Camera.main.transform;
            }

            if (hmdSource != null)
            {
                var hmdPosition = hmdSource.position;
                var hmdRotation = hmdSource.rotation;
                payload.hmdAvailable = true;
                payload.hmdFrame = "unity_world";
                payload.hmdPx = hmdPosition.x;
                payload.hmdPy = hmdPosition.y;
                payload.hmdPz = hmdPosition.z;
                payload.hmdRx = hmdRotation.x;
                payload.hmdRy = hmdRotation.y;
                payload.hmdRz = hmdRotation.z;
                payload.hmdRw = hmdRotation.w;
            }
        }

        if (sendJoystickWristPose &&
            TryGetWristPose(out var wristPose, out var joystickState))
        {
            var correctedRotation =
                wristPose.rotation * Quaternion.Euler(wristRotationOffsetEuler);
            var deadmanPressed =
                !requireWristDeadman ||
                joystickState.trigger >= wristDeadmanTriggerThreshold ||
                joystickState.grip >= 0.5f;
            var startedCommand = deadmanPressed && !wristCommandActive;
            var shouldCalibrate =
                calibrateWristOnNextSample ||
                (!wristCalibrationReady &&
                    (calibrateWristOnFirstSample || deadmanPressed)) ||
                startedCommand;

            if (shouldCalibrate && robotWristStateReady)
            {
                controllerWristAnchorPosition = wristPose.position;
                controllerWristAnchorRotation = correctedRotation;
                robotWristAnchorPosition = lastRobotWristTargetPosition;
                robotWristAnchorRotation = lastRobotWristTargetRotation;
                wristCalibrationReady = true;
                calibrateWristOnNextSample = false;
                Debug.Log(
                    "ViveTeleop WebRTC: anchored controller to the current " +
                    "robot wrist target.");
            }

            wristCommandActive = deadmanPressed;

            if (robotWristStateReady && wristCalibrationReady && deadmanPressed)
            {
                var unityDelta =
                    wristPose.position - controllerWristAnchorPosition;
                var robotDelta =
                    UnityDeltaToRobot(unityDelta) * wristPositionScale;
                var unityRotationDelta =
                    correctedRotation *
                    Quaternion.Inverse(controllerWristAnchorRotation);
                var robotRotationDelta =
                    UnityRotationDeltaToRobot(unityRotationDelta);

                lastRobotWristTargetPosition =
                    robotWristAnchorPosition + robotDelta;
                lastRobotWristTargetRotation =
                    robotRotationDelta * robotWristAnchorRotation;
                TryNormalize(ref lastRobotWristTargetRotation);
            }

            payload.wristAvailable = true;
            payload.wristCommandEnabled =
                robotWristStateReady &&
                wristCalibrationReady &&
                deadmanPressed;
            payload.wristSource = wristPose.source;
            payload.wristFrame = "unity_world";
            payload.wristPx = wristPose.position.x;
            payload.wristPy = wristPose.position.y;
            payload.wristPz = wristPose.position.z;
            payload.wristRx = correctedRotation.x;
            payload.wristRy = correctedRotation.y;
            payload.wristRz = correctedRotation.z;
            payload.wristRw = correctedRotation.w;
            payload.robotWristFrame = robotWristFrame;
            payload.robotWristPx = lastRobotWristTargetPosition.x;
            payload.robotWristPy = lastRobotWristTargetPosition.y;
            payload.robotWristPz = lastRobotWristTargetPosition.z;
            payload.robotWristRx = lastRobotWristTargetRotation.x;
            payload.robotWristRy = lastRobotWristTargetRotation.y;
            payload.robotWristRz = lastRobotWristTargetRotation.z;
            payload.robotWristRw = lastRobotWristTargetRotation.w;
            payload.joystickAxisX = joystickState.primary2DAxis.x;
            payload.joystickAxisY = joystickState.primary2DAxis.y;
            payload.joystickTrigger = joystickState.trigger;
            payload.joystickGrip = joystickState.grip;
            payload.joystickPrimaryButton = joystickState.primaryButton;

            HandleRecordingMenuButton(joystickState.menuButton);
        }
        else if (sendJoystickWristPose)
        {
            wristCommandActive = false;
        }

        var payloadJson = JsonUtility.ToJson(payload);
        if (recordingActive && payload.wristAvailable)
        {
            WriteRecordingSample(payloadJson);
        }

        if (inputChannelOpen && inputChannel != null)
        {
            inputChannel.Send(payloadJson);
            recenterHeadsetPoseOnNextSample = false;
        }
    }

    static Vector3 UnityDeltaToRobot(Vector3 unityDelta)
    {
        return new Vector3(
            unityDelta.z,
            -unityDelta.x,
            unityDelta.y);
    }

    static Quaternion UnityRotationDeltaToRobot(Quaternion unityRotation)
    {
        var robotRotation = new Quaternion(
            -unityRotation.z,
            unityRotation.x,
            -unityRotation.y,
            unityRotation.w);
        TryNormalize(ref robotRotation);
        return robotRotation;
    }

    static bool TryNormalize(ref Quaternion rotation)
    {
        var normSquared =
            (rotation.x * rotation.x) +
            (rotation.y * rotation.y) +
            (rotation.z * rotation.z) +
            (rotation.w * rotation.w);
        if (!float.IsFinite(normSquared) || normSquared < 1e-8f)
        {
            return false;
        }

        var inverseNorm = 1f / Mathf.Sqrt(normSquared);
        rotation = new Quaternion(
            rotation.x * inverseNorm,
            rotation.y * inverseNorm,
            rotation.z * inverseNorm,
            rotation.w * inverseNorm);
        return true;
    }

    static bool IsFinite(Vector3 value)
    {
        return
            float.IsFinite(value.x) &&
            float.IsFinite(value.y) &&
            float.IsFinite(value.z);
    }

    void LockDisplayToCamera()
    {
        if (!headLockDisplay)
        {
            return;
        }

        if (displayCamera == null && Camera.main != null)
        {
            displayCamera = Camera.main.transform;
        }

        if (displayCamera == null)
        {
            return;
        }

        if (transform.parent != displayCamera)
        {
            transform.SetParent(displayCamera, false);
        }

        transform.localPosition = headLockedLocalPosition;
        transform.localRotation = Quaternion.Euler(headLockedLocalEulerAngles);
        transform.localScale = headLockedLocalScale;
    }

    bool TryGetWristPose(out WristPose wristPose, out JoystickState joystickState)
    {
        wristPose = default;
        joystickState = default;

        if (wristPoseSource != null)
        {
            wristPose = new WristPose
            {
                source = wristPoseSource.name,
                position = wristPoseSource.position,
                rotation = wristPoseSource.rotation,
            };
            return true;
        }

        if (preferOpenVrControllerTracking && OpenVR.System != null)
        {
            return TryGetOpenVrWristPose(out wristPose, out joystickState);
        }

        xrWristDevices.Clear();
        InputDevices.GetDevicesAtXRNode(wristXrNode, xrWristDevices);
        foreach (var device in xrWristDevices)
        {
            if (!device.isValid)
            {
                continue;
            }

            var hasPosition =
                device.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
            var hasRotation =
                device.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);

            device.TryGetFeatureValue(CommonUsages.primary2DAxis, out var primary2DAxis);
            device.TryGetFeatureValue(CommonUsages.trigger, out var trigger);
            device.TryGetFeatureValue(CommonUsages.grip, out var grip);
            device.TryGetFeatureValue(CommonUsages.primaryButton, out var primaryButton);
            device.TryGetFeatureValue(CommonUsages.menuButton, out var menuButton);

            if (!hasPosition || !hasRotation)
            {
                continue;
            }

            if (wristTrackingOrigin != null)
            {
                position = wristTrackingOrigin.TransformPoint(position);
                rotation = wristTrackingOrigin.rotation * rotation;
            }

            wristPose = new WristPose
            {
                source = $"{wristXrNode}:{device.name}",
                position = position,
                rotation = rotation,
            };
            joystickState = new JoystickState
            {
                primary2DAxis = primary2DAxis,
                trigger = trigger,
                grip = grip,
                primaryButton = primaryButton,
                menuButton = menuButton,
            };
            return true;
        }

        return false;
    }

    bool TryGetOpenVrWristPose(
        out WristPose wristPose,
        out JoystickState joystickState)
    {
        wristPose = default;
        joystickState = default;

        var system = OpenVR.System;
        if (system == null)
        {
            return false;
        }

        var role = wristXrNode == XRNode.LeftHand
            ? ETrackedControllerRole.LeftHand
            : ETrackedControllerRole.RightHand;
        var deviceIndex = system.GetTrackedDeviceIndexForControllerRole(role);
        if (deviceIndex == OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            return false;
        }

        var controllerState = new VRControllerState_t();
        var trackedPose = new TrackedDevicePose_t();
        var hasControllerState = system.GetControllerStateWithPose(
            SteamVR_Settings.instance.trackingSpace,
            deviceIndex,
            ref controllerState,
            (uint)Marshal.SizeOf(typeof(VRControllerState_t)),
            ref trackedPose);
        if (!hasControllerState ||
            !trackedPose.bDeviceIsConnected ||
            !trackedPose.bPoseIsValid)
        {
            return false;
        }

        var trackedTransform =
            new SteamVR_Utils.RigidTransform(trackedPose.mDeviceToAbsoluteTracking);
        var position = trackedTransform.pos;
        var rotation = trackedTransform.rot;
        if (wristTrackingOrigin != null)
        {
            position = wristTrackingOrigin.TransformPoint(position);
            rotation = wristTrackingOrigin.rotation * rotation;
        }

        var pressed = controllerState.ulButtonPressed;
        var menuButton = IsOpenVrButtonPressed(
            pressed,
            EVRButtonId.k_EButton_ApplicationMenu);
        var gripButton = IsOpenVrButtonPressed(
            pressed,
            EVRButtonId.k_EButton_Grip);

        lastOpenVrControllerIndex = deviceIndex;
        wristPose = new WristPose
        {
            source = $"OpenVR:{role}:{GetOpenVrDeviceName(system, deviceIndex)}",
            position = position,
            rotation = rotation,
        };
        joystickState = new JoystickState
        {
            primary2DAxis = new Vector2(
                controllerState.rAxis0.x,
                controllerState.rAxis0.y),
            trigger = Mathf.Clamp01(controllerState.rAxis1.x),
            grip = gripButton ? 1f : 0f,
            primaryButton = menuButton,
            menuButton = menuButton,
        };
        return true;
    }

    public void ToggleRecording()
    {
        if (recordingActive)
        {
            StopRecording();
        }
        else
        {
            StartRecording();
        }
    }

    public void StartRecording()
    {
        if (recordingActive)
        {
            return;
        }

        try
        {
            var directory = ResolveRecordingDirectory();
            Directory.CreateDirectory(directory);
            recordingPath = Path.Combine(
                directory,
                $"vive_controller_6dof_{DateTime.UtcNow:yyyyMMdd_HHmmss_fff}Z.jsonl");
            recordingWriter = new StreamWriter(
                recordingPath,
                false,
                new UTF8Encoding(false));
            recordingSamplesSinceFlush = 0;
            recordingActive = true;
            PulseRecordingState(true);
            Debug.Log($"ViveTeleop 6-DoF recording started: {recordingPath}");
        }
        catch (Exception exception)
        {
            recordingActive = false;
            recordingWriter?.Dispose();
            recordingWriter = null;
            Debug.LogError(
                $"ViveTeleop 6-DoF recording could not start: {exception.Message}");
        }
    }

    public void StopRecording()
    {
        if (!recordingActive && recordingWriter == null)
        {
            return;
        }

        recordingActive = false;
        try
        {
            recordingWriter?.Flush();
            recordingWriter?.Dispose();
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                $"ViveTeleop 6-DoF recording close failed: {exception.Message}");
        }
        finally
        {
            recordingWriter = null;
        }

        PulseRecordingState(false);
        Debug.Log($"ViveTeleop 6-DoF recording saved: {recordingPath}");
    }

    void HandleRecordingMenuButton(bool menuButton)
    {
        if (toggleRecordingWithMenuButton &&
            menuButton &&
            !previousRecordingMenuButton)
        {
            ToggleRecording();
        }

        previousRecordingMenuButton = menuButton;
    }

    void WriteRecordingSample(string payloadJson)
    {
        if (recordingWriter == null)
        {
            return;
        }

        try
        {
            recordingWriter.WriteLine(payloadJson);
            recordingSamplesSinceFlush++;
            if (recordingSamplesSinceFlush >=
                Mathf.Max(1, recordingFlushIntervalSamples))
            {
                recordingWriter.Flush();
                recordingSamplesSinceFlush = 0;
            }
        }
        catch (Exception exception)
        {
            Debug.LogError(
                $"ViveTeleop 6-DoF recording failed: {exception.Message}");
            StopRecording();
        }
    }

    bool ShouldRecordOnStart()
    {
        if (recordControllerPoseOnStart)
        {
            return true;
        }

        var envValue =
            Environment.GetEnvironmentVariable("VIVE_TELEOP_RECORD_CONTROLLER");
        if (string.Equals(envValue, "1", StringComparison.OrdinalIgnoreCase) ||
            string.Equals(envValue, "true", StringComparison.OrdinalIgnoreCase))
        {
            return true;
        }

        foreach (var arg in Environment.GetCommandLineArgs())
        {
            if (string.Equals(
                arg,
                "--record-controller",
                StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }

        return false;
    }

    string ResolveRecordingDirectory()
    {
        var configuredDirectory =
            Environment.GetEnvironmentVariable("VIVE_TELEOP_RECORDING_DIR");
        if (!string.IsNullOrWhiteSpace(configuredDirectory))
        {
            return Path.GetFullPath(configuredDirectory);
        }

        return Path.Combine(
            Application.persistentDataPath,
            string.IsNullOrWhiteSpace(recordingFolderName)
                ? "ViveTeleopRecordings"
                : recordingFolderName);
    }

    void PulseRecordingState(bool started)
    {
        if (!preferOpenVrControllerTracking)
        {
            return;
        }

        var system = OpenVR.System;
        if (system == null ||
            lastOpenVrControllerIndex == OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            return;
        }

        system.TriggerHapticPulse(
            lastOpenVrControllerIndex,
            0,
            started ? (ushort)1800 : (ushort)600);
    }

    static bool IsOpenVrButtonPressed(ulong pressed, EVRButtonId button)
    {
        return (pressed & (1UL << (int)button)) != 0;
    }

    string GetOpenVrDeviceName(CVRSystem system, uint deviceIndex)
    {
        if (openVrDeviceNames.TryGetValue(deviceIndex, out var cachedName))
        {
            return cachedName;
        }

        var error = ETrackedPropertyError.TrackedProp_Success;
        var buffer = new StringBuilder(128);
        system.GetStringTrackedDeviceProperty(
            deviceIndex,
            ETrackedDeviceProperty.Prop_ModelNumber_String,
            buffer,
            (uint)buffer.Capacity,
            ref error);
        var deviceName = error == ETrackedPropertyError.TrackedProp_Success
            ? buffer.ToString()
            : $"device-{deviceIndex}";
        openVrDeviceNames[deviceIndex] = deviceName;
        return deviceName;
    }

    void ApplyVideoTexture(Texture texture)
    {
        if (targetRenderer == null || texture == null)
        {
            return;
        }

        foreach (var propertyName in texturePropertyNames)
        {
            if (!string.IsNullOrWhiteSpace(propertyName) &&
                targetRenderer.material.HasProperty(propertyName))
            {
                targetRenderer.material.SetTexture(propertyName, texture);
            }
        }
    }

    string ResolveConfiguredUrl()
    {
        var envUrl = Environment.GetEnvironmentVariable("VIVE_TELEOP_WEBRTC_CONFIG_URL");
        if (!string.IsNullOrWhiteSpace(envUrl))
        {
            return envUrl;
        }

        foreach (var arg in Environment.GetCommandLineArgs())
        {
            const string prefix = "--webrtc-config-url=";
            if (arg.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
            {
                return arg.Substring(prefix.Length);
            }
        }

        return configUrl;
    }

    static string TrimTrailingSlash(string value)
    {
        return string.IsNullOrWhiteSpace(value) ? "" : value.TrimEnd('/');
    }

    static RTCSdpType ParseSdpType(string value)
    {
        return string.Equals(value, "answer", StringComparison.OrdinalIgnoreCase)
            ? RTCSdpType.Answer
            : RTCSdpType.Offer;
    }

    void OnDestroy()
    {
        StopRecording();
        Disconnect();
        if (poseSampleRoutine != null)
        {
            StopCoroutine(poseSampleRoutine);
            poseSampleRoutine = null;
        }

        if (webRtcUpdateRoutine != null)
        {
            StopCoroutine(webRtcUpdateRoutine);
            webRtcUpdateRoutine = null;
        }
    }

    [Serializable]
    class ServerConfig
    {
        public string serverUrl;
        public string offerUrl;
        public string inputOfferUrl;
        public IceServerConfig[] iceServers;

        public string VideoOfferUrl =>
            !string.IsNullOrWhiteSpace(offerUrl)
                ? offerUrl
                : $"{TrimTrailingSlash(serverUrl)}/offer";

        public string InputOfferUrl =>
            !string.IsNullOrWhiteSpace(inputOfferUrl)
                ? inputOfferUrl
                : $"{TrimTrailingSlash(serverUrl)}/input_offer";
    }

    [Serializable]
    class IceServerConfig
    {
        public string[] urls;
        public string username;
        public string credential;
    }

    [Serializable]
    class SdpMessage
    {
        public string sdp;
        public string type;
    }

    [Serializable]
    class RobotStateSnapshot
    {
        public bool ready;
        public RobotWristState wrist;
    }

    [Serializable]
    class RobotWristState
    {
        public string frame;
        public JsonVector3 position;
        public JsonQuaternion orientation;
    }

    [Serializable]
    class JsonVector3
    {
        public float x;
        public float y;
        public float z;

        public Vector3 ToVector3()
        {
            return new Vector3(x, y, z);
        }
    }

    [Serializable]
    class JsonQuaternion
    {
        public float x;
        public float y;
        public float z;
        public float w;

        public Quaternion ToQuaternion()
        {
            return new Quaternion(x, y, z, w);
        }
    }

    [Serializable]
    class PosePayload
    {
        public string type;
        public float timestamp;
        public bool hmdAvailable;
        public string hmdFrame;
        public float hmdPx;
        public float hmdPy;
        public float hmdPz;
        public float hmdRx;
        public float hmdRy;
        public float hmdRz;
        public float hmdRw;
        public bool headsetRecenter;
        public bool wristAvailable;
        public bool wristCommandEnabled;
        public string wristSource;
        public string wristFrame;
        public float wristPx;
        public float wristPy;
        public float wristPz;
        public float wristRx;
        public float wristRy;
        public float wristRz;
        public float wristRw;
        public string robotWristFrame;
        public float robotWristPx;
        public float robotWristPy;
        public float robotWristPz;
        public float robotWristRx;
        public float robotWristRy;
        public float robotWristRz;
        public float robotWristRw;
        public float joystickAxisX;
        public float joystickAxisY;
        public float joystickTrigger;
        public float joystickGrip;
        public bool joystickPrimaryButton;
    }

    struct WristPose
    {
        public string source;
        public Vector3 position;
        public Quaternion rotation;
    }

    struct JoystickState
    {
        public Vector2 primary2DAxis;
        public float trigger;
        public float grip;
        public bool primaryButton;
        public bool menuButton;
    }
}
