using System;
using System.Collections;
using System.Collections.Generic;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.XR;

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
    public Transform poseSource;
    public float poseSendRateHz = 30f;
    public bool sendPoseWhenChannelOpens = true;
    public bool sendJoystickWristPose = true;
    public Transform wristPoseSource;
    public XRNode wristXrNode = XRNode.RightHand;
    public bool calibrateWristOnFirstSample = true;
    public Vector3 wristRotationOffsetEuler = Vector3.zero;
    public KeyCode recalibrateWristKey = KeyCode.R;

    RTCPeerConnection videoPeer;
    RTCPeerConnection inputPeer;
    RTCDataChannel inputChannel;
    Coroutine webRtcUpdateRoutine;
    Coroutine poseSendRoutine;
    bool inputChannelOpen;
    bool wristCalibrationReady;
    bool calibrateWristOnNextSample;
    Quaternion wristCalibrationInverse = Quaternion.identity;

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

        if (connectOnStart)
        {
            yield return ConnectRoutine();
        }
    }

    void LateUpdate()
    {
        if (Input.GetKeyDown(recalibrateWristKey))
        {
            CalibrateWristRotation();
        }

        LockDisplayToCamera();
    }

    public void Connect()
    {
        StartCoroutine(ConnectRoutine());
    }

    public void Disconnect()
    {
        if (poseSendRoutine != null)
        {
            StopCoroutine(poseSendRoutine);
            poseSendRoutine = null;
        }

        inputChannelOpen = false;
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

            if (poseSendRoutine == null && poseSendRateHz > 0f)
            {
                poseSendRoutine = StartCoroutine(SendPoseLoop());
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
        while (inputChannelOpen)
        {
            SendPose();
            yield return wait;
        }

        poseSendRoutine = null;
    }

    public void SendPose()
    {
        if (!inputChannelOpen || inputChannel == null)
        {
            return;
        }

        var hmdSource = poseSource;
        if (hmdSource == null && Camera.main != null)
        {
            hmdSource = Camera.main.transform;
        }

        var payload = new PosePayload
        {
            type = "unity_teleop_pose",
            timestamp = Time.realtimeSinceStartup,
        };

        if (sendHmdPose && hmdSource != null)
        {
            var hmdPosition = hmdSource.position;
            var hmdRotation = hmdSource.rotation;
            payload.hmdAvailable = true;
            payload.hmdPx = hmdPosition.x;
            payload.hmdPy = hmdPosition.y;
            payload.hmdPz = hmdPosition.z;
            payload.hmdRx = hmdRotation.x;
            payload.hmdRy = hmdRotation.y;
            payload.hmdRz = hmdRotation.z;
            payload.hmdRw = hmdRotation.w;
        }

        if (sendJoystickWristPose &&
            TryGetWristPose(out var wristPose, out var joystickState))
        {
            var correctedRotation =
                wristPose.rotation * Quaternion.Euler(wristRotationOffsetEuler);
            var shouldCalibrate =
                calibrateWristOnNextSample ||
                (calibrateWristOnFirstSample && !wristCalibrationReady);

            if (shouldCalibrate)
            {
                wristCalibrationInverse = Quaternion.Inverse(correctedRotation);
                wristCalibrationReady = true;
                calibrateWristOnNextSample = false;
                Debug.Log("ViveTeleop WebRTC: calibrated wrist rotation reference.");
            }

            var robotWristRotation = wristCalibrationReady
                ? wristCalibrationInverse * correctedRotation
                : correctedRotation;

            payload.wristAvailable = true;
            payload.wristSource = wristPose.source;
            payload.wristPx = wristPose.position.x;
            payload.wristPy = wristPose.position.y;
            payload.wristPz = wristPose.position.z;
            payload.wristRx = correctedRotation.x;
            payload.wristRy = correctedRotation.y;
            payload.wristRz = correctedRotation.z;
            payload.wristRw = correctedRotation.w;
            payload.robotWristFrame = wristCalibrationReady
                ? "calibrated_relative"
                : "unity_world";
            payload.robotWristRx = robotWristRotation.x;
            payload.robotWristRy = robotWristRotation.y;
            payload.robotWristRz = robotWristRotation.z;
            payload.robotWristRw = robotWristRotation.w;
            payload.joystickAxisX = joystickState.primary2DAxis.x;
            payload.joystickAxisY = joystickState.primary2DAxis.y;
            payload.joystickTrigger = joystickState.trigger;
            payload.joystickGrip = joystickState.grip;
            payload.joystickPrimaryButton = joystickState.primaryButton;
        }

        inputChannel.Send(JsonUtility.ToJson(payload));
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

        var devices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(wristXrNode, devices);
        foreach (var device in devices)
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

            if (!hasPosition && !hasRotation)
            {
                continue;
            }

            wristPose = new WristPose
            {
                source = $"{wristXrNode}:{device.name}",
                position = hasPosition ? position : Vector3.zero,
                rotation = hasRotation ? rotation : Quaternion.identity,
            };
            joystickState = new JoystickState
            {
                primary2DAxis = primary2DAxis,
                trigger = trigger,
                grip = grip,
                primaryButton = primaryButton,
            };
            return true;
        }

        return false;
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
        Disconnect();
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
    class PosePayload
    {
        public string type;
        public float timestamp;
        public bool hmdAvailable;
        public float hmdPx;
        public float hmdPy;
        public float hmdPz;
        public float hmdRx;
        public float hmdRy;
        public float hmdRz;
        public float hmdRw;
        public bool wristAvailable;
        public string wristSource;
        public float wristPx;
        public float wristPy;
        public float wristPz;
        public float wristRx;
        public float wristRy;
        public float wristRz;
        public float wristRw;
        public string robotWristFrame;
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
    }
}
