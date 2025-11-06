//================================================================================================================================
//
//  Copyright (c) 2015-2025 VisionStar Information Technology (Shanghai) Co., Ltd. All Rights Reserved.
//  EasyAR is the registered trademark or trademark of VisionStar Information Technology (Shanghai) Co., Ltd in China
//  and other countries for the augmented reality technology developed by VisionStar Information Technology (Shanghai) Co., Ltd.
//
//================================================================================================================================

#if !EASYAR_HAVE_SENSE
#error Require EasyAR Sense Unity Plugin (com.easyar.sense) to be imported!
#elif !EASYAR_ENABLE_SENSE
#error Incompatible EasyAR Sense Unity Plugin detected, require EasyAR Sense Unity Plugin >= 4000.0.0!
#elif !EASYAR_HAVE_YVR
#error Require YVR Unity  SDK (com.yvr.core) to be imported!
#elif !EASYAR_ENABLE_YVR
#error Incompatible YVR SDK detected, require YVR Core SDK >= 1.25.1
#else

using System;
using System.Buffers;
using System.Collections;
<<<<<<< HEAD
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Serialization;
=======
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Jobs;
using UnityEngine;
>>>>>>> d38df90 (temp)
using UnityEngine.UI;
using UnityEngine.XR;
using YVR.Enterprise.Camera;
using YVR.Core;
using YVR.Utilities;

namespace easyar
{
    /// <summary>
    /// <para xml:lang="en">A custom frame source which connects my device output to EasyAR input in the scene, providing my device support using custom camera feature of EasyAR Sense.</para>
    /// <para xml:lang="en">This frame source is one type of motion tracking device, and will output motion data in a <see cref="ARSession"/>.</para>
<<<<<<< HEAD
    /// <para xml:lang="zh">在场景中将设备的输出连接到EasyAR输入的自定义frame source。通过EasyAR Sense的自定义相机功能提供设备的支持。</para>
=======
    /// <para xml:lang="zh">在场景中将我的设备的输出连接到EasyAR输入的自定义frame source。通过EasyAR Sense的自定义相机功能提供我的设备的支持。</para>
>>>>>>> d38df90 (temp)
    /// <para xml:lang="zh">这个frame source是一种运动跟踪设备，在<see cref="ARSession"/>中会输出运动数据。</para>
    /// </summary>
    public class YVRFrameSource : ExternalDeviceMotionFrameSource
    {
<<<<<<< HEAD
        [SerializeField] private VSTCameraResolutionType m_ResolutionType = VSTCameraResolutionType.VSTResolution660_616;
        [SerializeField] private VSTCameraFrequencyType m_FrequencyType = VSTCameraFrequencyType.VSTFrequency10Hz;
=======
        public RawImage cameraImage;
>>>>>>> d38df90 (temp)
        public  bool passThroughEnabled = true;
        private CameraParameters m_CameraParameters;
        private ARSession m_ARSession;
        private bool m_Opened;
<<<<<<< HEAD
        private bool m_Started;
        private DeviceFrameSourceCamera m_DeviceFrameSourceCamera;
        private InputDevice m_CameraDevice;
        private readonly ConcurrentQueue<FrameProcessingRequest> m_PendingCameraFrames = new ConcurrentQueue<FrameProcessingRequest>();
        private CancellationTokenSource m_FrameProcessingCts;
        private Task m_FrameProcessingTask;
        private SemaphoreSlim m_FrameAvailableSignal;
        private VSTCameraExtrinsicData m_CameraExtrinsics;
=======
        private long m_CurTimestamp;
        private bool m_Started;
        private DeviceFrameSourceCamera m_DeviceFrameSourceCamera;
        private InputDevice cameraDevice;
>>>>>>> d38df90 (temp)
        protected override bool IsHMD => true;

        protected override IDisplay Display => easyar.Display.DefaultHMDDisplay;

        protected override Optional<bool> IsAvailable => true;

        protected override DeviceOriginType OriginType => DeviceOriginType.XROrigin;

        protected override bool CameraFrameStarted => m_Started;

        protected override List<FrameSourceCamera> DeviceCameras => new List<FrameSourceCamera> { m_DeviceFrameSourceCamera };

        private UndistortionFrameWrapper m_UndistortionFrameWrapper;

<<<<<<< HEAD
        private static byte[] s_CacheFrameBytes;

        private struct FrameProcessingRequest
        {
            public byte[] Nv21Data;
            public Vector2Int Resolution;
            public long Timestamp;
            public Pose Pose;
            public MotionTrackingStatus TrackingStatus;
        }

=======
        private bool m_GenerateTexture = false;
>>>>>>> d38df90 (temp)
        private void Update()
        {
            if (!m_Started) { return; }

            InputRenderFrameMotionData();
<<<<<<< HEAD
=======
            if (YVRInput.GetDown(YVRInput.RawButton.A))
            {
                m_GenerateTexture = !m_GenerateTexture;
            }
>>>>>>> d38df90 (temp)
        }

        protected override void OnSessionStart(ARSession session)
        {
            base.OnSessionStart(session);
            this.m_ARSession = session;
            StartCoroutine(InitializeCamera());
        }

        protected override void OnSessionStop()
        {
            base.OnSessionStop();
            m_Started = false;
<<<<<<< HEAD
            StopFrameProcessingTask();
=======
>>>>>>> d38df90 (temp)
            StopAllCoroutines();
            m_ARSession = null;
            m_CameraParameters?.Dispose();
            m_CameraParameters = null;
            m_DeviceFrameSourceCamera?.Dispose();
            m_DeviceFrameSourceCamera = null;
            if (m_Opened)
            {
                YVRVSTCameraPlugin.CloseVSTCamera();
            }
        }

        private IEnumerator InitializeCamera()
        {
<<<<<<< HEAD
            m_CameraDevice = InputDevices.GetDeviceAtXRNode(XRNode.CenterEye);
=======
            Debug.Log($"InitializeCamera");
            cameraDevice = InputDevices.GetDeviceAtXRNode(XRNode.CenterEye);
>>>>>>> d38df90 (temp)
            yield return new WaitForEndOfFrame();

            if (passThroughEnabled)
            {
                YVRPlugin.Instance.SetPassthrough(true);
<<<<<<< HEAD
            }

            GetResolution(m_ResolutionType,out int imageWidth, out int imageHeight);
            VSTCameraIntrinsicExtrinsicData cameraIntrinsicExtrinsicData = default;
            YVRVSTCameraPlugin.OpenVSTCamera();
            YVRVSTCameraPlugin.SetVSTCameraFrequency(m_FrequencyType);
            YVRVSTCameraPlugin.SetVSTCameraResolution(m_ResolutionType);
            YVRVSTCameraPlugin.SetVSTCameraFormat(VSTCameraFormatType.VSTCameraFmtNv21);
            YVRVSTCameraPlugin.SetVSTCameraOutputSource( VSTCameraSourceType.VSTCameraLeftEye);
            YVRVSTCameraPlugin.GetVSTCameraIntrinsicExtrinsic(VSTCameraSourceType.VSTCameraLeftEye,ref cameraIntrinsicExtrinsicData);
            m_UndistortionFrameWrapper = new UndistortionFrameWrapper(m_ResolutionType,VSTCameraSourceType.VSTCameraLeftEye,imageWidth,imageHeight);
            var focalLength = m_UndistortionFrameWrapper.undistortionMap.focalLength.ToArray();
            var principalPoint = m_UndistortionFrameWrapper.undistortionMap.principalPoint.ToArray();
            var cameraParamList = new List<float> { focalLength[0],focalLength[1],principalPoint[0],principalPoint[1] };
            YVRLog.Debug($"InitializeCamera: cameraParamList: {focalLength[0]},{focalLength[1]},{principalPoint[0]},{principalPoint[1]}");

            var cameraParameters = CameraParameters.tryCreateWithCustomIntrinsics(new Vec2I(imageWidth, imageHeight), cameraParamList,
                CameraModelType.Pinhole, CameraDeviceType.Back, 270);

            if (cameraParameters.OnNone)
                Debug.LogError("cameraParameters none");

            YVRVSTCameraPlugin.GetEyeCenterToVSTCameraExtrinsic(VSTCameraSourceType.VSTCameraLeftEye,ref m_CameraExtrinsics);
            m_CameraParameters = cameraParameters.Value;
            m_Opened = true;
            var frameRateRange = new Vector2(8,30);
            var axisSystem = AxisSystemType.Unity;

            var offset = m_CameraExtrinsics.translation;
            Quaternion quaternion = new Quaternion( -0.00081f, 0.07177f, -0.70294f, -0.70762f);
            var extrinsics = new DeviceFrameSourceCamera.CameraExtrinsics(
                new Pose(offset, quaternion),
                true
            );
            YVRLog.Debug($"cameraI extrinsics position:{offset.ToString()}, rotation:{quaternion.ToString()}");

            m_DeviceFrameSourceCamera = new DeviceFrameSourceCamera(CameraDeviceType.Back, 270,
                new Vector2Int(imageWidth, imageHeight), frameRateRange, extrinsics, axisSystem);
            s_CacheFrameBytes = new byte[imageWidth*imageHeight*3];
            StartFrameProcessingTask();
=======
                Debug.Log($"InitializeCamera SetPassthrough enable");
            }

            VSTCameraResolutionType resolutionType = VSTCameraResolutionType.VSTResolution1320_1232;
            GetResolution(resolutionType,out int imageWidth, out int imageHeight);
            VSTCameraIntrinsicExtrinsicData cameraIntrinsicExtrinsicData = default;
            YVRVSTCameraPlugin.OpenVSTCamera();
            YVRVSTCameraPlugin.SetVSTCameraFrequency(VSTCameraFrequencyType.VSTFrequency30Hz);
            YVRVSTCameraPlugin.SetVSTCameraResolution(resolutionType);
            YVRVSTCameraPlugin.SetVSTCameraFormat(VSTCameraFormatType.VSTCameraFmtNv21);
            YVRVSTCameraPlugin.SetVSTCameraOutputSource( VSTCameraSourceType.VSTCameraLeftEye);
            YVRVSTCameraPlugin.GetVSTCameraIntrinsicExtrinsic(VSTCameraSourceType.VSTCameraLeftEye,ref cameraIntrinsicExtrinsicData);
            m_UndistortionFrameWrapper = new UndistortionFrameWrapper(resolutionType,VSTCameraSourceType.VSTCameraLeftEye,imageWidth,imageHeight);
            var focalLength = m_UndistortionFrameWrapper.undistortionMap.focalLength.ToArray();
            var principalPoint = m_UndistortionFrameWrapper.undistortionMap.principalPoint.ToArray();
            var cameraParamList = new List<float> { focalLength[0],focalLength[1],principalPoint[0],principalPoint[1] };
            Debug.Log($"InitializeCamera: cameraParamList: {focalLength[0]},{focalLength[1]},{principalPoint[0]},{principalPoint[1]}");
            var cameraParameters = CameraParameters.tryCreateWithCustomIntrinsics(new Vec2I(imageHeight, imageWidth), cameraParamList,
                CameraModelType.Pinhole, CameraDeviceType.Back, 0);

            if (cameraParameters.OnNone)
                Debug.Log("cameraParameters none");

            m_CameraParameters = cameraParameters.Value;
            m_Opened = true;
            var frameRateRange = new Vector2(30,30);
            var axisSystem = AxisSystemType.Unity;
            VSTCameraExtrinsicData cameraExtrinsicData = default;
            YVRVSTCameraPlugin.GetEyeCenterToVSTCameraExtrinsic(VSTCameraSourceType.VSTCameraLeftEye, ref cameraExtrinsicData);
            var extrinsics = new DeviceFrameSourceCamera.CameraExtrinsics(new Pose(cameraExtrinsicData.translation,cameraExtrinsicData.rotation), true);
            Debug.Log($"cameraExtrinsicData:{cameraExtrinsicData.translation},{cameraExtrinsicData.rotation}");
            m_DeviceFrameSourceCamera = new DeviceFrameSourceCamera(CameraDeviceType.Back, 0,
                new Vector2Int(imageHeight, imageWidth), frameRateRange, extrinsics, axisSystem);
>>>>>>> d38df90 (temp)
            YVRVSTCameraPlugin.SubscribeVSTCameraFrame(AcquireVSTCameraFrame);
            m_Started = true;
        }

        private static void GetResolution(VSTCameraResolutionType resolution, out int width, out int height)
        {
            switch (resolution)
            {
                case VSTCameraResolutionType.VSTResolution660_616:
                    width = 660;
                    height = 616;
                    break;
                case VSTCameraResolutionType.VSTResolution1320_1232:
                    width = 1320;
                    height = 1232;
                    break;
                case VSTCameraResolutionType.VSTResolution2640_2464:
                    width = 2640;
                    height = 2464;
                    break;
                default:
                    width = 660;
                    height = 616;
                    break;
            }
        }
<<<<<<< HEAD

        private void AcquireVSTCameraFrame(VSTCameraFrameData frameData)
        {
            if (!m_Started || m_FrameProcessingCts == null || m_FrameAvailableSignal == null)
            {
                return;
            }

            if (frameData.cameraFrameItem.data[0] == IntPtr.Zero)
            {
                return;
            }

            var size = new Vector2Int(frameData.cameraFrameItem.width, frameData.cameraFrameItem.height);
            try
            {
                Marshal.Copy(frameData.cameraFrameItem.data[0], s_CacheFrameBytes, 0, s_CacheFrameBytes.Length);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"YVRFrameSource failed to copy VST frame: {ex}");
                return;
            }

            var trackingStatus = frameData.sixDofPose.confidence == 1 ? MotionTrackingStatus.Tracking : MotionTrackingStatus.NotTracking;
            m_CameraDevice.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
            m_CameraDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);
            Pose pose = new Pose(position, rotation);
            var timestamp = frameData.cameraFrameItem.soeTimestamp;
            var request = new FrameProcessingRequest
            {
                Nv21Data = s_CacheFrameBytes,
                Resolution = size,
                Timestamp = timestamp,
                Pose = pose,
                TrackingStatus = trackingStatus
            };

            // Only keep the newest frame to avoid processing backlog.
            while (m_PendingCameraFrames.TryDequeue(out _)) { }
            var shouldSignal = m_FrameAvailableSignal.CurrentCount == 0;
            m_PendingCameraFrames.Enqueue(request);
            if (shouldSignal)
            {
                m_FrameAvailableSignal.Release();
=======
        private static ArrayPool<byte> s_ArrayPool = ArrayPool<byte>.Shared;
        private byte[] cacheframeBytes;

        /// <summary>
        /// Flip image data vertically for EasyAR coordinate system
        /// </summary>
        private byte[] FlipImageForEasyAR(byte[] sourceData, int width, int height, int bytesPerPixel = 3)
        {
            byte[] flippedData = new byte[sourceData.Length];

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // Calculate source position (flip only vertically)
                    int srcY = height - 1 - y;
                    int srcX = x;  // Keep horizontal position the same
                    int srcIndex = (srcY * width + srcX) * bytesPerPixel;

                    // Calculate destination position
                    int dstIndex = (y * width + x) * bytesPerPixel;

                    // Copy pixel data (RGB)
                    for (int b = 0; b < bytesPerPixel; b++)
                    {
                        flippedData[dstIndex + b] = sourceData[srcIndex + b];
                    }
                }
            }

            return flippedData;
        }

        private void AcquireVSTCameraFrame(VSTCameraFrameData frameData)
        {
            Debug.Log($"AcquireVSTCameraFrame:{frameData.ToString()}");
            if (frameData.cameraFrameItem.data[0] == IntPtr.Zero) return;
            var size = new Vector2Int(frameData.cameraFrameItem.width, frameData.cameraFrameItem.height);
            var bufferSize = size.x * size.y * 3;

            if (frameData.cameraFrameItem.data[0] != IntPtr.Zero)
            {
                var bufferO = TryAcquireBuffer(bufferSize);
                if (bufferO.OnNone) { return; }

                if (cacheframeBytes != null)
                    s_ArrayPool.Return(cacheframeBytes);

                cacheframeBytes = s_ArrayPool.Rent(frameData.cameraFrameItem.dataSize);
                Marshal.Copy(frameData.cameraFrameItem.data[0], cacheframeBytes, 0, frameData.cameraFrameItem.dataSize);
                var rgbBuffer = m_UndistortionFrameWrapper.ConvertRGBData(cacheframeBytes);

                // Flip image for EasyAR (EasyAR coordinate system is different from Unity's)
                var rgbBufferForEasyAR = FlipImageForEasyAR(rgbBuffer, size.y, size.x, 3);

                UIThreadDispatcher.instance.Enqueue(() =>
                {
                    if (cameraImage.texture != null)
                    {
                        Destroy(cameraImage.texture);
                    }
                    if (m_GenerateTexture)
                    {
                        Texture2D texture = new Texture2D(size.y, size.x, TextureFormat.RGB24, false);
                        texture.LoadRawTextureData(rgbBuffer);
                        texture.Apply();
                        cameraImage.texture = texture;
                    }
                });
                m_CurTimestamp = frameData.cameraFrameItem.soeTimestamp;
                var trackingStatus = frameData.sixDofPose.confidence == 1 ? MotionTrackingStatus.Tracking: MotionTrackingStatus.NotTracking;
                // Pose pose = GetHeadPose(frameData.sixDofPose.timestamp);
                cameraDevice.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
                cameraDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);
                Pose pose = new Pose(position, rotation);
                Debug.Log($"FramePose:{pose.position},{pose.rotation},timestamp:{m_CurTimestamp * 1e-9}");
                // Pose pose = new Pose();
                // pose.position = new Vector3((float)frameData.sixDofPose.x,(float)frameData.sixDofPose.y,(float)frameData.sixDofPose.z).FromFlippedZVector3f();
                // pose.rotation = new Quaternion((float)frameData.sixDofPose.rx,(float)frameData.sixDofPose.ry,(float)frameData.sixDofPose.rz,(float)frameData.sixDofPose.rw).FromFlippedZQuatf();
                var buffer = bufferO.Value;
                buffer.copyFromByteArray(rgbBufferForEasyAR,0,0,bufferSize);
                using (buffer)
                using (var image = Image.create(buffer, PixelFormat.RGB888, size.y, size.x, size.y, size.x))
                {
                    HandleCameraFrameData(m_DeviceFrameSourceCamera, m_CurTimestamp*1e-9, image, m_CameraParameters, pose, trackingStatus);
                }
>>>>>>> d38df90 (temp)
            }
        }

        private void InputRenderFrameMotionData()
        {
            long timestamp = YVRGetPredictedTime();
<<<<<<< HEAD
            m_CameraDevice.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
            m_CameraDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);
            Pose pose = new Pose(position, rotation);
            HandleRenderFrameData(timestamp * 1e-9, pose, MotionTrackingStatus.Tracking);
        }

        [DllImport("yvrplugin")]
        private static extern long YVRGetPredictedTime();

        private void StartFrameProcessingTask()
        {
            StopFrameProcessingTask();
            m_FrameProcessingCts = new CancellationTokenSource();
            m_FrameAvailableSignal = new SemaphoreSlim(0);
            m_FrameProcessingTask = Task.Run(() => FrameProcessingLoop(m_FrameProcessingCts.Token), m_FrameProcessingCts.Token);
        }

        private void StopFrameProcessingTask()
        {
            var cts = m_FrameProcessingCts;
            if (cts == null)
            {
                return;
            }

            try
            {
                cts.Cancel();
                try
                {
                    m_FrameAvailableSignal?.Release();
                }
                catch (ObjectDisposedException)
                {
                }
                m_FrameProcessingTask?.Wait();
            }
            catch (AggregateException ex)
            {
                ex.Handle(e => e is OperationCanceledException);
            }
            finally
            {
                cts.Dispose();
                m_FrameProcessingCts = null;

                m_FrameAvailableSignal?.Dispose();
                m_FrameAvailableSignal = null;

                m_FrameProcessingTask = null;
            }
        }

        private void FrameProcessingLoop(CancellationToken token)
        {
            try
            {
                while (true)
                {
                    token.ThrowIfCancellationRequested();
                    m_FrameAvailableSignal.Wait(token);

                    while (m_PendingCameraFrames.TryDequeue(out var frame))
                    {
                        token.ThrowIfCancellationRequested();
                        try
                        {
                            ProcessCameraFrame(frame, token);
                        }
                        catch (OperationCanceledException)
                        {
                            throw;
                        }
                        catch (Exception ex)
                        {
                            Debug.LogWarning($"YVRFrameSource frame task failed: {ex}");
                        }
                    }
                }
            }
            catch (OperationCanceledException)
            {
                Debug.LogError("FrameProcessingLoop Error");
            }
        }

        private void ProcessCameraFrame(FrameProcessingRequest frame, CancellationToken token)
        {
            if (m_UndistortionFrameWrapper == null || m_DeviceFrameSourceCamera == null || m_CameraParameters == null)
            {
                return;
            }

            token.ThrowIfCancellationRequested();

            var bufferSize = frame.Resolution.x * frame.Resolution.y * 3;
            var bufferOptional = TryAcquireBuffer(bufferSize);
            if (bufferOptional.OnNone)
            {
                return;
            }

            using (var buffer = bufferOptional.Value)
            {
                var rgbBuffer = m_UndistortionFrameWrapper.ConvertRGBData(frame.Nv21Data, token);
                if (rgbBuffer == null)
                {
                    return;
                }

                token.ThrowIfCancellationRequested();

                buffer.copyFromByteArray(rgbBuffer, 0, 0, bufferSize);
                using (var image = Image.create(buffer, PixelFormat.RGB888, frame.Resolution.x, frame.Resolution.y, frame.Resolution.x, frame.Resolution.y))
                {
                    HandleCameraFrameData(m_DeviceFrameSourceCamera, frame.Timestamp * 1e-9, image, m_CameraParameters, frame.Pose, frame.TrackingStatus);
                }
            }
        }
=======
            // SixDofPoseData poseData = default;
            // YVRVSTCameraPlugin.GetHeadPose(timestamp,ref poseData);
            // Pose pose = GetHeadPose(timestamp);
            cameraDevice.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
            cameraDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);
            Pose pose = new Pose(position, rotation);
            Debug.Log($"UpdatePose:{pose.position},{pose.rotation},timestamp:{timestamp * 1e-9}");
            // Debug.Log($"timestamp:{timestamp}");
            HandleRenderFrameData(timestamp * 1e-9, pose, MotionTrackingStatus.Tracking);
        }

        private Pose GetHeadPose(long timestamp)
        {
            Debug.Log($"timestamp:{timestamp}");
            XRPose xrPose =  YVRGetPredictedTimeHeadPose(timestamp);
            Pose pose = new Pose();
            pose. position= xrPose.position.FromFlippedZVector3f();
            pose.rotation = xrPose.orientation.FromFlippedZQuatf();
            return pose;
        }

        [DllImport("yvrplugin")]
        private static extern long YVRGetPredictedTime();

        [DllImport("yvrplugin")]
        private static extern XRPose YVRGetPredictedTimeHeadPose(long timestamp);
>>>>>>> d38df90 (temp)
    }
}
#endif
