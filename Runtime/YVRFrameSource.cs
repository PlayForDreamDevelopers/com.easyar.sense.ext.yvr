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
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.XR;
using YVR.Enterprise.Camera;
using YVR.Core;
using YVR.Utilities;
using UnityEngine.UI;

namespace easyar
{
    /// <summary>
    /// <para xml:lang="en">A custom frame source which connects my device output to EasyAR input in the scene, providing my device support using custom camera feature of EasyAR Sense.</para>
    /// <para xml:lang="en">This frame source is one type of motion tracking device, and will output motion data in a <see cref="ARSession"/>.</para>
    /// <para xml:lang="zh">在场景中将设备的输出连接到EasyAR输入的自定义frame source。通过EasyAR Sense的自定义相机功能提供设备的支持。</para>
    /// <para xml:lang="zh">这个frame source是一种运动跟踪设备，在<see cref="ARSession"/>中会输出运动数据。</para>
    /// </summary>
    public class YVRFrameSource : ExternalDeviceMotionFrameSource
    {
        [SerializeField] private VSTCameraResolutionType m_ResolutionType = VSTCameraResolutionType.VSTResolution660_616;
        [SerializeField] private VSTCameraFrequencyType m_FrequencyType = VSTCameraFrequencyType.VSTFrequency10Hz;
        public  bool passThroughEnabled = true;
        private CameraParameters m_CameraParameters;
        private ARSession m_ARSession;
        private bool m_Opened;
        private bool m_Started;
        private DeviceFrameSourceCamera m_DeviceFrameSourceCamera;
        private InputDevice m_CameraDevice;
        private VSTCameraExtrinsicData m_CameraExtrinsics;
        protected override bool IsHMD => true;

        protected override IDisplay Display => easyar.Display.DefaultHMDDisplay;

        protected override Optional<bool> IsAvailable => true;

        protected override DeviceOriginType OriginType => DeviceOriginType.XROrigin;

        protected override bool CameraFrameStarted => m_Started;

        protected override List<FrameSourceCamera> DeviceCameras => new List<FrameSourceCamera> { m_DeviceFrameSourceCamera };
        private NativeArray<byte> m_DstPacked;
        private void Update()
        {
            if (!m_Started) { return; }

            InputRenderFrameMotionData();
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
            StopAllCoroutines();
            YVRVSTCameraPlugin.UnsubscribeVSTCameraFrameUndistorted();
            m_ARSession = null;
            m_CameraParameters?.Dispose();
            m_CameraParameters = null;
            m_DeviceFrameSourceCamera?.Dispose();
            m_DeviceFrameSourceCamera = null;
            if (m_Opened)
            {
                YVRVSTCameraPlugin.CloseVSTCamera();
                m_Opened = false;
            }
        }

        private IEnumerator InitializeCamera()
        {
            m_CameraDevice = InputDevices.GetDeviceAtXRNode(XRNode.CenterEye);
            yield return new WaitForEndOfFrame();

            if (passThroughEnabled)
            {
                YVRPlugin.Instance.SetPassthrough(true);
            }

            GetResolution(m_ResolutionType,out int imageWidth, out int imageHeight);
            VSTCameraIntrinsicExtrinsicData cameraIntrinsicExtrinsicData = default;
            YVRVSTCameraPlugin.SetVSTCameraResolution(m_ResolutionType);
            YVRVSTCameraPlugin.OpenVSTCamera();
            YVRVSTCameraPlugin.SetVSTCameraFrequency(m_FrequencyType);
            YVRVSTCameraPlugin.SetVSTCameraFormat(VSTCameraFormatType.VSTCameraFmtRGB);
            YVRVSTCameraPlugin.SetVSTCameraOutputSource( VSTCameraSourceType.VSTCameraLeftEye);
            YVRVSTCameraPlugin.GetVSTCameraIntrinsicExtrinsic(VSTCameraSourceType.VSTCameraLeftEye,ref cameraIntrinsicExtrinsicData);
            UndistortionMap undistortionMap = new UndistortionMap(VSTCameraSourceType.VSTCameraLeftEye,m_ResolutionType);
            var focalLength = undistortionMap.focalLength.ToArray();
            var principalPoint = undistortionMap.principalPoint.ToArray();
            var cameraParamList = new List<float> { focalLength[0],focalLength[1],principalPoint[0],principalPoint[1] };
            YVRLog.Debug($"InitializeCamera: cameraParamList: {focalLength[0]},{focalLength[1]},{principalPoint[0]},{principalPoint[1]}");

            var cameraParameters = CameraParameters.tryCreateWithCustomIntrinsics(new Vec2I(imageWidth, imageHeight), cameraParamList,
                CameraModelType.Pinhole, CameraDeviceType.Back, 270);

            if (cameraParameters.OnNone)
                Debug.LogError("cameraParameters none");

            YVRVSTCameraPlugin.GetEyeCenterToVSTCameraExtrinsic(VSTCameraSourceType.VSTCameraLeftEye,ref m_CameraExtrinsics);
            m_CameraParameters = cameraParameters.Value;
            m_Opened = true;
            var frameRateRange = new Vector2(30,30);
            var axisSystem = AxisSystemType.Unity;

            Quaternion quaternion = m_CameraExtrinsics.rotation * new Quaternion(1,0,0,0);
            var extrinsics = new DeviceFrameSourceCamera.CameraExtrinsics(
                new Pose(m_CameraExtrinsics.translation, quaternion),
                true
            );

            YVRLog.Debug($"cameraI extrinsics position:{m_CameraExtrinsics.translation.ToString()}, rotation:{quaternion.ToString()}");
            m_DeviceFrameSourceCamera = new DeviceFrameSourceCamera(CameraDeviceType.Back, 270,
                new Vector2Int(imageWidth, imageHeight), frameRateRange, extrinsics, axisSystem);
            YVRVSTCameraPlugin.SubscribeVSTCameraFrameUndistorted(AcquireVSTCameraFrame);
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

        private void AcquireVSTCameraFrame(VSTCameraFrameData frameData)
        {
            if (!m_Started || m_DeviceFrameSourceCamera == null || m_CameraParameters == null)
            {
                return;
            }

            var frameItem = frameData.cameraFrameItem;
            if (frameItem.data[0] == IntPtr.Zero)
            {
                return;
            }

            var resolution = new Vector2Int(frameItem.width, frameItem.height);
            var dataSize = frameItem.dataSize;
            if (dataSize <= 0)
            {
                return;
            }

            var bufferOptional = TryAcquireBuffer(dataSize);
            if (bufferOptional.OnNone) return;

            var trackingStatus = frameData.sixDofPose.confidence == 1 ? MotionTrackingStatus.Tracking : MotionTrackingStatus.NotTracking;
            m_CameraDevice.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
            m_CameraDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);
            Pose pose = new Pose(position, rotation);
            var timestamp = frameItem.soeTimestamp;

            if (!m_DstPacked.IsCreated)
            {
                m_DstPacked =  new NativeArray<byte>(frameItem.width*frameItem.height*3, Allocator.Persistent);
            }
            CopyToPacked(frameItem.data[0],frameItem.width,frameItem. height, frameItem.stride, m_DstPacked);
            using (var buffer = bufferOptional.Value)
            {
                IntPtr intPtr = IntPtr.Zero;
                unsafe
                {
                    void* ptr = m_DstPacked.GetUnsafeReadOnlyPtr();
                    intPtr = (IntPtr)ptr;
                    buffer.tryCopyFrom(intPtr, 0, 0, dataSize);
                    m_DstPacked.Dispose();
                }
                using (var image = Image.create(buffer, PixelFormat.RGB888, frameItem.width, frameItem.height, resolution.x, resolution.y))
                {
                    HandleCameraFrameData(m_DeviceFrameSourceCamera, timestamp * 1e-9, image, m_CameraParameters, pose, trackingStatus);
                }
            }
        }

        private static unsafe void CopyToPacked(
            IntPtr basePtr,
            int width, int height,
            int stridePixels,
            NativeArray<byte> packedDst)
        {
            int rowBytes = width * 3;
            int packedBytes = rowBytes * height;

            if (!packedDst.IsCreated || packedDst.Length < packedBytes)
                throw new ArgumentException("packedDst too small");

            int srcStrideBytes = stridePixels * 3;

            byte* srcBase = (byte*)basePtr.ToPointer();
            byte* dstBase = (byte*)NativeArrayUnsafeUtility.GetUnsafePtr(packedDst);

            for (int y = 0; y < height; y++)
            {
                byte* srcRow = srcBase + y * srcStrideBytes;
                byte* dstRow = dstBase + y * rowBytes;

                UnsafeUtility.MemCpy(dstRow, srcRow, rowBytes);
            }
        }
        private void InputRenderFrameMotionData()
        {
            long timestamp = YVRGetPredictedTime();
            m_CameraDevice.TryGetFeatureValue(CommonUsages.devicePosition, out var position);
            m_CameraDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out var rotation);
            Pose pose = new Pose(position, rotation);
            HandleRenderFrameData(timestamp * 1e-9, pose, MotionTrackingStatus.Tracking);
        }

        [DllImport("yvrplugin")]
        private static extern long YVRGetPredictedTime();

    }
}
#endif
