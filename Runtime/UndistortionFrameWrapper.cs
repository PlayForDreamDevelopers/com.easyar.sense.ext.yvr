using System;
<<<<<<< HEAD
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
=======
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
>>>>>>> d38df90 (temp)
using YVR.Enterprise.Camera;

public class UndistortionFrameWrapper
{
<<<<<<< HEAD
    public UndistortionMap undistortionMap = null;

    private readonly int m_SourceWidth;
    private readonly int m_SourceHeight;
    private readonly int m_PixelCount;
    private readonly byte[] m_RgbBuffer;
    private readonly byte[] m_UndistortedBuffer;
    private readonly float[] m_MapX;
    private readonly float[] m_MapY;

    public UndistortionFrameWrapper(VSTCameraResolutionType resolutionType, VSTCameraSourceType sourceType, int width, int height)
    {
        // 原始 NV21 数据的宽高
        m_SourceWidth = width;
        m_SourceHeight = height;
        m_PixelCount = width * height;

        m_RgbBuffer = new byte[m_PixelCount * 3];
        m_UndistortedBuffer = new byte[m_PixelCount * 3];

        undistortionMap = new UndistortionMap(sourceType, resolutionType);
        m_MapX = undistortionMap.xDataArray.ToArray();
        m_MapY = undistortionMap.yDataArray.ToArray();
    }

    public byte[] ConvertRGBData(byte[] data, CancellationToken token)
    {
        if (data == null || data.Length == 0 || token.IsCancellationRequested)
        {
            return null;
        }

        ConvertNV21ToRgb(data, m_RgbBuffer, token);
        ApplyUndistortion(m_RgbBuffer, m_UndistortedBuffer, token);
        return m_UndistortedBuffer;
    }

    private void ConvertNV21ToRgb(byte[] nv21Data, byte[] rgbData, CancellationToken token)
    {
        int width = m_SourceWidth;
        int height = m_SourceHeight;
        int frameSize = width * height;
        int uvStride = (width + 1) & ~1;

        ExecuteInParallel(height, token, (startY, endY) =>
        {
            for (int y = startY; y < endY; y++)
            {
                int yRowStart = y * width;
                int uvRow = (y >> 1) * uvStride;
                for (int x = 0; x < width; x++)
                {
                    int yIndex = yRowStart + x;
                    int uvIndex = frameSize + uvRow + (x & ~1);
                    float yValue = ClampFloat(nv21Data[yIndex], 16f, 235f) - 16f;
                    float v = nv21Data[uvIndex] - 128f;
                    float u = nv21Data[uvIndex + 1] - 128f;

                    float r = 1.164f * yValue + 1.596f * v;
                    float g = 1.164f * yValue - 0.391f * u - 0.813f * v;
                    float b = 1.164f * yValue + 2.018f * u;

                    int rgbIndex = yIndex * 3;
                    rgbData[rgbIndex] = (byte)ClampToByte(r);
                    rgbData[rgbIndex + 1] = (byte)ClampToByte(g);
                    rgbData[rgbIndex + 2] = (byte)ClampToByte(b);
                }
            }
        });
    }

    private void ApplyUndistortion(byte[] srcRgb, byte[] dstRgb, CancellationToken token)
    {
        int width = m_SourceWidth;
        int height = m_SourceHeight;
        float[] mapX = m_MapX;
        float[] mapY = m_MapY;

        ExecuteInParallel(m_PixelCount, token, (startIdx, endIdx) =>
        {
            for (int idx = startIdx; idx < endIdx; idx++)
            {
                float srcX = mapX[idx];
                float srcY = mapY[idx];

                int x0 = (int)Math.Floor(srcX);
                int y0 = (int)Math.Floor(srcY);
                int x1 = x0 + 1;
                int y1 = y0 + 1;
                float dx = srcX - x0;
                float dy = srcY - y0;

                int dstIndex = idx * 3;
                for (int c = 0; c < 3; c++)
                {
                    float v00 = SampleRgb(srcRgb, width, height, x0, y0, c);
                    float v10 = SampleRgb(srcRgb, width, height, x1, y0, c);
                    float v01 = SampleRgb(srcRgb, width, height, x0, y1, c);
                    float v11 = SampleRgb(srcRgb, width, height, x1, y1, c);

                    float v0 = Lerp(v00, v10, dx);
                    float v1 = Lerp(v01, v11, dx);
                    float value = Lerp(v0, v1, dy);
                    dstRgb[dstIndex + c] = (byte)ClampToByte(value);
                }
            }
        });
    }

    private void ExecuteInParallel(int workSize, CancellationToken token, Action<int, int> worker)
    {
        if (workSize <= 0)
        {
            return;
        }

        int workerCount = Math.Max(1, Math.Min(workSize, Environment.ProcessorCount));
        if (workerCount == 1)
        {
            worker(0, workSize);
            return;
        }

        int chunkSize = Math.Max(1, (int)Math.Ceiling(workSize / (double)workerCount));
        var tasks = new List<Task>(workerCount);

        for (int start = 0; start < workSize; start += chunkSize)
        {
            int rangeStart = start;
            int rangeEnd = Math.Min(workSize, rangeStart + chunkSize);
            tasks.Add(Task.Run(() =>
            {
                token.ThrowIfCancellationRequested();
                worker(rangeStart, rangeEnd);
            }, token));
        }

        try
        {
            Task.WaitAll(tasks.ToArray());
        }
        catch (AggregateException ex)
        {
            bool allCanceled = true;
            foreach (var inner in ex.InnerExceptions)
            {
                if (!(inner is OperationCanceledException))
                {
                    allCanceled = false;
                    break;
                }
            }

            if (allCanceled)
            {
                throw new OperationCanceledException(token);
            }

            throw;
        }
    }


    private static float SampleRgb(byte[] data, int width, int height, int x, int y, int channel)
    {
        if (x < 0 || x >= width || y < 0 || y >= height)
        {
            return 0f;
        }

        int index = (y * width + x) * 3 + channel;
        return data[index];
    }

    private static float Lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    private static float ClampFloat(float value, float min, float max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    private static int ClampToByte(float value)
    {
        if (value < 0f) return 0;
        if (value > 255f) return 255;
        return (int)Math.Round(value);
    }
}
=======
    public NV21DataConverter NV21DataConverter = null;
    public UndistortionMap undistortionMap = null;

    public UndistortionFrameWrapper(VSTCameraResolutionType resolutionType, VSTCameraSourceType sourceType, int width, int height)
    {
        // As the image is rotated 90 degrees, the width and height of image are swapped
        NV21DataConverter = new NV21DataConverter(width, height);
        undistortionMap = new UndistortionMap(sourceType,
            resolutionType);
    }

    public byte[] ConvertRGBData(byte[]  data)
    {
        if (data == null) return null;

        using NativeArray<byte> nv21NativeLeft =  new NativeArray<byte>(data, Allocator.Persistent);
        JobHandle distortionJobHandle = NV21DataConverter.GetNormalizeRGBDataJobHandle(nv21NativeLeft, undistortionMap);
        distortionJobHandle.Complete();
        return NV21DataConverter.normalizedRGBDataArray.ToArray();
    }

    private static NativeArray<byte> IntPtrToNativeArray(IntPtr ptr, int length)
    {
        unsafe
        {
            NativeArray<byte> arr = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<byte>(
                (void*)ptr, length, Allocator.None);
            return arr;
        }
    }
}
>>>>>>> d38df90 (temp)
