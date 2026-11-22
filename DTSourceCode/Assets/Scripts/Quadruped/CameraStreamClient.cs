using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// WAVEGO 摄像头画面同步客户端。
///
/// 树莓派上运行 camera_server.py (Flask + OpenCV), 通过 5000 端口提供 MJPEG 流:
///   GET /video_feed   multipart/x-mixed-replace, 每帧是一张 JPEG
///   GET /snapshot.jpg 单张 JPEG (调试用)
///
/// 本组件在后台线程手写 HTTP GET 拉流并按 Content-Length 切出每一帧 JPEG,
/// 主线程只负责 Texture2D.LoadImage 与 OnGUI 预览, 网络卡顿不会拖慢 Unity 帧率。
/// </summary>
public class CameraStreamClient : MonoBehaviour
{
    #region 配置

    [Header("摄像头服务 (树莓派 Flask MJPEG)")]
    public string host = "192.168.67.78";
    public int port = 5000;
    public string path = "/video_feed";
    public bool autoConnect = true;

    [Header("预览窗口")]
    public bool showPreview = true;
    public float previewWidth = 320f;
    public float previewHeight = 240f;

    #endregion

    #region 运行时状态

    private TcpClient _client;
    private Thread _thread;
    private volatile bool _running;

    private readonly object _lock = new object();
    private byte[] _pending;
    private int _version;
    private int _applied;
    private string _status = "未连接";

    private Texture2D _texture;
    private int _framesInWindow;
    private float _fpsWindowTime;
    private float _fps;

    public string Status { get { lock (_lock) return _status; } }
    public float Fps { get { return _fps; } }
    public bool HasFrame { get { return _texture != null; } }

    #endregion

    #region 生命周期

    private void Start()
    {
        _fpsWindowTime = Time.unscaledTime;
        if (autoConnect) Connect();
    }

    /// <summary>公开入口, 供需要显式启动的场合使用。</summary>
    public void Connect()
    {
        if (_thread != null) return;
        _running = true;
        _thread = new Thread(StreamLoop);
        _thread.IsBackground = true;
        _thread.Start();
    }

    private void OnDestroy()
    {
        _running = false;
        try { if (_client != null) _client.Close(); } catch (Exception) { }
        try { if (_thread != null) _thread.Join(500); } catch (Exception) { }
        _thread = null;

        if (_texture != null)
        {
            Destroy(_texture);
            _texture = null;
        }
    }

    #endregion

    #region 主线程

    private void Update()
    {
        byte[] data;
        int version;
        lock (_lock)
        {
            data = _pending;
            version = _version;
        }

        if (data != null && version != _applied)
        {
            if (_texture == null)
                _texture = new Texture2D(2, 2, TextureFormat.RGB24, false);

            // 失败也标记为已处理, 避免同一张坏帧每帧重试
            if (_texture.LoadImage(data, false))
            {
                _framesInWindow++;
                if (_applied == 0)
                    Debug.Log($"[CameraStreamClient] 摄像头画面已同步: {_texture.width}x{_texture.height}");
            }
            _applied = version;
        }

        float now = Time.unscaledTime;
        float span = now - _fpsWindowTime;
        if (span >= 1f)
        {
            _fps = _framesInWindow / span;
            _framesInWindow = 0;
            _fpsWindowTime = now;
        }
    }

    #endregion

    #region 后台拉流

    /// <summary>连接 -> 发 HTTP GET -> 循环切出 multipart 中的 JPEG, 断开后自动重连。</summary>
    private void StreamLoop()
    {
        while (_running)
        {
            try
            {
                using (var client = new TcpClient())
                {
                    client.NoDelay = true;
                    client.ReceiveTimeout = 5000;
                    _client = client;
                    client.Connect(host, port);

                    using (Stream stream = client.GetStream())
                    {
                        string request = "GET " + path + " HTTP/1.1\r\n"
                                       + "Host: " + host + ":" + port + "\r\n"
                                       + "Connection: close\r\n\r\n";
                        byte[] bytes = Encoding.ASCII.GetBytes(request);
                        stream.Write(bytes, 0, bytes.Length);

                        // 跳过 HTTP 响应头(状态行 + 空行)
                        string line;
                        do
                        {
                            line = ReadLine(stream);
                            if (line == null) throw new IOException("连接被关闭");
                        } while (line.Length != 0);

                        SetStatus("在线");

                        while (_running)
                        {
                            // 读到空行结束一段 part 头, 期间取 Content-Length
                            int length = -1;
                            do
                            {
                                line = ReadLine(stream);
                                if (line == null) throw new IOException("流结束");

                                if (line.StartsWith("Content-Length:", StringComparison.OrdinalIgnoreCase))
                                    int.TryParse(line.Substring(15).Trim(), out length);
                            } while (line.Length != 0);

                            if (length <= 0) continue;

                            byte[] jpeg = ReadExact(stream, length);
                            if (jpeg == null) throw new IOException("帧不完整");
                            Publish(jpeg);
                        }
                    }
                }
            }
            catch (Exception e)
            {
                SetStatus("断开: " + e.Message);
            }
            finally
            {
                try { if (_client != null) _client.Close(); } catch (Exception) { }
                _client = null;
            }

            // 断线后每 100ms 检查一次退出标志
            for (int i = 0; i < 10 && _running; i++) Thread.Sleep(100);
        }
    }

    private void Publish(byte[] jpeg)
    {
        lock (_lock)
        {
            _pending = jpeg;
            _version++;
        }
    }

    private void SetStatus(string status)
    {
        lock (_lock) _status = status;
    }

    // 逐字节读一行(仅用于很小的 HTTP/multipart 头), 避免缓冲区里残留 JPEG 数据
    private static string ReadLine(Stream stream)
    {
        var sb = new StringBuilder(64);
        while (true)
        {
            int b = stream.ReadByte();
            if (b < 0) return null;
            if (b == '\n')
            {
                string s = sb.ToString();
                return s.EndsWith("\r") ? s.Substring(0, s.Length - 1) : s;
            }
            sb.Append((char)b);
        }
    }

    private static byte[] ReadExact(Stream stream, int count)
    {
        var buffer = new byte[count];
        int offset = 0;
        while (offset < count)
        {
            int n = stream.Read(buffer, offset, count - offset);
            if (n <= 0) return null;
            offset += n;
        }
        return buffer;
    }

    #endregion

    #region 界面

    private void OnGUI()
    {
        if (!showPreview)
        {
            if (GUI.Button(new Rect(Screen.width - 95f, Screen.height - 32f, 80f, 22f), "摄像头"))
                showPreview = true;
            return;
        }

        float w = previewWidth;
        float h = previewHeight;
        var panel = new Rect(Screen.width - w - 15f, Screen.height - h - 50f, w, h + 35f);
        GUI.Box(panel, GUIContent.none);

        string state = HasFrame
            ? $"{_fps:F1} fps"
            : Status;

        GUI.Label(new Rect(panel.x + 6f, panel.y + 4f, w - 70f, 20f), "摄像头 " + state);
        if (GUI.Button(new Rect(panel.xMax - 58f, panel.y + 4f, 52f, 20f), "收起"))
            showPreview = false;

        var imageRect = new Rect(panel.x + 6f, panel.y + 26f, w - 12f, h - 6f);
        if (_texture != null)
            GUI.DrawTexture(imageRect, _texture, ScaleMode.ScaleToFit, false);
        else
            GUI.Label(imageRect, "等待画面…");
    }

    #endregion
}
