using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// WAVEGO 实机网关客户端。
///
/// 实机侧(Pi 192.168.67.78, 与 PC 同一 WiFi 时由路由器分配)运行 wavego_pi_gateway.py:
///   9000  TCP  接收一行一条 JSON 命令, 形如 {"var":"move","val":1}
///   9001  TCP  推送一行一条遥测 JSON, 含 12 个舵机角/IMU/电压
///
/// 本组件负责:
///   1. 后台线程持续读取 9001 遥测, 解析后缓存最新一帧
///   2. 后台线程把控制命令排队写入 9000 (主线程不做任何 socket 连接, 避免卡顿)
///   3. 可选的"实机 -> 孪生体"姿态镜像(由足端位置反算五杆闭链关节)
///
/// 命令 var/val 取值来自 ESP32 固件 WAVEGO.ino:
///   move: 1 前进, 2 左转, 3 停止前进, 4 右转, 5 后退, 6 停止转向
///   ges:  1/2 抬头低头, 4/5 侧倾, 3/6 复位
///   light/buzzer: 灯光与蜂鸣器
/// </summary>
public class WavegoClient : MonoBehaviour
{
    #region 配置

    [Header("网关地址")]
    public string host = "192.168.67.78";
    public int controlPort = 9000;
    public int telemetryPort = 9001;
    [Tooltip("遥测轮询间隔(ms), 9001 需要主动请求才回帧")]
    public int telemetryPollMs = 50;
    public bool autoConnect = true;

    // 面板上的地址输入框内容, 便于现场改 IP 而不用改代码
    private string _hostEdit;

    [Header("遥控 (Play 时可用, 点击按钮即发送)")]
    public bool remoteEnabled = true;

    [Header("实机姿态镜像到孪生体")]
    public bool mirrorToTwin = true;

    #endregion

    #region 遥测数据结构 (字段名必须与网关 JSON 一致)

    [Serializable]
    public class WavegoState
    {
        public float vol;
        public float servo0, servo1, servo2, servo3, servo4, servo5;
        public float servo6, servo7, servo8, servo9, servo10, servo11;
        public float ax, ay, az;
        public float gx, gy, gz;
        public float mx, my, mz;
        public float roll, pitch, yaw;
    }

    [Serializable]
    public class WavegoPacket
    {
        public string type;
        public float t;
        public float x;
        public float z;
        public WavegoState state;
    }

    #endregion

    #region 运行时状态

    private readonly object _lock = new object();
    private readonly float[] _servo = new float[12];
    private readonly float[] _windowServo = new float[12];
    private bool _hasState;
    private DateTime _lastPacketUtc;
    private DateTime _lastMotionUtc;
    private DateTime _windowUtc;
    private float _vol, _roll, _pitch, _yaw, _x, _z;
    private int _packetCount;

    // 遥测帧率统计: ESP32 空闲时可能十几秒才推一帧, 这种稀疏数据不能拿来镜像姿态
    private int _framesInWindow;
    private float _telemetryHz;
    private DateTime _rateWindowUtc = DateTime.MinValue;

    // 面板/遥控发出的运动命令, 供本地步态在实机未连接或静止时复用
    private int _commandedFB;
    private int _commandedLR;

    private TcpClient _telemetryClient;
    private Thread _telemetryThread;
    private StreamReader _telemetryReader;
    private TcpClient _controlClient;
    private StreamWriter _controlWriter;
    private Thread _controlThread;
    private readonly Queue<string> _sendQueue = new Queue<string>();
    private readonly object _controlLock = new object();
    private DateTime _nextControlAttemptUtc = DateTime.MinValue;
    private volatile bool _running;
    private string _status = "未连接";

    public bool HasState { get { lock (_lock) return _hasState; } }
    public bool IsConnected { get { lock (_lock) return _hasState; } }
    public string Status { get { lock (_lock) return _status; } }

    /// <summary>最近 seconds 秒内是否收到过遥测帧。</summary>
    public bool TelemetryFresh(float seconds)
    {
        lock (_lock)
        {
            return _hasState && (DateTime.UtcNow - _lastPacketUtc).TotalSeconds <= seconds;
        }
    }

    /// <summary>实机舵机角是否仍在明显变化(即实机确实在运动)。</summary>
    public bool IsMoving
    {
        get
        {
            lock (_lock)
            {
                return _hasState && (DateTime.UtcNow - _lastMotionUtc).TotalSeconds < 0.6;
            }
        }
    }

    /// <summary>最近统计到的遥测帧率(Hz)。</summary>
    public float TelemetryHz { get { lock (_lock) return _telemetryHz; } }

    /// <summary>
    /// 遥测是否在持续推流。ESP32 空闲时约十几秒才一帧, 这种稀疏帧一旦触发镜像,
    /// 会让孪生体突跳到某个瞬间姿态(整机悬空/乱摆), 所以帧率不够时禁止镜像。
    /// </summary>
    public bool TelemetryStreaming { get { lock (_lock) return _telemetryHz >= 4f; } }

    /// <summary>最近一次 move 命令的前后方向: 1 前进 / -1 后退 / 0 停止。</summary>
    public int CommandedFB { get { lock (_lock) return _commandedFB; } }

    /// <summary>最近一次 move 命令的转向: 1 右转 / -1 左转 / 0 停止。</summary>
    public int CommandedLR { get { lock (_lock) return _commandedLR; } }

    #endregion

    #region 生命周期

    private void Start()
    {
        _hostEdit = host;
        Connect();
    }

    /// <summary>公开入口, 供编辑器批处理冒烟测试等非 Play 场景显式启动。</summary>
    public void Connect()
    {
        if (_telemetryThread != null) return;
        _running = true;
        _telemetryThread = new Thread(TelemetryLoop);
        _telemetryThread.IsBackground = true;
        _telemetryThread.Start();

        _controlThread = new Thread(ControlLoop);
        _controlThread.IsBackground = true;
        _controlThread.Start();
    }

    private void OnDestroy()
    {
        _running = false;
        try { if (_controlThread != null) _controlThread.Join(300); } catch (Exception) { }

        // 退出 Play 前停住实机, 避免机器狗继续行走
        StopDog();
        CloseControl();

        try { if (_telemetryClient != null) _telemetryClient.Close(); } catch (Exception) { }
        try { if (_telemetryThread != null) _telemetryThread.Join(500); } catch (Exception) { }
    }

    private void OnApplicationQuit()
    {
        _running = false;
    }

    /// <summary>更换网关地址并重建连接(面板 "重连" 按钮调用)。</summary>
    public void Reconnect(string newHost)
    {
        if (!string.IsNullOrEmpty(newHost) && newHost.Trim() != host)
            host = newHost.Trim();

        _running = false;
        CloseControl();
        try { if (_telemetryClient != null) _telemetryClient.Close(); } catch (Exception) { }
        try { if (_telemetryReader != null) _telemetryReader.Dispose(); } catch (Exception) { }
        try { if (_telemetryThread != null) _telemetryThread.Join(300); } catch (Exception) { }
        _telemetryThread = null;
        try { if (_controlThread != null) _controlThread.Join(300); } catch (Exception) { }
        _controlThread = null;

        lock (_lock)
        {
            _hasState = false;
            _telemetryHz = 0f;
            _framesInWindow = 0;
            _rateWindowUtc = DateTime.MinValue;
            _status = "重连中 " + host;
        }
        _nextControlAttemptUtc = DateTime.MinValue;
        Connect();
    }

    #endregion

    #region 遥测

    /// <summary>
    /// 网关是推送式: ESP32 每上报一帧状态就广播给所有订阅客户端, 这里只需持续读取。
    /// (以前误以为要发换行轮询, 那会白白增加延迟, 现已去掉。)
    /// </summary>
    private void TelemetryLoop()
    {
        while (_running)
        {
            NetworkStream stream = null;
            try
            {
                _telemetryClient = new TcpClient();
                _telemetryClient.NoDelay = true;
                _telemetryClient.Connect(host, telemetryPort);
                _telemetryClient.ReceiveTimeout = 500;

                stream = new NetworkStream(_telemetryClient.Client, false);
                var reader = new StreamReader(stream, Encoding.UTF8);
                _telemetryReader = reader;
                SetStatus("遥测在线 " + host + ":" + telemetryPort);

                while (_running)
                {
                    string line = reader.ReadLine();
                    if (line == null) break;
                    if (line.Length == 0) continue;

                    try
                    {
                        var packet = JsonUtility.FromJson<WavegoPacket>(line);
                        if (packet != null && packet.state != null) StoreState(packet);
                    }
                    catch (Exception) { /* 单帧解析失败不影响后续 */ }
                }
            }
            catch (Exception e)
            {
                SetStatus("遥测断开: " + e.Message);
            }
            finally
            {
                try { if (_telemetryReader != null) _telemetryReader.Dispose(); } catch (Exception) { }
                try { if (stream != null) stream.Dispose(); } catch (Exception) { }
                try { if (_telemetryClient != null) _telemetryClient.Close(); } catch (Exception) { }
                _telemetryReader = null;
                _telemetryClient = null;
            }

            for (int i = 0; i < 10 && _running; i++) Thread.Sleep(100);
        }
    }

    private void StoreState(WavegoPacket p)
    {
        var s = p.state;
        float[] cur =
        {
            s.servo0, s.servo1, s.servo2, s.servo3, s.servo4, s.servo5,
            s.servo6, s.servo7, s.servo8, s.servo9, s.servo10, s.servo11
        };

        lock (_lock)
        {
            DateTime now = DateTime.UtcNow;

            // 以 0.25s 为窗口比较舵机角: 单帧差分会把慢速行走误判成静止,
            // 窗口差分只要窗口内单关节变化超过 1° 就认为实机在运动。
            if (!_hasState || (now - _windowUtc).TotalSeconds >= 0.25)
            {
                double maxDelta = 0.0;
                if (_hasState)
                {
                    for (int i = 0; i < 12; i++)
                    {
                        double d = Math.Abs(cur[i] - _windowServo[i]);
                        if (d > maxDelta) maxDelta = d;
                    }
                }
                for (int i = 0; i < 12; i++) _windowServo[i] = cur[i];
                _windowUtc = now;
                if (maxDelta > 1.0) _lastMotionUtc = now;
            }

            for (int i = 0; i < 12; i++) _servo[i] = cur[i];

            _lastPacketUtc = now;

            // 帧率统计(1s 滑窗)。稀疏帧时 Hz 会迅速掉下来, 镜像随之关闭。
            _framesInWindow++;
            if (_rateWindowUtc == DateTime.MinValue)
                _rateWindowUtc = now;
            double windowSec = (now - _rateWindowUtc).TotalSeconds;
            if (windowSec >= 1.0)
            {
                _telemetryHz = (float)(_framesInWindow / windowSec);
                _framesInWindow = 0;
                _rateWindowUtc = now;
            }

            _vol = s.vol;
            _roll = s.roll; _pitch = s.pitch; _yaw = s.yaw;
            _x = p.x; _z = p.z;
            _hasState = true;
            _packetCount++;
            _status = "遥测在线 " + host + ":" + telemetryPort;
        }
    }

    #endregion

    #region 控制命令

    /// <summary>发送一条 WAVEGO 命令, 如 Send("move", 1)。仅入队, 不阻塞主线程。</summary>
    public bool Send(string var, int val)
    {
        if (var == "move") ApplyMoveCommand(val);

        string json = "{\"var\":\"" + var + "\",\"val\":" + val + "}";
        lock (_sendQueue)
        {
            // 离线积压过多时丢弃最旧命令, 保证最新指令优先
            if (_sendQueue.Count >= 32) _sendQueue.Dequeue();
            _sendQueue.Enqueue(json);
        }
        return true;
    }

    /// <summary>记录最近一次运动命令, 供本地步态在无实机时复现同样的动作。</summary>
    private void ApplyMoveCommand(int val)
    {
        lock (_lock)
        {
            switch (val)
            {
                case 1: _commandedFB = 1; break;
                case 5: _commandedFB = -1; break;
                case 3: _commandedFB = 0; break;
                case 4: _commandedLR = 1; break;
                case 2: _commandedLR = -1; break;
                case 6: _commandedLR = 0; break;
            }
        }
    }

    /// <summary>控制线程: 顺序取队列中的命令写入 9000, 断线自动重连。</summary>
    private void ControlLoop()
    {
        while (_running)
        {
            string json = null;
            lock (_sendQueue)
            {
                if (_sendQueue.Count > 0) json = _sendQueue.Dequeue();
            }

            if (json == null)
            {
                Thread.Sleep(10);
                continue;
            }

            lock (_controlLock)
            {
                if (_controlWriter == null && !ConnectControl()) continue;
                try
                {
                    _controlWriter.WriteLine(json);
                    _controlWriter.Flush();
                    SetStatus("控制在线 " + host + ":" + controlPort);
                }
                catch (Exception e)
                {
                    SetStatus("控制断开: " + e.Message);
                    CloseControl();
                }
            }
        }
    }

    /// <summary>带超时的控制连接, 避免不可达主机让线程长时间挂起。</summary>
    private bool ConnectControl()
    {
        CloseControl();

        if (DateTime.UtcNow < _nextControlAttemptUtc) return false;

        TcpClient client = null;
        try
        {
            client = new TcpClient();
            client.NoDelay = true;
            IAsyncResult ar = client.BeginConnect(host, controlPort, null, null);
            if (!ar.AsyncWaitHandle.WaitOne(300))
            {
                client.Close();
                _nextControlAttemptUtc = DateTime.UtcNow.AddSeconds(1);
                SetStatus("控制超时 " + host + ":" + controlPort);
                return false;
            }
            client.EndConnect(ar);
            _controlClient = client;
            _controlWriter = new StreamWriter(
                new NetworkStream(client.Client, false), Encoding.UTF8);
            _controlWriter.AutoFlush = true;
            return true;
        }
        catch (Exception e)
        {
            try { if (client != null) client.Close(); } catch (Exception) { }
            _nextControlAttemptUtc = DateTime.UtcNow.AddSeconds(1);
            SetStatus("控制连接失败: " + e.Message);
            return false;
        }
    }

    /// <summary>退出时给实机补发停止命令。</summary>
    private void StopDog()
    {
        lock (_controlLock)
        {
            if (_controlWriter == null) return;
            try
            {
                _controlWriter.WriteLine("{\"var\":\"move\",\"val\":3}");
                _controlWriter.WriteLine("{\"var\":\"move\",\"val\":6}");
                _controlWriter.Flush();
            }
            catch (Exception) { }
        }
    }

    private void CloseControl()
    {
        try { if (_controlWriter != null) _controlWriter.Dispose(); } catch (Exception) { }
        try { if (_controlClient != null) _controlClient.Close(); } catch (Exception) { }
        _controlWriter = null;
        _controlClient = null;
    }

    private void SetStatus(string text)
    {
        lock (_lock) _status = text;
    }

    #endregion

    #region 镜像

    // 遥测组序: 前-前-后-后(实测站立前两组 46.8/35.1, 后两组 25.2/56.7)
    private static readonly string[] TelemetryGroupNames = { "前A", "前B", "后A", "后B" };

    /// <summary>
    /// 用实机遥测的 12 个舵机角反算足端位置, 再用五杆闭链反解成孪生体关节角。
    /// 遥测每组三个值为 (侧摆 W, 前摆 F, 膝 B), 且是未经舵机方向修正的 IK 角度
    /// (站立时四条腿分别约为 3.6/46.8/35.1 与 3.6/25.2/56.7)。
    /// 遥测组序为 前-前-后-后, 左右腿在各自外展坐标系下几何相同, 组内顺序不影响姿态。
    /// </summary>
    public void GetMirrorTargets(float[] targetsDeg)
    {
        if (targetsDeg == null || targetsDeg.Length < 12) return;

        lock (_lock)
        {
            if (!_hasState) return;
            for (int leg = 0; leg < 4; leg++)
            {
                int src = leg * 3;

                double xMm, yMm, zMm;
                WavegoLegKinematics.ServoToFoot(_servo[src], _servo[src + 1], _servo[src + 2],
                                                out xMm, out yMm, out zMm);

                // 遥测异常(牛顿迭代发散/舵机角越界)时保留上一帧姿态, 避免腿部乱收缩导致整机悬空。
                // 窗口按站立几何收紧: 站立足端 (x≈±16, y≈95), 行走时 x∈±32、y∈[84,105];
                // 超出该范围说明反算已经跑偏, 宁可丢弃这一帧也不要让孪生体突然抬腿离地。
                if (double.IsNaN(xMm) || double.IsNaN(yMm) || double.IsNaN(zMm)) continue;
                if (yMm < 80.0 || yMm > 112.0 || Math.Abs(xMm) > 35.0) continue;

                double hip, psiA, psiB;
                WavegoLegKinematics.FootToLinkAngles(xMm, yMm, zMm,
                                                     out hip, out psiA, out psiB);
                if (double.IsNaN(hip) || double.IsNaN(psiA) || double.IsNaN(psiB)) continue;

                int dst = leg * 3;
                targetsDeg[dst + 0] = (float)Mathf.Clamp((float)hip, -40f, 40f);
                targetsDeg[dst + 1] = (float)Mathf.Clamp((float)psiA, 0f, 180f);
                targetsDeg[dst + 2] = (float)Mathf.Clamp((float)psiB, 0f, 180f);
            }
        }
    }

    #endregion

    #region 面板

    private void OnGUI()
    {
        const float w = 280f;
        float x = 460f;   // 放在控制器面板右侧, 避免两个面板重叠
        float y = 15f;

        // === 状态面板 ===
        var statusRect = new Rect(x, y, w, 250f);
        GUI.Box(statusRect, "实机 WAVEGO");
        float yy = statusRect.y + 22f;
        GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f), Status); yy += 18f;

        // 网关地址可直接在面板修改, 点"重连"生效
        _hostEdit = GUI.TextField(new Rect(x + 8f, yy, 150f, 20f), _hostEdit, 32);
        if (GUI.Button(new Rect(x + 162f, yy, 50f, 20f), "重连"))
            Reconnect(_hostEdit);
        GUI.Label(new Rect(x + 216f, yy, 60f, 20f), "9000/9001");
        yy += 22f;

        if (!HasState)
        {
            GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f),
                "提示: 电脑需与狗在同一网段, 否则必然超时");
            yy += 18f;
        }

        if (HasState)
        {
            GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f),
                string.Format("{0:F2}V  帧 {1}", _vol, _packetCount)); yy += 18f;
            GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f),
                string.Format("姿态 R{0:F1} P{1:F1} Y{2:F1}", _roll, _pitch, _yaw)); yy += 18f;
            GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f),
                string.Format("遥测:{0}  实机:{1}",
                    TelemetryFresh(0.5f) ? "新鲜" : "陈旧",
                    IsMoving ? "运动中" : "静止")); yy += 18f;
            GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f),
                string.Format("里程 x{0:F1} z{1:F1}", _x, _z)); yy += 20f;

            lock (_lock)
            {
                for (int leg = 0; leg < 4; leg++)
                {
                    GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f),
                        string.Format("{0}: {1:F1} {2:F1} {3:F1}",
                            TelemetryGroupNames[leg],
                            _servo[leg * 3], _servo[leg * 3 + 1], _servo[leg * 3 + 2]));
                    yy += 16f;
                }
            }
        }
        else
        {
            GUI.Label(new Rect(x + 8f, yy, w - 16f, 18f), "等待遥测...");
        }
        y = statusRect.yMax + 6f;

        // === 控制面板 ===
        var ctrlRect = new Rect(x, y, w, 250f);
        GUI.Box(ctrlRect, "控制");
        float cy = ctrlRect.y + 22f;

        // 遥控开关 + 镜像开关
        if (GUI.Button(new Rect(x + 8f, cy, 90f, 22f),
            remoteEnabled ? "● 遥控 ON" : "○ 遥控 OFF"))
            remoteEnabled = !remoteEnabled;
        if (GUI.Button(new Rect(x + 100f, cy, 90f, 22f),
            mirrorToTwin ? "● 镜像 ON" : "○ 镜像 OFF"))
            mirrorToTwin = !mirrorToTwin;
        cy += 30f;

        if (!remoteEnabled)
        {
            GUI.Label(new Rect(x + 8f, cy, w - 16f, 20f), "点击遥控 ON 开启按键/按钮控制");
            return;
        }

        // 移动按钮 (十字布局)
        float cx = x + w / 2f;
        float btnW = 70f, btnH = 28f;
        // 前进
        if (GUI.Button(new Rect(cx - btnW / 2f, cy, btnW, btnH), "↑ 前进"))
            Send("move", 1);
        cy += btnH + 2f;
        // 左转 停止 右转
        if (GUI.Button(new Rect(cx - btnW - 2f, cy, btnW, btnH), "← 左转"))
            Send("move", 2);
        if (GUI.Button(new Rect(cx - btnW / 2f, cy, btnW, btnH), "■ 停止"))
        {
            Send("move", 3);
            Send("move", 6);
        }
        if (GUI.Button(new Rect(cx + 2f, cy, btnW, btnH), "右转 →"))
            Send("move", 4);
        cy += btnH + 2f;
        // 后退
        if (GUI.Button(new Rect(cx - btnW / 2f, cy, btnW, btnH), "↓ 后退"))
            Send("move", 5);
        cy += btnH + 8f;

        // 动作行: 抬头/复位/稳态
        float bw = 80f;
        if (GUI.Button(new Rect(x + 8f, cy, bw, 22f), "抬头")) Send("ges", 1);
        if (GUI.Button(new Rect(x + 8f + bw + 2f, cy, bw, 22f), "姿态复位"))
        {
            Send("ges", 3); Send("ges", 6);
        }
        if (GUI.Button(new Rect(x + 8f + (bw + 2f) * 2, cy, bw, 22f),
            _steady ? "稳态:开" : "稳态:关"))
        {
            _steady = !_steady;
            Send("funcMode", _steady ? 1 : 0);
        }
        cy += 30f;

        // 键盘提示
        GUI.Label(new Rect(x + 8f, cy, w - 16f, 18f),
            "键盘:方向键移动, R=复位, X=停止");
    }

    private bool _steady;

    #endregion
}
