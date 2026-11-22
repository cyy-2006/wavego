using System;
using UnityEngine;

/// <summary>
/// 12 自由度四足机器狗数字孪生控制器。
///
/// 沿用原机械臂工程的 "Unity 作为 Modbus TCP 服务器" 架构:
///   - 本地控制模式: 键盘/步态发生器驱动 12 个关节, 并将状态发布到寄存器
///   - 外部同步模式: 客户端写入 Holding Register 目标角, 孪生体跟随
///
/// 寄存器映射(浮点占 2 个寄存器, IEEE754 大端):
///   Input Registers (Unity -> 客户端, 只读)
///     0  ~ 23   当前 12 关节角 (deg)      关节 i 起始地址 = 2*i
///     24 ~ 25   初始化完成标志 (1.0)
///     26 ~ 27   控制模式 (1=本地控制, 0=外部同步)
///     28 ~ 29   步态 (0=站立, 1=trot)
///     30 ~ 31   关节跟踪误差 (deg, 最大绝对误差)
///     32 ~ 33   机体离地高度 (m)
///
///   Holding Registers (客户端 -> Unity, 可写)
///     0  ~ 23   目标 12 关节角 (deg)      关节 i 起始地址 = 2*i
///     24 ~ 25   步态命令 (0=站立, 1=trot)
///     26 ~ 27   前进速度 (m/s)
///     28 ~ 29   转向角速度 (rad/s)
///     30 ~ 31   机体高度 (m)
///     32 ~ 33   控制模式命令 (1=本地控制, 0=外部同步)
///     34 ~ 35   复位命令 (写入 >0.5 触发一次复位)
/// </summary>
public class QuadrupedController : MonoBehaviour
{
    #region 配置

    [Header("引用")]
    public QuadrupedBuilder builder;
    public ModbusTcpServer modbusServer;
    public WavegoClient wavegoClient;
    public CameraStreamClient cameraClient;

    [Header("控制参数")]
    [Tooltip("关节跟随速度 (deg/s)")] public float jointSpeed = 260f;
    [Tooltip("最大前进速度 (m/s)")] public float maxVelocity = 0.2f;
    [Tooltip("最大转向角速度 (rad/s)")] public float maxYawRate = 1.0f;
    [Tooltip("键盘每按一次的速度增量 (m/s 或 rad/s)")] public float velocityStep = 0.1f;
    [Tooltip("步态频率 (Hz)")] public float gaitFrequency = 1.6f;
    [Tooltip("单腿步幅 (m, 足端前后总位移)")] public float footStride = 0.032f;
    [Tooltip("抬腿高度 (m)")] public float footLift = 0.009f;
    [Tooltip("机体高度调节步长 (m)")] public float heightStep = 0.02f;

    [Header("默认模式")]
    public bool defaultLocalControl = true;

    #endregion

    #region 寄存器地址

    // Input Registers (Unity 发布)
    private const int IR_CURRENT_ANGLE = 0;   // 12 关节 x 2
    private const int IR_INIT_DONE = 24;
    private const int IR_MODE = 26;
    private const int IR_GAIT = 28;
    private const int IR_TRACK_ERROR = 30;
    private const int IR_BODY_HEIGHT = 32;

    // Holding Registers (客户端写入)
    private const int HR_TARGET_ANGLE = 0;    // 12 关节 x 2
    private const int HR_GAIT = 24;
    private const int HR_VELOCITY = 26;
    private const int HR_YAW_RATE = 28;
    private const int HR_HEIGHT = 30;
    private const int HR_MODE = 32;
    private const int HR_RESET = 34;

    #endregion

    #region 运行时状态

    private readonly float[] _targetDeg = new float[12];
    private readonly float[] _currentDeg = new float[12];
    private bool _ready;
    private bool _localControl;
    private bool _trot;
    private float _vx;
    private float _yawRate;
    private float _gaitPhase;
    private float _heightCmd;
    private float _trackError;
    private float _lastModeCmd;
    private float _lastResetCmd;

    // 运动命令状态, 语义与 WAVEGO 固件 move 指令一致:
    //   moveFB: 1=前进 0=停止 -1=后退 ; moveLR: 1=右转 0=停止 -1=左转
    private int _moveFB;
    private int _moveLR;
    private int _sentMoveFB;
    private int _sentMoveLR;

    // 键盘是否正在按住方向键(松开即停), 以及给实机重发命令的计时器
    private bool _keyboardActive;
    private float _dogCmdTimer;

    private Vector3 _homePosition;
    private Quaternion _homeRotation;

    // 本帧是否处于实机镜像(遥测新鲜且实机确实在动)
    private bool _mirrorActive;

    // 镜像模式下消除"整机悬空"的机身高度补偿量(m)
    private float _groundFix;

    #endregion

    #region 生命周期

    private void Start()
    {
        if (builder == null) builder = GetComponent<QuadrupedBuilder>();
        if (builder == null) builder = FindAnyObjectByType<QuadrupedBuilder>();
        if (builder == null)
        {
            Debug.LogError("[QuadrupedController] 缺少 QuadrupedBuilder, 无法启动");
            return;
        }

        builder.Build();

        if (modbusServer == null) modbusServer = FindAnyObjectByType<ModbusTcpServer>();
        if (modbusServer == null)
            modbusServer = new GameObject("ModbusTcpServer").AddComponent<ModbusTcpServer>();

        // 实机网关客户端: 场景里没有就自动创建, 默认连 192.168.67.78
        if (wavegoClient == null) wavegoClient = FindAnyObjectByType<WavegoClient>();
        if (wavegoClient == null)
            wavegoClient = new GameObject("WavegoClient").AddComponent<WavegoClient>();

        // 摄像头画面同步: 同样自动挂载, 默认拉 192.168.67.78:5000 的 MJPEG 流
        if (cameraClient == null) cameraClient = FindAnyObjectByType<CameraStreamClient>();
        if (cameraClient == null)
            cameraClient = new GameObject("CameraStreamClient").AddComponent<CameraStreamClient>();

        _homePosition = transform.position;
        _homeRotation = transform.rotation;
        _heightCmd = builder.StandHeight;

        float[] stand = builder.StandPose();
        Array.Copy(stand, _targetDeg, 12);
        Array.Copy(stand, _currentDeg, 12);
        builder.ApplyJointAngles(_currentDeg);

        _localControl = defaultLocalControl;
        _lastModeCmd = _localControl ? 1f : 0f;

        if (modbusServer != null)
        {
            modbusServer.WriteFloat(IR_INIT_DONE, 1f);
            modbusServer.WriteFloat(HR_MODE, _lastModeCmd);
            modbusServer.WriteFloat(HR_GAIT, 0f);
            modbusServer.WriteFloat(HR_VELOCITY, 0f);
            modbusServer.WriteFloat(HR_YAW_RATE, 0f);
            modbusServer.WriteFloat(HR_HEIGHT, _heightCmd);
        }

        _ready = true;
        Debug.Log("[QuadrupedController] 四足孪生已启动, 12 关节就绪");
    }

    private void Update()
    {
        if (!_ready) return;

        float dt = Time.deltaTime;
        if (dt <= 0f) return;

        HandleKeyboard();
        ReadExternalCommands();

        // 每帧同步面板/遥控的运动命令, 保证镜像结束后本地步态状态不残留旧指令
        SyncRemoteCommand();

        // 实机在动时以遥测为准(数字孪生必须跟着实机走); 实机静止或未连接时走本地步态,
        // 保证没有实机的情况下点击/按键也能立即看到关节动作。
        // 额外要求遥测在持续推流: 稀疏单帧(ESP32 空闲时十几秒才一帧)会让孪生体突跳到
        // 某个瞬间姿态, 表现为"走不到一秒就整机悬空", 因此帧率不够时一律走本地步态。
        _mirrorActive = wavegoClient != null && wavegoClient.mirrorToTwin
                        && wavegoClient.HasState
                        && wavegoClient.TelemetryStreaming
                        && wavegoClient.TelemetryFresh(0.5f)
                        && wavegoClient.IsMoving;

        // 由当前操控指令推导机体速度(镜像模式下也要据此移动机体位置)
        UpdateLocomotionVelocity();

        if (_mirrorActive)
            wavegoClient.GetMirrorTargets(_targetDeg);
        else if (_localControl)
            GenerateLocalTargets(dt);

        // 静止时周期性补发停止命令, 防止某次停止包丢失导致实机一直走
        HeartbeatDog(dt);

        SmoothJoints(dt);
        ApplyBodyMotion(dt);
        PublishState();
    }

    #endregion

    #region 键盘输入

    private void HandleKeyboard()
    {
        // 模式切换与复位两种模式下都可执行
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            _localControl = !_localControl;
            if (modbusServer != null) modbusServer.WriteFloat(HR_MODE, _localControl ? 1f : 0f);
            _lastModeCmd = _localControl ? 1f : 0f;
            Debug.Log($"[QuadrupedController] 切换到{(_localControl ? "本地控制" : "外部同步")}模式");
        }

        if (Input.GetKeyDown(KeyCode.R)) ResetPose();

        if (!_localControl) return;

        // 按住方向键行走, 松开立即停止; 避免按一次之后实机一直前进。
        bool fwd = Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow);
        bool back = Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow);
        bool right = Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow);
        bool left = Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow);

        if (fwd || back || right || left)
        {
            _moveFB = fwd ? 1 : (back ? -1 : 0);
            _moveLR = right ? 1 : (left ? -1 : 0);
            _keyboardActive = true;
        }
        else if (_keyboardActive ||
                 Input.GetKeyDown(KeyCode.X) || Input.GetKeyDown(KeyCode.Space))
        {
            _moveFB = 0;
            _moveLR = 0;
            _keyboardActive = false;
        }

        SendMoveToDog();

        if (Input.GetKey(KeyCode.E))
            _heightCmd = Mathf.Clamp(_heightCmd + heightStep * Time.deltaTime * 20f,
                                     builder.MinBodyHeight, builder.MaxBodyHeight);
        if (Input.GetKey(KeyCode.Q))
            _heightCmd = Mathf.Clamp(_heightCmd - heightStep * Time.deltaTime * 20f,
                                     builder.MinBodyHeight, builder.MaxBodyHeight);
    }

    /// <summary>
    /// 只在"静止"状态下周期补发停止命令。运动中的指令绝不重发:
    /// WAVEGO 固件每次收到 move 都会重新触发步态, 高频重发会让实机走得一卡一卡。
    /// 停止命令是幂等的, 重发不会造成抖动, 却能兜住丢失的停止包。
    /// </summary>
    private void HeartbeatDog(float dt)
    {
        if (wavegoClient == null || !wavegoClient.remoteEnabled) return;
        if (_moveFB != 0 || _moveLR != 0)
        {
            _dogCmdTimer = 1f;
            return;
        }

        _dogCmdTimer -= dt;
        if (_dogCmdTimer > 0f) return;
        _dogCmdTimer = 1f;

        wavegoClient.Send("move", 3);
        wavegoClient.Send("move", 6);
    }

    /// <summary>把面板/遥控最近一次 move 命令同步进本地步态状态(不再重复下发)。</summary>
    private void SyncRemoteCommand()
    {
        if (wavegoClient == null) return;

        int fb = wavegoClient.CommandedFB;
        if (fb != _sentMoveFB)
        {
            _moveFB = fb;
            _sentMoveFB = fb;
        }

        int lr = wavegoClient.CommandedLR;
        if (lr != _sentMoveLR)
        {
            _moveLR = lr;
            _sentMoveLR = lr;
        }
    }

    /// <summary>把锁存的运动命令按固件协议转发给实机, 只在变化时发送。</summary>
    private void SendMoveToDog()
    {
        if (wavegoClient == null || !wavegoClient.remoteEnabled) return;

        if (_moveFB != _sentMoveFB)
        {
            wavegoClient.Send("move", _moveFB > 0 ? 1 : (_moveFB < 0 ? 5 : 3));
            _sentMoveFB = _moveFB;
        }
        if (_moveLR != _sentMoveLR)
        {
            wavegoClient.Send("move", _moveLR > 0 ? 4 : (_moveLR < 0 ? 2 : 6));
            _sentMoveLR = _moveLR;
        }
    }

    #endregion

    #region 外部指令读取

    private void ReadExternalCommands()
    {
        if (modbusServer == null) return;

        // 模式命令: 仅在值变化时生效, 避免覆盖键盘切换
        float modeCmd = modbusServer.ReadFloat(HR_MODE);
        if (!float.IsNaN(modeCmd) && !float.IsInfinity(modeCmd) &&
            Mathf.Abs(modeCmd - _lastModeCmd) > 0.5f)
        {
            _lastModeCmd = modeCmd;
            _localControl = modeCmd > 0.5f;
        }

        // 复位命令: 上升沿触发一次
        float resetCmd = modbusServer.ReadFloat(HR_RESET);
        if (!float.IsNaN(resetCmd) && !float.IsInfinity(resetCmd))
        {
            if (resetCmd > 0.5f && _lastResetCmd <= 0.5f)
            {
                ResetPose();
                modbusServer.WriteFloat(HR_RESET, 0f);
                resetCmd = 0f;
            }
            _lastResetCmd = resetCmd;
        }

        if (_localControl) return;

        for (int i = 0; i < 12; i++)
        {
            float v = modbusServer.ReadFloat(HR_TARGET_ANGLE + i * 2);
            if (!float.IsNaN(v) && !float.IsInfinity(v)) _targetDeg[i] = v;
        }

        _trot = modbusServer.ReadFloat(HR_GAIT) > 0.5f;

        float vx = modbusServer.ReadFloat(HR_VELOCITY);
        if (!float.IsNaN(vx) && !float.IsInfinity(vx))
            _vx = Mathf.Clamp(vx, -maxVelocity, maxVelocity);

        float yaw = modbusServer.ReadFloat(HR_YAW_RATE);
        if (!float.IsNaN(yaw) && !float.IsInfinity(yaw))
            _yawRate = Mathf.Clamp(yaw, -maxYawRate, maxYawRate);

        float h = modbusServer.ReadFloat(HR_HEIGHT);
        if (h > 0.1f && h < 1f) _heightCmd = Mathf.Clamp(h, builder.MinBodyHeight, builder.MaxBodyHeight);
    }

    #endregion

    #region 本地步态生成

    /// <summary>由操控指令推导机体前进速度与转向角速度(本地步态与镜像模式共用)。</summary>
    private void UpdateLocomotionVelocity()
    {
        float gaitSpeed = footStride * gaitFrequency;
        float legRadius = Mathf.Max(0.05f,
            Mathf.Sqrt(builder.hipForwardOffset * builder.hipForwardOffset
                       + builder.hipLateralOffset * builder.hipLateralOffset));
        _vx = _moveFB * Mathf.Min(maxVelocity, gaitSpeed);
        _yawRate = _moveLR * Mathf.Min(maxYawRate, gaitSpeed / legRadius);
    }

    private void GenerateLocalTargets(float dt)
    {
        // 与实机一致: 收到前进/后退或转向命令就持续行走, 直到收到停止。
        // 机体移动速度由足端步幅决定, 避免脚底相对地面打滑。
        _trot = (_moveFB != 0 || _moveLR != 0);
        if (modbusServer != null) modbusServer.WriteFloat(HR_GAIT, _trot ? 1f : 0f);

        if (!_trot)
        {
            Array.Copy(builder.StandPose(), _targetDeg, 12);
            return;
        }

        _gaitPhase += 2f * Mathf.PI * gaitFrequency * dt;
        if (_gaitPhase > 2f * Mathf.PI) _gaitPhase -= 2f * Mathf.PI;

        // 足端轨迹参数(mm): 机体高度决定髋到足端的下移距离
        float baseYmm = (_heightCmd + builder.hipVerticalOffset - builder.footRadius) * 1000f;
        float halfStrideMm = footStride * 500f;
        float liftMm = footLift * 1000f;
        float baseZmm = builder.standFootZ * 1000f;

        for (int leg = 0; leg < 4; leg++)
        {
            // 对角步态: FR+RL 一组, FL+RR 一组(对应固件 simpleGait 的 Group_A/Group_B)
            bool groupA = leg == 1 || leg == 2;
            float cycle = _gaitPhase / (2f * Mathf.PI) + (groupA ? 0f : 0.5f);
            cycle -= Mathf.Floor(cycle);

            float strideMm, liftNow;
            if (cycle < 0.5f)
            {
                // 支撑相: 足端相对机体从前往后划
                strideMm = halfStrideMm * (1f - 4f * cycle);
                liftNow = 0f;
            }
            else
            {
                // 摆动相: 抬腿并回到前方
                float t = (cycle - 0.5f) * 2f;
                strideMm = halfStrideMm * (4f * (cycle - 0.5f) - 1f);
                liftNow = liftMm * Mathf.Sin(Mathf.PI * t);
            }

            float forwardMm = 0f;
            if (_moveFB != 0)
                forwardMm = strideMm * (_moveFB > 0 ? 1f : -1f);

            float lateralMm = 0f;
            if (_moveLR != 0)
            {
                // 转向时两组腿沿各自的"外展"方向反向摆动
                float sign = groupA ? -_moveLR : _moveLR;
                lateralMm = strideMm * sign;
            }

            float baseXmm = builder.standFootX * 1000f * (leg < 2 ? 1f : -1f);
            float xMm = baseXmm + forwardMm;
            float zMm = baseZmm + lateralMm;
            float yMm = baseYmm - liftNow;

            double hipDeg, psiA, psiB;
            WavegoLegKinematics.FootToLinkAngles(xMm, yMm, zMm,
                out hipDeg, out psiA, out psiB);

            int i = leg * 3;
            _targetDeg[i + 0] = (float)hipDeg;
            _targetDeg[i + 1] = (float)psiA;
            _targetDeg[i + 2] = (float)psiB;
        }
    }

    #endregion

    #region 关节与机体更新

    private void SmoothJoints(float dt)
    {
        float step = jointSpeed * dt;
        for (int i = 0; i < 12; i++)
        {
            Vector2 lim = QuadrupedBuilder.JointLimits[i];
            float target = Mathf.Clamp(_targetDeg[i], lim.x, lim.y);
            _currentDeg[i] = Mathf.MoveTowards(_currentDeg[i], target, step);
        }
        builder.ApplyJointAngles(_currentDeg);

        // 跟踪误差: 用于观察虚实/指令跟随效果
        float maxErr = 0f;
        for (int i = 0; i < 12; i++)
        {
            float e = Mathf.Abs(_targetDeg[i] - _currentDeg[i]);
            if (e > maxErr) maxErr = e;
        }
        _trackError = maxErr;
    }

    private void ApplyBodyMotion(float dt)
    {
        // 只沿水平方向前进: 用投影后的前向, 防止模型被赋予任何垂直位移而"升空"
        Vector3 forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
        if (forward.sqrMagnitude < 1e-6f) forward = Vector3.forward;
        if (Mathf.Abs(_vx) > 1e-4f)
            transform.position += forward * (_vx * dt);

        if (Mathf.Abs(_yawRate) > 1e-4f)
            transform.Rotate(Vector3.up, _yawRate * Mathf.Rad2Deg * dt, Space.World);

        // 足端高度闭环: 把最低脚稳定压在 footRadius 高度上。
        // 双向都允许(之前只许下压, 镜像退出后机身会卡在压低状态, 脚陷进地面);
        // 小死区 + 小增益避免抖动, 限幅防止机体被顶飞或压穿。
        float fixErr = builder.footRadius - builder.LowestFootY();
        if (!float.IsNaN(fixErr) && Mathf.Abs(fixErr) > 0.0015f)
        {
            _groundFix = Mathf.Clamp(
                _groundFix + Mathf.Clamp(fixErr, -0.02f, 0.02f) * Mathf.Min(1f, 8f * dt),
                -0.06f, 0.02f);
        }

        // 步态时躯干轻微上下起伏; 机身高度限定在固件的 WALK_HEIGHT_MIN/MAX 之间
        float bob = _trot ? 0.003f * Mathf.Sin(2f * _gaitPhase) : 0f;
        float h = Mathf.Clamp(_heightCmd + bob + _groundFix,
                              builder.MinBodyHeight, builder.MaxBodyHeight);
        builder.SetBodyHeight(h);
    }

    private void ResetPose()
    {
        transform.position = _homePosition;
        transform.rotation = _homeRotation;
        _vx = 0f;
        _yawRate = 0f;
        _trot = false;
        _gaitPhase = 0f;
        _heightCmd = builder.StandHeight;
        _groundFix = 0f;

        // 复位同时停止实机
        _moveFB = 0;
        _moveLR = 0;
        _keyboardActive = false;
        SendMoveToDog();

        float[] stand = builder.StandPose();
        Array.Copy(stand, _targetDeg, 12);
        Array.Copy(stand, _currentDeg, 12);
        builder.ApplyJointAngles(_currentDeg);

        if (modbusServer != null)
        {
            modbusServer.WriteFloat(HR_GAIT, 0f);
            modbusServer.WriteFloat(HR_VELOCITY, 0f);
            modbusServer.WriteFloat(HR_YAW_RATE, 0f);
            modbusServer.WriteFloat(HR_HEIGHT, _heightCmd);
        }
        Debug.Log("[QuadrupedController] 已复位");
    }

    #endregion

    #region 状态发布

    private void PublishState()
    {
        if (modbusServer == null) return;

        for (int i = 0; i < 12; i++)
            modbusServer.WriteFloat(IR_CURRENT_ANGLE + i * 2, _currentDeg[i]);

        modbusServer.WriteFloat(IR_MODE, _localControl ? 1f : 0f);
        modbusServer.WriteFloat(IR_GAIT, _trot ? 1f : 0f);
        modbusServer.WriteFloat(IR_TRACK_ERROR, _trackError);
        modbusServer.WriteFloat(IR_BODY_HEIGHT, builder.Body != null
            ? builder.Body.localPosition.y : 0f);
    }

    #endregion

    #region 界面

    private GUIStyle _box, _title, _label, _value;

    private void OnGUI()
    {
        InitStyles();

        GUILayout.BeginArea(new Rect(15f, 15f, 430f, 300f), _box);

        GUILayout.BeginHorizontal();
        GUILayout.Label("四足机器狗数字孪生", _title);
        GUILayout.FlexibleSpace();
        bool running = modbusServer != null && modbusServer.IsRunning;
        GUI.color = running ? Color.green : Color.red;
        GUILayout.Label(running ? "●" : "○", _title, GUILayout.Width(20f));
        GUI.color = Color.white;
        GUILayout.EndHorizontal();

        GUILayout.Space(6f);

        _value.normal.textColor = _localControl
            ? new Color(0.5f, 1f, 0.5f) : new Color(1f, 0.8f, 0.3f);
        GUILayout.BeginHorizontal();
        GUILayout.Label("模式:", _label, GUILayout.Width(40f));
        GUILayout.Label(_localControl ? "本地控制" : "外部同步", _value, GUILayout.Width(80f));
        GUILayout.Label("步态:", _label, GUILayout.Width(40f));
        _value.normal.textColor = _trot ? new Color(0.5f, 1f, 0.5f) : Color.gray;
        GUILayout.Label(_trot ? "TROT" : "站立", _value, GUILayout.Width(60f));
        GUILayout.Label("客户端:", _label, GUILayout.Width(50f));
        _value.normal.textColor = new Color(0.5f, 1f, 0.5f);
        GUILayout.Label($"{modbusServer?.ClientCount ?? 0}", _value);
        GUILayout.EndHorizontal();

        GUILayout.Space(4f);
        GUILayout.Label(
            $"vx {_vx:F2} m/s    ω {_yawRate:F2} rad/s    高度 {_heightCmd:F2} m    误差 {_trackError:F1}°",
            _label);

        if (builder != null && builder.Body != null)
        {
            GUILayout.Label(
                $"镜像 {(_mirrorActive ? "跟随实机" : "本地")}  遥测 {(wavegoClient != null ? wavegoClient.TelemetryHz : 0f):F1}Hz  " +
                $"机身Y {builder.Body.position.y:F3}  贴地补偿 {_groundFix:F3}  最低脚 {builder.LowestFootY():F3}",
                _label);
        }

        GUILayout.Space(6f);
        GUILayout.Label("关节角 (deg):", _label);

        string[] legNames = QuadrupedBuilder.LegPrefixes;
        for (int leg = 0; leg < 4; leg++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(legNames[leg], _label, GUILayout.Width(28f));
            for (int j = 0; j < 3; j++)
            {
                int i = leg * 3 + j;
                _value.normal.textColor = new Color(0.5f, 1f, 0.5f);
                GUILayout.Label($"{_currentDeg[i],7:F1}", _value, GUILayout.Width(78f));
            }
            GUILayout.EndHorizontal();
        }

        GUILayout.Space(6f);
        _label.fontSize = 10;
        _label.normal.textColor = new Color(0.6f, 0.6f, 0.6f);
        GUILayout.Label("W/S:前后  A/D:转向  Q/E:高度  X/空格:停止  Tab:模式  R:复位", _label);
        _label.fontSize = 12;
        _label.normal.textColor = new Color(0.85f, 0.85f, 0.85f);

        GUILayout.EndArea();
    }

    private void InitStyles()
    {
        if (_box != null) return;

        _box = new GUIStyle(GUI.skin.box);
        _box.normal.background = MakeTex(new Color(0.1f, 0.1f, 0.15f, 0.85f));
        _box.padding = new RectOffset(12, 12, 10, 10);

        _title = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft,
        };
        _title.normal.textColor = new Color(0.4f, 0.8f, 1f);

        _label = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
        };
        _label.normal.textColor = new Color(0.85f, 0.85f, 0.85f);

        _value = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
        };
    }

    private static Texture2D MakeTex(Color color)
    {
        var tex = new Texture2D(2, 2);
        var pixels = new Color[4];
        for (int i = 0; i < pixels.Length; i++) pixels[i] = color;
        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }

    #endregion
}
