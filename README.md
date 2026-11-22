# WAVEGO 四足机器狗数字孪生系统

基于 **Unity 6 + 树莓派网关 + WAVEGO（ESP32）实机** 的四足机器狗数字孪生工程：
Unity 端重建了与实机一致的 12 自由度、平面五杆闭链连杆腿部结构，可通过键盘/面板控制实机行走，
也可以把实机姿态、实机摄像头画面同步回 Unity 孪生体。

同时附带一套 ROS 2 / Isaac Sim 的数字孪生验证包，用于关节级联调与 URDF 校验。

---

## 目录

- [1. 系统架构](#1-系统架构)
- [2. 目录结构](#2-目录结构)
- [3. 环境要求](#3-环境要求)
- [4. 网络规划](#4-网络规划)
- [5. 部署一：树莓派网关与摄像头](#5-部署一树莓派网关与摄像头)
- [6. 部署二：Unity 数字孪生](#6-部署二unity-数字孪生)
- [7. 部署三：ROS 2 / Isaac Sim（可选）](#7-部署三ros-2--isaac-sim可选)
- [8. 联调流程](#8-联调流程)
- [9. 通信协议](#9-通信协议)
- [10. 运动学与步态](#10-运动学与步态)
- [11. 常见问题排查](#11-常见问题排查)
- [12. 已知限制](#12-已知限制)

---

## 1. 系统架构

```
┌──────────────────────────── PC ────────────────────────────┐
│  Unity 6 (场景: Assets/Scenes/QuadrupedTwin.unity)          │
│    QuadrupedController   键盘/面板指令、对角步态、贴地补偿    │
│    QuadrupedBuilder      程序化生成 12 自由度五杆闭链模型     │
│    WavegoClient    ─── TCP 9000 控制 / 9001 遥测 ──────┐     │
│    CameraStreamClient ─── HTTP 5000 MJPEG 画面 ────┐   │     │
│    ModbusTcpServer  ←── 1502 外部 PLC / 上位机 ──┐  │   │     │
└──────────────────────────────────────────────────┼──┼───┼────┘
                          WiFi / 以太网             │  │   │
┌───────────────── 树莓派 4B (默认 192.168.67.78) ──┼──┼───┼────┐
│  wavego_pi_gateway.py   9000/9001 ↔ /dev/serial0 │  │   │    │
│  camera_server.py       5000 MJPEG               │  │   │    │
└──────────────────────────────────────────────────┼──┼───┼────┘
                          UART 115200               │  │   │
┌──────────────────────── ESP32 (WAVEGO 主控) ──────┼──┼───┼────┐
│  12 路舵机（4 腿 × 3）+ IMU + 电池电压 + 摄像头   │  │   │    │
└──────────────────────────────────────────────────┴──┴───┴────┘
```

### 端口一览

| 端口 | 协议 | 方向 | 用途 |
|------|------|------|------|
| **9000** | TCP，一行一条 JSON | PC → 树莓派 → ESP32 | 运动/动作/灯光控制 |
| **9001** | TCP，一行一条 JSON | ESP32 → 树莓派 → PC | 舵机角、IMU、电压遥测 |
| **5000** | HTTP / MJPEG | 树莓派 → PC | 摄像头画面（`/video_feed`、`/snapshot.jpg`、`/health`） |
| **1502** | Modbus TCP | 外部客户端 → Unity | 孪生体状态读取 / 关节角下发 |

---

## 2. 目录结构

```
wavego/
├── README.md                       # 本文档
├── .gitignore
├── DTSourceCode/                   # Unity 6 数字孪生工程
│   ├── Assets/
│   │   ├── Scenes/QuadrupedTwin.unity
│   │   ├── Scripts/
│   │   │   ├── Modbus/ModbusTcpServer.cs
│   │   │   ├── Quadruped/
│   │   │   │   ├── QuadrupedBuilder.cs        # 程序化建模（五杆闭链腿）
│   │   │   │   ├── QuadrupedController.cs     # 步态/输入/贴地补偿
│   │   │   │   ├── WavegoLegKinematics.cs     # 正逆解（移植 WAVEGO 固件）
│   │   │   │   ├── WavegoClient.cs            # 9000/9001 网关客户端
│   │   │   │   └── CameraStreamClient.cs      # 5000 MJPEG 画面同步
│   │   │   ├── RotateAround.cs                # 轨道相机
│   │   │   └── Editor/MissingScriptsCleaner.cs
│   │   └── Editor/
│   ├── Packages/manifest.json
│   └── ProjectSettings/
└── quadruped_digital_twin/         # ROS 2 / Isaac Sim 验证工程（可选）
    ├── src/
    │   ├── quadruped_description/  # URDF + RViz
    │   ├── quadruped_control/      # 12 关节指令发布节点
    │   ├── quadruped_twin/         # 虚实同步桥 twin_bridge
    │   └── quadruped_sim/          # Isaac Sim 启动与配置
    └── scripts/install_isaacsim.sh
```

> 树莓派侧的 `wavego_pi_gateway.py`、`camera_server.py` 及 systemd 服务不随仓库分发，
> 部署方式见 [第 5 章](#5-部署一树莓派网关与摄像头)。

---

## 3. 环境要求

### 3.1 硬件

| 设备 | 说明 |
|------|------|
| WAVEGO 四足机器人 | ESP32 主控，12 路总线舵机，板载 IMU / 电池检测 |
| 树莓派 4B | 建议 2GB 内存以上 + 16GB 以上 TF 卡，带 WiFi |
| 摄像头 | USB 摄像头（`/dev/video0`）或 CSI 摄像头（Picamera2） |
| PC | Ubuntu 20.04+ / Windows 10+，需与机器狗处于同一局域网 |
| 路由器 | 2.4GHz WiFi，建议给树莓派固定 DHCP 绑定 |

### 3.2 软件版本

| 软件 | 版本 |
|------|------|
| Unity Editor | **6000.6.7f1**（Unity 6，`ProjectSettings/ProjectVersion.txt`） |
| Python（树莓派） | 3.9+（Raspberry Pi OS Bookworm 自带 3.11） |
| Python 依赖 | `pyserial`、`flask`、`opencv`（`python3-opencv`） |
| ROS 2（可选） | Humble |
| Isaac Sim（可选） | 4.5.0（pip 安装，见 `scripts/install_isaacsim.sh`） |

Unity 依赖见 `DTSourceCode/Packages/manifest.json`：TextMeshPro、Timeline、UGUI、Test Framework 等，
全部为官方包，无需额外下载第三方插件。

---

## 4. 网络规划

### 4.1 与 PC 同一 WiFi（推荐）

路由器给树莓派分配一个地址，工程默认使用：

- 树莓派：`192.168.67.78`
  - 9000 控制、9001 遥测、5000 摄像头

PC 需要与树莓派处于**同一网段**，可用下面的命令确认连通性：

```bash
ping -c 3 192.168.67.78
```

### 4.2 机器狗自身热点（AP 直连）

WAVEGO 自身开启热点时，树莓派地址为 `192.168.146.78`，PC 需要连接机器狗的热点。
此时请把 Unity 面板里的地址改为 `192.168.146.78` 后点“重连”。

### 4.3 修改地址

| 组件 | 修改方式 |
|------|----------|
| `WavegoClient` | Play 后在“实机 WAVEGO”面板的地址框输入新 IP，点“重连” |
| `CameraStreamClient` | 在 Inspector 修改 `host` / `port` / `path`，或 Play 前改场景中的默认值 |

---

## 5. 部署一：树莓派网关与摄像头

以下命令均在树莓派上执行（SSH 或串口终端）。

### 5.1 系统准备

```bash
sudo apt update && sudo apt full-upgrade -y

# 启用硬件串口（关闭串口登录 shell，保留 /dev/serial0）
sudo raspi-config nonint do_serial 0

# 当前用户加入串口与摄像头组
sudo usermod -aG dialout,video pi

# 重启使串口与用户组生效
sudo reboot
```

重启后确认串口存在：

```bash
ls -l /dev/serial0
# lrwxrwxrwx ... /dev/serial0 -> ttyAMA0
```

### 5.2 安装依赖

```bash
sudo apt install -y python3-serial python3-flask python3-opencv
python3 -c "import serial, flask, cv2; print('deps ok')"
```

### 5.3 部署网关程序

```bash
sudo mkdir -p /opt/wavego-twin
sudo install -m 0644 wavego_pi_gateway.py /opt/wavego-twin/
```

网关职责：

1. 打开 `/dev/serial0`（115200），按行读取 ESP32 上报的 JSON 并广播给所有 9001 客户端；
2. 监听 9000，把收到的每行 JSON 追加换行后写入串口，转发给 ESP32。

启动参数（与 systemd 单元一致）：

```bash
python3 /opt/wavego-twin/wavego_pi_gateway.py \
  --port /dev/serial0 --baud 115200 \
  --control-port 9000 --telemetry-port 9001
```

**注意**：9001 为**单客户端独占**推送模式，同一时间只能有一个订阅者（Unity 或其它调试工具二选一），
否则先连接的客户端会被踢下线。

### 5.4 部署摄像头服务

```bash
sudo install -m 0644 camera_server.py /opt/wavego-twin/

sudo systemctl restart wavego-camera
```

运行参数：`--host 0.0.0.0 --port 5000 --width 640 --height 480 --fps 12 --quality 70`。

摄像头服务提供三个端点：

| 端点 | 说明 |
|------|------|
| `GET /health` | 返回 `{"backend":"picamera2|opencv","ok":true}` |
| `GET /video_feed` | MJPEG 流（`multipart/x-mixed-replace; boundary=frame`） |
| `GET /snapshot.jpg` | 单张 JPEG，便于调试 |

#### ⚠️ 必读：MJPEG 推流必须按帧序号去重

早期版本的 `mjpeg_frames()` 在“没有新帧”时会**把同一帧死循环重发**，实测 6 秒传输 71MB
（约 12MB/s），会把 WiFi 打满并拖垮遥测。正确写法：

```python
def mjpeg_frames():
    last_seq = -1
    while True:
        seq, frame = camera.latest_with_seq()
        if frame is None:
            time.sleep(0.02)
            continue
        # 只推“新”帧：旧版本在同一个帧上死循环重发，会把 WiFi 打满（实测 12MB/s）
        if seq == last_seq:
            time.sleep(0.01)
            continue
        last_seq = seq
        yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
               + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
```

对应的 `CameraWorker` 需要维护 `seq` 计数：

```python
def loop(self):
    while self.running:
        ok, image = self.read()
        if ok:
            encoded, jpeg = cv2.imencode(".jpg", image,
                                         [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if encoded:
                with self.lock:
                    self.frame = jpeg.tobytes()
                    self.seq += 1

def latest_with_seq(self):
    with self.lock:
        return self.seq, self.frame
```

打补丁后实测：**12.4 fps、平均 22.5KB/帧、约 280KB/s（2.2Mbps）**，画面流畅且不再抢占带宽。

### 5.5 systemd 服务

`/etc/systemd/system/wavego-gateway.service`：

```ini
[Unit]
Description=WAVEGO serial gateway (9000 control / 9001 telemetry)
After=network.target

[Service]
Type=simple
User=pi
ExecStart=/usr/bin/python3 /opt/wavego-twin/wavego_pi_gateway.py \
  --port /dev/serial0 --baud 115200 \
  --control-port 9000 --telemetry-port 9001
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

`/etc/systemd/system/wavego-camera.service`：

```ini
[Unit]
Description=WAVEGO camera MJPEG server
After=network.target

[Service]
Type=simple
User=pi
ExecStart=/usr/bin/python3 /opt/wavego-twin/camera_server.py \
  --host 0.0.0.0 --port 5000 \
  --width 640 --height 480 --fps 12 --quality 70
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

启用并启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now wavego-gateway wavego-camera
systemctl status wavego-gateway wavego-camera --no-pager
```

> 若出现 `Failed to connect to bus: Connection refused`，说明 systemd 用户总线异常。
> 可先 `sudo systemctl daemon-reexec` 或重启树莓派；临时调试也可用
> `sudo setsid nohup python3 /opt/wavego-twin/camera_server.py ... &` 直接拉起。

### 5.6 部署自检

```bash
# 1) 摄像头健康检查
curl -s http://127.0.0.1:5000/health
# {"backend":"opencv","ok":true}

# 2) 抽一张快照，确认是合法 JPEG
curl -s -o /tmp/snap.jpg -w '%{http_code} %{size_download} %{content_type}\n' \
  http://127.0.0.1:5000/snapshot.jpg
# 200 22711 image/jpeg

# 3) 限时拉流，确认码率正常（约 280KB/s，而不是 12MB/s）
timeout 6 curl -s -o /dev/null -w '%{size_download}\n' http://127.0.0.1:5000/video_feed

# 4) 遥测端口是否有数据（ESP32 空闲时可能很稀疏）
timeout 3 nc 127.0.0.1 9001

# 5) 控制端口是否监听
ss -lntp | grep -E '9000|9001|5000'
```

---

## 6. 部署二：Unity 数字孪生

### 6.1 打开工程

1. 用 **Unity Hub** 添加 `DTSourceCode` 目录；
2. 选择 **Unity 6000.6.7f1** 打开；
3. 打开场景 `Assets/Scenes/QuadrupedTwin.unity`。

首次导入时 Unity 会重新生成 `Library/`（已加入 `.gitignore`，不需要提交）。

> Unity 6 已移除内置的 `com.unity.modules.vr`，若 `manifest.json` 里残留该项会导致启动报错，删除即可。
> 同理 `com.unity.collab-proxy` 与 Unity 6 存在 API 兼容问题，本项目不需要，建议移除。

### 6.2 场景组成

| 对象 | 组件 | 说明 |
|------|------|------|
| `QuadrupedTwin` | `QuadrupedBuilder` | 程序化生成躯干、4 条五杆闭链腿、12 个关节 |
| 同上 | `QuadrupedController` | 步态、输入、状态发布 |
| 同上 | `ModbusTcpServer` | Modbus TCP 服务器，端口 1502 |
| `Ground Plane` | — | 60m × 60m 地面，y = 0 |
| `Main Camera` | `RotateAround` | 鼠标拖拽轨道相机，跟随 `CameraTarget` |

运行时会**自动挂载**以下组件（场景里没有就新建，无需手动拖拽）：

- `WavegoClient` —— 网关客户端，默认 `192.168.67.78:9000/9001`
- `CameraStreamClient` —— 摄像头客户端，默认 `192.168.67.78:5000/video_feed`

### 6.3 运行与操作

点击 **Play** 后，控制台应出现：

```
[QuadrupedController] 四足孪生已启动, 12 关节就绪
[ModbusTcpServer] 服务器已启动: 1502
[CameraStreamClient] 摄像头画面已同步: 640x480   # 摄像头在线时
```

| 按键 | 功能 |
|------|------|
| `W` / `↑` | 前进（按住持续行走，松开停止） |
| `S` / `↓` | 后退 |
| `A` / `←` | 左转 |
| `D` / `→` | 右转 |
| `X` / `空格` | 立即停止 |
| `Q` / `E` | 降低 / 升高机体 |
| `Tab` | 切换 本地控制 / 外部同步 模式 |
| `R` | 复位（回到初始位姿，同时停止实机） |

### 6.4 界面说明

| 位置 | 内容 |
|------|------|
| 左上（15,15） | 孪生体状态：模式、步态、客户端数、速度、高度、跟踪误差、12 关节角、贴地补偿 |
| 右上（460,15） | “实机 WAVEGO”：连接状态、地址框 + 重连、电压、姿态、遥测新鲜度、舵机角、遥控按钮 |
| 右下 | 摄像头实时预览，标题显示帧率，点“收起”可隐藏 |

“实机 WAVEGO”面板：

- **遥控 ON/OFF**：关闭后不向实机发任何命令，只做本地仿真；
- **镜像 ON/OFF**：允许实机姿态镜像到孪生体（需遥测 ≥ 4Hz 才会真正生效）；
- 方向按钮与键盘等价，指令采用**锁存式**语义，只在指令变化时下发一次。

### 6.5 实机镜像的启用条件

镜像不是“收到一帧就跟随”，必须同时满足：

1. `mirrorToTwin` 打开；
2. 收到过遥测帧；
3. 遥测帧率 ≥ **4Hz**（`TelemetryStreaming`）；
4. 最近 **0.5s** 内有遥测（`TelemetryFresh`）；
5. 实机舵机角仍在变化（`IsMoving`，0.25s 窗口内单关节变化 > 1°）。

任一条件不满足就自动回退到**本地对角步态**，避免稀疏遥测把孪生体拽到某个瞬间姿态导致“整机悬空”。

镜像时还会对足端坐标做合理性校验（单位 mm）：`80 ≤ y ≤ 112` 且 `|x| ≤ 35`，
不符合的帧直接丢弃。

### 6.6 贴地补偿

每帧根据最低足端与地面（`footRadius` = 10mm）的误差闭环调整机体高度：

- 双向调节（可升可降），避免只下压导致脚陷进地面；
- 死区 1.5mm，限幅 `[-60mm, +20mm]`；
- 最终高度再限制在固件的 `WALK_HEIGHT_MIN~MAX`（75~110mm）对应范围内。

---

## 7. 部署三：ROS 2 / Isaac Sim（可选）

该工程用于关节级验证与 URDF 可视化，不影响 Unity 侧使用。

```bash
cd quadruped_digital_twin
colcon build
source install/setup.bash

# 仅 RViz 可视化
ros2 launch quadruped_twin twin.launch.py

# 12 关节指令演示（stand / sine / hold）
ros2 run quadruped_control joint_commander --ros-args -p mode:=stand

# 虚实同步桥：sim_only | mirror_real
ros2 run quadruped_twin twin_bridge --ros-args -p mode:=sim_only
```

| 包 | 作用 |
|----|------|
| `quadruped_description` | 12 自由度 URDF（关节命名与 Unity 侧一致） |
| `quadruped_control` | 发布 `/sim/joint_commands`（stand/sine/hold） |
| `quadruped_twin` | `twin_bridge`：`/real/joint_states` → `/sim/joint_commands`，并统计同步误差 |
| `quadruped_sim` | Isaac Sim 启动与 `sim.yaml` 配置 |

安装 Isaac Sim（约 10GB+）：

```bash
bash scripts/install_isaacsim.sh
```

---

## 8. 联调流程

按以下顺序启动，任何一步异常先单独排查：

1. **实机上电**：确认 ESP32 正常启动、舵机回中；
2. **树莓派联网**：`ping 192.168.67.78` 或 SSH 登录 `pi@192.168.67.78`（密码 `pi`）；
3. **启动服务**：

   ```bash
   sudo systemctl restart wavego-gateway wavego-camera
   curl -s http://192.168.67.78:5000/health
   ```

4. **PC 侧验证端口**：

   ```bash
   nc -vz 192.168.67.78 9000
   nc -vz 192.168.67.78 9001
   curl -s http://192.168.67.78:5000/snapshot.jpg -o snap.jpg
   ```

5. **Unity 打开场景并 Play**：先确认右下角出现摄像头画面；
6. **键盘测试**：在空地上按 `W`，观察实机与孪生体是否同步前进；
7. **停止测试**：松开 `W`，实机应在一个心跳周期（1s）内停止；
8. **外部 Modbus 测试**（可选）：用任意 Modbus TCP 客户端写 Holding Register 下发关节角。

---

## 9. 通信协议

### 9.1 控制命令（9000，一行一条 JSON）

```json
{"var":"move","val":1}
```

| var | val | 含义 |
|-----|-----|------|
| `move` | 1 | 前进 |
| `move` | 2 | 左转 |
| `move` | 3 | 停止前进 |
| `move` | 4 | 右转 |
| `move` | 5 | 后退 |
| `move` | 6 | 停止转向 |
| `ges` | 1 / 2 | 抬头 / 低头 |
| `ges` | 4 / 5 | 左倾 / 右倾 |
| `ges` | 3 / 6 | 姿态复位 |
| `light` | 0 / 1 | 关闭 / 打开补光灯 |
| `buzzer` | 0 / 1 | 关闭 / 打开蜂鸣器 |

命令必须以 **换行符结尾**。运动指令不要高频重发——WAVEGO 固件每收到一次 `move` 都会重新触发步态，
重复下发会让实机“一卡一卡”。本工程只在指令变化时发送，静止时每秒补发一次停止命令兜底。

### 9.2 遥测数据（9001，一行一条 JSON）

```json
{
  "type": "state",
  "t": 12345.6,
  "x": 0.0,
  "z": 0.0,
  "state": {
    "vol": 7.8,
    "servo0": 90.0, "servo1": 45.0, "servo2": 135.0,
    "servo3": 90.0, "servo4": 45.0, "servo5": 135.0,
    "servo6": 90.0, "servo7": 45.0, "servo8": 135.0,
    "servo9": 90.0, "servo10": 45.0, "servo11": 135.0,
    "ax": 0.0, "ay": 0.0, "az": 9.8,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "mx": 0.0, "my": 0.0, "mz": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0
  }
}
```

舵机通道与腿的对应关系：`servo[leg*3 + 0]` 髋侧摆、`+1` / `+2` 为闭链的两个主动摆杆。

### 9.3 摄像头（5000）

| 端点 | 返回 |
|------|------|
| `/health` | `{"backend":"opencv","ok":true}` |
| `/video_feed` | MJPEG 流，boundary=`frame`，每帧带 `Content-Length` |
| `/snapshot.jpg` | 单张 640×480 JPEG |

### 9.4 Modbus TCP 寄存器映射（1502）

所有浮点数占 **2 个寄存器**，IEEE754 大端。

**Input Registers（Unity → 客户端，只读）**

| 地址 | 内容 |
|------|------|
| 0 ~ 23 | 当前 12 关节角（deg），关节 i 起始地址 = 2i |
| 24 ~ 25 | 初始化完成标志（1.0） |
| 26 ~ 27 | 控制模式（1=本地控制，0=外部同步） |
| 28 ~ 29 | 步态（0=站立，1=trot） |
| 30 ~ 31 | 最大关节跟踪误差（deg） |
| 32 ~ 33 | 机体离地高度（m） |

**Holding Registers（客户端 → Unity，可写）**

| 地址 | 内容 |
|------|------|
| 0 ~ 23 | 目标 12 关节角（deg），关节 i 起始地址 = 2i |
| 24 ~ 25 | 步态命令（0=站立，1=trot） |
| 26 ~ 27 | 前进速度（m/s） |
| 28 ~ 29 | 转向角速度（rad/s） |
| 30 ~ 31 | 机体高度（m） |
| 32 ~ 33 | 控制模式命令（1=本地，0=外部） |
| 34 ~ 35 | 复位命令（写入 > 0.5 触发一次复位） |

关节顺序（与 URDF 一致）：

```
FL_hip, FL_thigh, FL_calf,
FR_hip, FR_thigh, FR_calf,
RL_hip, RL_thigh, RL_calf,
RR_hip, RR_thigh, RR_calf
```

---

## 10. 运动学与步态

### 10.1 腿部拓扑

WAVEGO 每条腿为**平面五杆闭链**：机架上两个独立主动铰点（相距 `LinkageS`），
连杆 A/B 为主动摆杆，C/D 为中间传递杆，E 为末端足部构件，全部旋转铰接形成闭合约束环。
Unity 侧严格按该拓扑建模，而非串联“大腿 + 小腿”。

### 10.2 关键尺寸（单位 mm）

| 参数 | 值 | 含义 |
|------|-----|------|
| `LinkageW` | 19.15 | 侧摆舵机到腿平面的距离 |
| `LinkageS` | 12.2 | 两个相邻舵机的间距 |
| `LinkageA` / `B` | 40.0 / 40.0 | 主动摆杆 |
| `LinkageC` | 39.8153 | 大腿 |
| `LinkageD` | 31.7750 | 小腿 |
| `LinkageE` | 30.8076 | 足 |
| `StandX` / `StandY` / `StandZ` | 16 / 95 / 25 | 站立时足端前后 / 竖直 / 外展偏移 |
| `WalkHeight` | 95 | 默认行走髋到足端距离 |
| `WalkHeightMin` / `Max` | 75 / 110 | 可调范围 |
| `WalkLift` | 9 | 抬腿高度 |
| `StandArmAngle` | 45° | 站立时 A/B 与竖直方向夹角 ±45° |

`QuadrupedBuilder` 中的等价参数：`footRadius = 10mm`、`hipVerticalOffset = -10mm`，
由此得到 `StandHeight = 95 + 10 - (-10) = 115mm`。

### 10.3 算法来源

正逆解完整移植 WAVEGO 官方固件的 `WigglePlaneIK`、`SingleLegPlaneIK`、`SimpleLinkageIK`，
保证 Unity 孪生体与实机运动逻辑一致：

- `FootToLinkAngles`：足端 (x, y, z) mm → 髋角 + A/B 摆杆角；
- `FiveBarForward` / `FiveBarInverse`：五杆闭链正反解；
- `ApplyJointAngles`：按解算结果摆放 A/B/C/D、胫段与足部。

### 10.4 本地步态

- 对角步态（trot）：FR + RL 一组，FL + RR 一组，相位差 0.5；
- 默认频率 1.6Hz，单腿步幅 32mm，抬腿 9mm；
- 支撑相足端相对机体向后划，摆动相抬起并回到前方，机体速度由步幅 × 频率推导，避免打滑；
- 步态中躯干叠加 3mm 正弦起伏，并持续执行贴地补偿。

---

## 11. 常见问题排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 孪生体完全不动 | 未进入 Play / 脚本编译失败 | 查看 Console 是否有 `error CS`；重新 Play |
| 按键盘无反应 | 面板“遥控 OFF”或处于外部同步模式 | 打开遥控 ON；`Tab` 切回本地控制 |
| 走不到一秒整机悬空 | 稀疏遥测触发镜像 | 已加 ≥4Hz 门控；确认 `TelemetryHz`，不足时自动走本地步态 |
| 实机行走一顿一顿 | 高频重复下发 `move` | 只在指令变化时发送；检查是否有其它客户端在发命令 |
| 松开按键实机不停 | 停止命令丢包 | 系统每秒补发一次停止命令，检查 WiFi 是否丢包 |
| 孪生体脚陷进地面 | 贴地补偿只许下压 | 已改为双向 + 死区 + 限幅 |
| 摄像头带宽被打满 | 旧版 MJPEG 重复帧 | 按 [5.4](#54-部署摄像头服务) 打补丁并重启服务 |
| 摄像头画面卡顿 | WiFi 信号弱 / 分辨率高 | 降低 `--quality` 或 `--fps`；靠近路由器 |
| 遥测 1 帧/十几秒 | ESP32 固件上报稀疏（约 0.1Hz） | 升级固件提升上报频率；不足 4Hz 时只能用本地步态 |
| 9001 连上后很快断开 | 端口为单客户端独占 | 关闭其它订阅者（调试脚本、ROS 桥等） |
| 无法连接 9000/9001 | IP 变化 / 不同网段 / 防火墙 | `ping` 确认；在面板改地址重连；检查防火墙 |
| Unity 启动报 VR 模块错误 | `manifest.json` 残留 `com.unity.modules.vr` | 删除该依赖 |
| Unity 编译警告 collab-proxy | Unity 6 兼容性问题 | 移除 `com.unity.collab-proxy` |
| `systemctl` 报 bus Connection refused | 树莓派 systemd 用户总线异常 | `sudo systemctl daemon-reexec` 或重启；临时用 `setsid nohup` 拉起 |
| GitHub 22 端口连不上 | 网络环境拦截 22 | 在 `~/.ssh/config` 中让 `github.com` 走 `ssh.github.com:443` |

---

## 12. 已知限制

1. **没有雷达**：WAVEGO / WAVEGO Pro 官方配置只有摄像头、ICM20948、RGB、OLED、INA219 与蜂鸣器，
   唯一扩展口为 2×5P（引出 ESP32 RX0/TX0、G21/G15/G12、3V3/5V/GND），无现成雷达模块与协议。
   因此 Unity 端暂未实现雷达障碍物可视化。
2. **实机遥测帧率低**：当前 ESP32 固件空闲时约 0.1Hz，无法支撑连续姿态镜像；
   工程会在帧率不足时自动回退本地步态。
3. **9001 单客户端独占**：Unity 与调试工具不能同时订阅遥测。
4. **摄像头 12fps / 640×480**：受树莓派与 WiFi 带宽限制，暂不做更高分辨率推流。
5. **镜像合理性窗口**：足端 y 限制在 80~112mm、|x| ≤ 35mm，极端姿态会被丢弃。
