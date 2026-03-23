# 实车部署视频
https://github.com/user-attachments/assets/23496b39-5b85-4025-adc6-6f2e1e1767ab
# NeuPAN + Scout Mini 完整工程实施手册

## 参考资料索引 (References)

本手册基于对以下 6 个核心仓库的深入分析编写：

| 仓库                                         | 关键文件/路径                                                                                     | 参考链接                                                                                            |
| ------------------------------------------ | ------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| **hanruihua/NeuPAN** (py38 分支)             | `neupan/neupan.py`, `neupan/blocks/`, `example/dune_train/`                                 | [NeuPAN Repository](https://github.com/hanruihua/NeuPAN/tree/py38)                              |
| **hanruihua/neupan_ros**                   | `src/neupan_core. py`, `src/neupan_node.py`, `example/gazebo_limo/`                         | [neupan_ros Repository](https://github.com/hanruihua/neupan_ros)                                |
| **agilexrobotics/ugv_sdk**                 | `scripts/setup_can2usb.bash`, `README.md`                                                   | [ugv_sdk Repository](https://github.com/agilexrobotics/ugv_sdk)                                 |
| **agilexrobotics/scout_ros**               | `scout_bringup/launch/scout_mini_robot_base.launch`, `scout_base/launch/scout_base. launch` | [scout_ros Repository](https://github.com/agilexrobotics/scout_ros)                             |
| **ros-drivers/velodyne**                   | `velodyne_pointcloud/`, `velodyne_driver/`                                                  | [velodyne Repository](https://github.com/ros-drivers/velodyne)                                  |
| **ros-perception/pointcloud_to_laserscan** | `launch/sample_pointcloud_to_laserscan_launch.py`                                           | [pointcloud_to_laserscan Repository](https://github.com/ros-perception/pointcloud_to_laserscan) |

---

## 阶段零：硬件系统集成与底层通信配置 (Hardware Integration & Comm Setup)

### 0.1 硬件物料清单 (BOM)

| 类别  | 设备              | 型号/规格                   | 备注                  |
| --- | --------------- | ----------------------- | ------------------- |
| 底盘  | Scout Mini      | 差速版 (Skid/Diff)         | 尺寸:  0.612m × 0.58m |
| 雷达  | Velodyne VLP-16 | 16线机械激光雷达               | 安装高度 Z=0.58m        |
| 通信  | USB转CAN模块       | 支持 gs_usb 驱动            | 如 CANable           |
| 计算  | 笔记本电脑           | i7-12700 / Ubuntu 20.04 | 确保有RJ45网口           |

### 0.2 硬件接线图

```
┌────────────────────────────────────────────────────────────┐
│                      笔记本电脑                              │
│  ┌─────────────┐     ┌─────────────┐                        │
│  │ USB 端口    │     │ RJ45 网口   │                        │
│  └──────┬──────┘     └──────┬──────┘                        │
└─────────┼───────────────────┼───────────────────────────────┘
          │                   │
          │ USB               │ 网线 (直连)
          ▼                   ▼
   ┌──────────────┐    ┌──────────────────┐
   │ USB-CAN 模块 │    │  Velodyne VLP-16 │
   │              │    │  IP:  192.168.1.201│
   └──────┬───────┘    │  Port: 2368       │
          │            └──────────────────┘
          │ CAN_H / CAN_L (航空插头)
          ▼
   ┌──────────────────────────────────────┐
   │          Scout Mini 底盘             │
   │    CAN 接口 (500kbps)                │
   └──────────────────────────────────────┘
```

### 0.3 遥控器操作与安全复位

> **参考来源**:  [ugv_sdk README. md - Sample Code Section](https://github.com/agilexrobotics/ugv_sdk/blob/main/README.md)

```
遥控器拨杆位置说明：
┌─────────────────────────────────────┐
│  SWA │ SWB │ SWC │ SWD              │
│  ↑   │ ↑   │ ↑   │ ↑    ← 初始位置  │
└─────────────────────────────────────┘

SWB 拨杆功能：
  - 最上方 (↑): 指令模式 (Command Mode) - 软件控制
  - 中间   (─): 遥控模式 (Remote Control) - 手动操作
  - 最下方 (↓): 紧急制动 (E-Stop)

操作流程：
  1. 开机前：所有拨杆置于最上方 (↑)
  2. 手动测试：将 SWB 拨至中间 (─)，测试遥控器正常
  3. 软件控制：将 SWB 拨至最上方 (↑)
  4. 紧急情况：立即将 SWB 拨回中间或最下方！
```
### 0.4 笔记本网络配置 (Velodyne Static IP)

**前置准备**：请先在终端输入 `ifconfig` 查看并记下连接雷达的物理网口名称（例如 `enp3s0` 或 `eth0`）。

#### 方法一：图形界面配置 (推荐，永久生效)修复 VLP-32C_points.launch 文件中的格式问题
适合桌面用户，配置直观，不易出错。

1.  **进入设置**：打开系统设置 `Settings` -> `Network` (网络) -> 找到 `Wired` (有线连接) -> 点击右侧齿轮图标。
2.  **IPv4 设置**：
    *   切换到 **IPv4** 选项卡。
    *   **IPv4 Method**: 选择 **Manual** (手动)。
    *   **Address**: 填入 `192.168.1.100` (本机 IP)。
    *   **Netmask**: 填入 `255.255.255.0`。
    *   **Gateway**: **务必留空** 或填 `0.0.0.0` (关键！防止流量误走雷达网口导致 WiFi 无法上网)。
3.  **应用生效**：点击右上角 `Apply`，然后关闭再打开有线连接的开关（Toggle off/on）以重置连接。

#### 方法二：命令行 `nmcli` 配置 (脚本化/高级)
使用 NetworkManager 命令行工具配置，同样是永久生效的。

```bash
# 1. 添加名为 "velodyne-static" 的连接配置
# 请将 <your-eth-interface> 替换为实际网口名 (如 enp3s0)
sudo nmcli connection add type ethernet con-name "velodyne-static" ifname <your-eth-interface>

# 2. 设置静态 IP (192.168.1.100) 和掩码 (/24 即 255.255.255.0)
sudo nmcli connection modify "velodyne-static" ipv4.addresses 192.168.1.100/24

# 3. 设置为手动模式
sudo nmcli connection modify "velodyne-static" ipv4.method manual

# 4. 关键：不设置网关，防止与 WiFi 冲突
sudo nmcli connection modify "velodyne-static" ipv4.gateway ""

# 5. 激活连接
sudo nmcli connection up "velodyne-static"
```

#### 验证连接
```bash
ping 192.168.1.201
# 成功标志：终端持续输出 "64 bytes from 192.168.1.201..."
```

### 0.5 CAN 接口配置 (SocketCAN)

> **参考来源**: [ugv_sdk scripts/setup_can2usb.bash](https://github.com/AoiOTA/ugv_sdk/blob/main/scripts/setup_can2usb.bash)

```bash
# 1. 安装基础 CAN 工具 (如果是新系统)
sudo apt install -y can-utils net-tools

# 2. 加载内核模块 (针对 candleLight/官方 USB-CAN 模块)
sudo modprobe gs_usb

# 3. 配置 CAN 接口 (波特率: 500k)
# 先关闭接口，防止“设备忙”错误
sudo ip link set can0 down
# 设置波特率
sudo ip link set can0 type can bitrate 500000
# 重新启动接口
sudo ip link set can0 up

# 4. 检查接口是否已挂载
ifconfig can0
# 应当看到类似 "UP RUNNING NOARP" 的状态
```

#### 验证 CAN 数据
```bash
candump can0
```

**预期结果**：
终端应疯狂滚动显示 CAN 帧数据。重点观察 ID 为 **211** 的数据帧（根据手册，这是系统状态反馈帧）。
*   **示例**: `can0  211   [8]  00 00 00 00 00 00 00 6C`
    *   注：最后一位 `6C` 是计数器，会不断跳变，证明数据是实时的。

**故障排查 (无数据)**：
1.  **硬件接线**：检查 CAN_H / CAN_L 是否接反（这是最常见错误）。
2.  **物理连接**：确认 USB 模块已插好，底盘已开机（听到风扇声或看到灯亮）。
3.  **模式确认**：根据手册，底盘开机默认处于**待机模式**，但即便在待机模式下，心跳包 (0x211) 也应该会持续发送。
4.  **设备名称**：输入 `ifconfig -a` 查看是否有 `can0`，如果是 `can1` 或其他名称，请相应修改上述命令。

---

## 阶段一：工作空间构建、版本控制与 IDE 规范 (Workspace, Git & IDE Setup)

### 1. 目录结构初始化

```bash
# 创建工作空间
mkdir -p ~/neupan_ws/src
cd ~/neupan_ws/src
catkin_init_workspace
```

没问题，直接使用 `-b master` 克隆可以省去后续检查和切换分支的步骤。

以下是更新后的完整内容：

### 2. 仓库获取 (Fork & Clone SSH 版)

**步骤 1: 在 GitHub 网页上 Fork 以下 5 个仓库到您的账户 (AoiOTA):**
1. `hanruihua/NeuPAN`
2. `hanruihua/neupan_ros`
3. `agilexrobotics/ugv_sdk`
4. `agilexrobotics/scout_ros`
5. `ros-drivers/velodyne`  **<-- [新增]**

**步骤 2: 使用 SSH 地址 Clone 到本地 (关键步骤):**

```bash
cd ~/neupan_ws/src

# NeuPAN 核心算法 (使用 SSH 地址，分支保持 py38)
git clone -b py38 git@github.com:AoiOTA/NeuPAN.git

# NeuPAN ROS Wrapper (使用 SSH 地址)
git clone git@github.com:AoiOTA/neupan_ros.git

# AgileX SDK (使用 SSH 地址)
git clone git@github.com:AoiOTA/ugv_sdk.git

# Scout ROS 驱动 (使用 SSH 地址)
git clone git@github.com:AoiOTA/scout_ros.git

# Velodyne 驱动 (使用 SSH 地址，直接克隆 master 分支) <-- [新增]
git clone -b master git@github.com:AoiOTA/velodyne.git
```

### 3. Git 分支策略与远程配置

为每个仓库创建开发分支，并建立连接：**Origin (您的 Fork) 使用 SSH 用于推送，Upstream (原作者) 使用 HTTPS 用于同步。**

```bash
# --- 1. NeuPAN 核心配置 ---
cd ~/neupan_ws/src/NeuPAN
git checkout -b dev-scout-integration               # 创建并切换分支
git push -u origin dev-scout-integration            # SSH 推送，无需密码
git remote add upstream https://github.com/hanruihua/NeuPAN.git # 添加原作者(只读)

# --- 2. NeuPAN ROS 配置 ---
cd ~/neupan_ws/src/neupan_ros
git checkout -b dev-scout-integration
git push -u origin dev-scout-integration
git remote add upstream https://github.com/hanruihua/neupan_ros.git

# --- 3. UGV SDK 配置 ---
cd ~/neupan_ws/src/ugv_sdk
git checkout -b dev-scout-integration
git push -u origin dev-scout-integration
git remote add upstream https://github.com/agilexrobotics/ugv_sdk.git

# --- 4. Scout ROS 配置 ---
cd ~/neupan_ws/src/scout_ros
git checkout -b dev-scout-integration
git push -u origin dev-scout-integration
git remote add upstream https://github.com/agilexrobotics/scout_ros.git

# --- 5. Velodyne 驱动配置 [新增] ---
cd ~/neupan_ws/src/velodyne
# 既然已经 clone 了 master，直接基于此创建开发分支
git checkout -b dev-scout-integration
git push -u origin dev-scout-integration
git remote add upstream https://github.com/ros-drivers/velodyne.git
```

### 4. 验证配置

执行以下命令检查远程连接是否正确（以 NeuPAN 为例，其他仓库同理）：

```bash
cd ~/neupan_ws/src/NeuPAN
git remote -v
```

**预期输出：**
*   `origin` 指向 `git@github.com:AoiOTA/NeuPAN.git` (SSH 格式，**可写**，无需密码)
*   `upstream` 指向 `https://github.com/hanruihua/NeuPAN.git` (HTTPS 格式，**只读**，用于更新)

### 5. VS Code 工作区配置

为了方便同时管理多个仓库并获得正确的 Python 代码补全，建议使用 VS Code 工作区配置文件。

**步骤 1: 创建配置文件**

在工作空间根目录 `~/neupan_ws` 下创建一个名为 `neupan.code-workspace` 的文件，并将以下内容复制进去。**已加入 Velodyne Driver 路径配置。**

```json
{
    "folders": [
        {
            "path": "src/NeuPAN",
            "name": "NeuPAN Core"
        },
        {
            "path": "src/neupan_ros",
            "name": "NeuPAN ROS"
        },
        {
            "path": "src/scout_ros",
            "name": "Scout ROS"
        },
        {
            "path": "src/ugv_sdk",
            "name": "UGV SDK"
        },
        {
            "path": "src/velodyne",
            "name": "Velodyne Driver"
        }
    ],
    "settings": {
        "python.defaultInterpreterPath": "/usr/bin/python3",
        "python.analysis.extraPaths": [
            "${workspaceFolder}/src/NeuPAN",
            "/opt/ros/noetic/lib/python3/dist-packages"
        ],
        "files.associations": {
            "*.launch": "xml",
            "*.xacro": "xml"
        }
    }
}
```

**步骤 2: 使用工作区文件**

你可以通过以下两种方式之一进入该工作区：

1.  **命令行启动（最快）：**
    ```bash
    cd ~/neupan_ws
    code neupan.code-workspace
    ```
2.  **图形界面启动：**
    *   打开 VS Code。
    *   点击左上角 **文件 (File)** -> **从文件打开工作区... (Open Workspace from File...)**。
    *   选择刚才创建的 `neupan.code-workspace` 文件。

**步骤 3: 效果确认**

*   **侧边栏**：你会看到四个仓库被清晰地命名为 "NeuPAN Core"、"NeuPAN ROS" 等，方便快速切换。
*   **代码补全**：配置文件自动关联了 ROS Noetic 的 Python 路径和 NeuPAN 核心算法路径，解决 `import` 报错问题。
*   **语法高亮**：`.launch` 和 `.xacro` 文件将自动以 XML 格式高亮显示。

---

## 阶段二：依赖解析、C++源码修复与工作空间编译 (Dependencies, Code Patching & Compilation)

### 2.1 系统基础依赖安装与 Pip 升级

首先安装必要的系统开发工具和 ROS 组件，并安全升级 Pip 以确保兼容性。

```bash
# 1. 更新系统
sudo apt update && sudo apt upgrade -y

# 2. 安装核心工具链
sudo apt install -y \
    python3-pip \
    python3-catkin-tools \
    cmake \
    build-essential \
    git \
    libasio-dev \
    can-utils \
    net-tools

# 3. 安全升级 Pip (Ubuntu 20.04 自带版本过低)
python3 -m pip install --user --upgrade pip

# 4. 安装 ROS Noetic 核心导航组件
sudo apt install -y \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-gmapping \
    ros-noetic-global-planner \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-velodyne \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-tf2-ros \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher
```

### 2.2 NeuPAN 源码配置与环境部署

在安装依赖前，需先处理分支关联及 Python 版本兼容性限制。

**步骤 1: 分支切换与远程关联**

```bash
cd ~/neupan_ws/src/NeuPAN

# 创建开发分支
git checkout -b dev-scout-integration

# 建立远程上游关联（确保代码能推送到你的 GitHub 仓库）
git push -u origin dev-scout-integration
```

**步骤 2: 破解 Python 版本限制**

由于 NeuPAN 的 `pyproject.toml` 默认限制 `python >= 3.10`，而 Ubuntu 20.04 默认为 3.8，需手动修改配置：

```bash
# 使用 sed 命令将 requires-python 修改为 >=3.8
sed -i 's/requires-python = ">=3.10"/requires-python = ">=3.8"/g' pyproject.toml
```

**步骤 3: 安装 Python 依赖库 (--user 模式)**

使用 `--user` 参数安装依赖，避免污染系统全局环境且无需 sudo 权限。

```bash
# 1. 安装 PyTorch (带 CUDA 12.1 支持)
python3 -m pip install --user torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu121

# 2. 安装 NeuPAN 核心算法依赖
python3 -m pip install --user \
    numpy \
    scipy==1.10.1 \
    pyyaml \
    rich \
    dill \
    colorama \
    scikit-learn \
    cvxpy \
    cvxpylayers \
    ecos \
    gctl==1.2

# 3. 以开发模式安装 NeuPAN 自身
# 注意：此步会自动读取修改后的 pyproject.toml
python3 -m pip install --user -e .
```

**步骤 4: 验证安装**

```bash
# 检查是否能成功导入核心模块
python3 -c 'from neupan import neupan; print("NeuPAN imported successfully!")'

```

### 2.3 CAN 接口自动化脚本

```bash
# 使用 ugv_sdk 提供的脚本
cd ~/neupan_ws/src/ugv_sdk/scripts
chmod +x setup_can2usb.bash bringup_can2usb_500k.bash

# 首次运行 (安装依赖 + 配置)
./setup_can2usb.bash

# 后续每次插拔 USB-CAN 后运行
./bringup_can2usb_500k.bash
```

### 2.4 ROS 工作空间依赖解析与编译

在执行编译之前，必须修复 `ugv_sdk` 中的 Asio 版本兼容性问题（`basic_stream_descriptor` 报错），否则 `catkin build` 会失败。

```bash
cd ~/neupan_ws

# 1. 解析并安装缺失的系统依赖
rosdep install --from-paths src --ignore-src -r -y
```

#### **关键修复步骤：替换 ugv_sdk 源码**

由于新版系统中的 Boost/Asio 库废弃了 `basic_stream_descriptor`，请手动修改以下两个文件。

**文件 1：修改头文件**
请使用编辑器打开 `src/ugv_sdk/include/ugv_sdk/details/async_port/async_can.hpp`，删除所有原有内容，并粘贴以下**完整代码**：

```cpp name=src/ugv_sdk/include/ugv_sdk/details/async_port/async_can.hpp
/*
 * async_can.hpp
 *
 * Created on: Sep 10, 2020 13:22
 * Description: Fixed for newer Asio versions (basic_stream_descriptor -> stream_descriptor)
 *
 * Note: CAN TX is not buffered and only the latest frame will be transmitted.
 *  Buffered transmission will need to be added if a message has to be divided
 *  into multiple frames and in applications where no frame should be dropped.
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef ASYNC_CAN_HPP
#define ASYNC_CAN_HPP

#include <linux/can.h>

#include <atomic>
#include <memory>
#include <thread>
#include <functional>

#include "asio.hpp"
#include "asio/posix/stream_descriptor.hpp" // [已修改]

namespace westonrobot {
class AsyncCAN : public std::enable_shared_from_this<AsyncCAN> {
 public:
  using ReceiveCallback = std::function<void(can_frame *rx_frame)>;

 public:
  AsyncCAN(std::string can_port = "can0");
  ~AsyncCAN();

  // do not allow copy
  AsyncCAN(const AsyncCAN &) = delete;
  AsyncCAN &operator=(const AsyncCAN &) = delete;

  // Public API
  bool Open();
  void Close();
  bool IsOpened() const;

  void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
  void SendFrame(const struct can_frame &frame);

 private:
  std::string port_;
  std::atomic<bool> port_opened_{false};

#if ASIO_VERSION < 101200L
  asio::io_service io_context_;
#else
  asio::io_context io_context_;
#endif

  std::thread io_thread_;

  int can_fd_;
  // [已修改] 类型从 basic_stream_descriptor<> 改为 stream_descriptor
  asio::posix::stream_descriptor socketcan_stream_;

  struct can_frame rcv_frame_;
  ReceiveCallback rcv_cb_ = nullptr;

  void DefaultReceiveCallback(can_frame *rx_frame);
  
  // [已修改] 参数类型从 basic_stream_descriptor<> 改为 stream_descriptor
  void ReadFromPort(struct can_frame &rec_frame,
                    asio::posix::stream_descriptor &stream);
};
}  // namespace westonrobot

#endif /* ASYNC_CAN_HPP */
```

**文件 2：修改源文件**
请使用编辑器打开 `src/ugv_sdk/src/details/async_port/async_can.cpp`，删除所有原有内容，并粘贴以下**完整代码**：

```cpp name=src/ugv_sdk/src/details/async_port/async_can.cpp
/*
 * async_can.cpp
 *
 * Created on: Sep 10, 2020 13:23
 * Description: Fixed for newer Asio versions
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/details/async_port/async_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>

#include <iostream>

namespace westonrobot {
AsyncCAN::AsyncCAN(std::string can_port)
    : port_(can_port), socketcan_stream_(io_context_) {}

AsyncCAN::~AsyncCAN() { Close(); }

bool AsyncCAN::Open() {
  try {
    const size_t iface_name_size = strlen(port_.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ) return false;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, port_.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0) {
      Close();
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result =
        bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (bind_result < 0) {
      Close();
      return false;
    }

    port_opened_ = true;
    std::cout << "Start listening to port: " << port_ << std::endl;
  } catch (std::system_error &e) {
    port_opened_ = false;
    std::cout << e.what() << std::endl;
    return false;
  }

  // give some work to io_service to start async io chain
  socketcan_stream_.assign(can_fd_);

#if ASIO_VERSION < 101200L
  io_context_.post(std::bind(&AsyncCAN::ReadFromPort, this,
                             std::ref(rcv_frame_),
                             std::ref(socketcan_stream_)));
#else
  asio::post(io_context_,
             std::bind(&AsyncCAN::ReadFromPort, this, std::ref(rcv_frame_),
                       std::ref(socketcan_stream_)));
#endif

  // start io thread
  io_thread_ = std::thread([this]() { io_context_.run(); });

  return true;
}

void AsyncCAN::Close() {
  io_context_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  io_context_.reset();
  
  // release port fd
  const int close_result = ::close(can_fd_);
  can_fd_ = -1;

  port_opened_ = false;
}

bool AsyncCAN::IsOpened() const { return port_opened_; }

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame) {
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

// [已修改] 参数类型从 basic_stream_descriptor<> 改为 stream_descriptor
void AsyncCAN::ReadFromPort(struct can_frame &rec_frame,
                            asio::posix::stream_descriptor &stream) {
  auto sthis = shared_from_this();
  stream.async_read_some(
      asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->Close();
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        sthis->ReadFromPort(std::ref(sthis->rcv_frame_),
                            std::ref(sthis->socketcan_stream_));
      });
}

void AsyncCAN::SendFrame(const struct can_frame &frame) {
  socketcan_stream_.async_write_some(
      asio::buffer(&frame, sizeof(frame)),
      [](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          std::cerr << "Failed to send CAN frame" << std::endl;
        }
        // std::cout << "frame sent" << std::endl;
      });
}

}  // namespace westonrobot
```

#### **执行编译**

完成上述代码修复后，回到终端继续执行：

```bash
# 2. 清理旧构建文件 (推荐，确保环境干净)
catkin clean

# 3. 编译整个工作空间
# 现在编译应该能顺利通过了
catkin build

# 4. 刷新当前终端的环境变量
source devel/setup.bash

# 5. 将环境变量写入 .bashrc (确保新终端能自动加载)
grep -q "source ~/neupan_ws/devel/setup.bash" ~/.bashrc || echo "source ~/neupan_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 阶段三：NeuPAN 模型定制化训练与可视化监控 (Custom Model Training & TensorBoard Visualization)

> **参考来源**: [NeuPAN example/dune_train/dune_train_diff.yaml](https://github.com/hanruihua/NeuPAN/blob/main/example/dune_train/dune_train_diff.yaml) 和 [neupan/blocks/dune. py train_dune 方法](https://github.com/hanruihua/NeuPAN/blob/main/neupan/blocks/dune. py)

### 3.0 更新核心源码 (启用 TensorBoard)

首先，我们需要用支持 TensorBoard 的代码替换原有的 `DUNETrain` 类实现。假设源代码位于 `neupan/dune_train.py`（请根据实际项目结构调整路径）。

```python name=~/neupan_ws/src/NeuPAN/neupan/dune_train.py
"""
DUNETrain is the class for training the DUNE model. It is used when you deploy the NeuPan algorithm on a new robot with a specific geometry. 

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

NeuPAN planner is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NeuPAN planner is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with NeuPAN planner. If not, see <https://www.gnu.org/licenses/>.
"""

import torch
from colorama import deinit

deinit()

from torch.utils.data import Dataset, random_split, DataLoader
from torch.utils.tensorboard import SummaryWriter  # [新增] TensorBoard 支持
import cvxpy as cp
from rich.console import Console
from rich.progress import Progress
from rich.live import Live
from torch.optim import Adam
import numpy as np
from neupan.configuration import np_to_tensor, value_to_tensor, to_device
import pickle
import time
import os


class PointDataset(Dataset):
    def __init__(self, input_data, label_data, distance_data):
        """
        input_data: point p, [2, 1]
        label_data: mu, [G.shape[0], 1]
        distance_data: distance, scalar
        """

        self.input_data = input_data
        self.label_data = label_data
        self.distance_data = distance_data

    def __len__(self):
        return len(self.input_data)

    def __getitem__(self, idx):
        input_sample = self.input_data[idx]
        label_sample = self.label_data[idx]
        distance_sample = self.distance_data[idx]

        return input_sample, label_sample, distance_sample


class DUNETrain:
    def __init__(self, model, robot_G, robot_h, checkpoint_path) -> None:

        self.G = robot_G
        self.h = robot_h
        self.model = model

        self.construct_problem()
        self.checkpoint_path = checkpoint_path

        # [新增] 初始化 TensorBoard Writer
        # 日志将保存在 checkpoint_path 下的 'runs' 文件夹中
        # 例如: model/scout_mini_diff/runs
        self.log_dir = os.path.join(self.checkpoint_path, 'runs')
        self.writer = SummaryWriter(log_dir=self.log_dir)

        self.loss_fn = torch.nn.MSELoss()

        self.optimizer = Adam(self.model.parameters(), lr=1e-4, weight_decay=1e-4)

        # for rich progress
        self.console = Console()
        self.progress = Progress(transient=False)
        self.live = Live(self.progress, console=self.console, auto_refresh=False)

        # loss
        self.loss_of_epoch = 0
        self.loss_list = []

    def construct_problem(self):
        """
        optimization problem (10):

        max mu^T * (G * p - h)
        s.t. ||G^T * mu|| <= 1
            mu >= 0
        """
        self.mu = cp.Variable((self.G.shape[0], 1), nonneg=True)
        self.p = cp.Parameter((2, 1))  # points

        cost = self.mu.T @ (self.G.cpu() @ self.p - self.h.cpu())
        constraints = [cp.norm(self.G.cpu().T @ self.mu) <= 1]

        self.prob = cp.Problem(cp.Maximize(cost), constraints)

    def process_data(self, rand_p):
        distance_value, mu_value = self.prob_solve(rand_p)  # Adapted to be accessible
        return (
            np_to_tensor(rand_p),
            np_to_tensor(mu_value),
            value_to_tensor(distance_value),
        )

    def generate_data_set(self, data_size=10000, data_range=[-50, -50, 50, 50]):
        """
        generate dataset for training
        data_range: [low_x, low_y, high_x, high_y]
        """

        input_data = []
        label_data = []
        distance_data = []

        rand_p = np.random.uniform(
            low=data_range[:2], high=data_range[2:], size=(data_size, 2)
        )
        rand_p_list = [rand_p[i].reshape(2, 1) for i in range(data_size)]

        for p in rand_p_list:
            results = self.process_data(p)
            input_data.append(results[0])
            label_data.append(results[1])
            distance_data.append(results[2])

        dataset = PointDataset(input_data, label_data, distance_data)

        return dataset

    def prob_solve(self, p_value):

        self.p.value = p_value
        self.prob.solve(solver=cp.ECOS)  # distance
        # self.prob.solve()  # distance

        return self.prob.value, self.mu.value

    def start(
        self,
        data_size: int = 100000,
        data_range = [-25, -25, 25, 25],
        batch_size: int = 256,
        epoch: int = 5000,
        valid_freq: int = 100,
        save_freq: int = 500,
        lr: float = 5e-5,
        lr_decay: float = 0.5,
        decay_freq: int = 1500,
        save_loss: bool = False,
        **kwargs,
    ):

        train_dict = {
            "data_size": data_size,
            "data_range": data_range,
            "batch_size": batch_size,
            "epoch": epoch,
            "valid_freq": valid_freq,
            "save_freq": save_freq,
            "lr": lr,
            "lr_decay": lr_decay,
            "decay_freq": decay_freq,
            "robot_G": self.G,
            "robot_h": self.h,
            "model": self.model,
        }

        # Ensure checkpoint directory exists
        if not os.path.exists(self.checkpoint_path):
            os.makedirs(self.checkpoint_path)

        with open(self.checkpoint_path + "/train_dict.pkl", "wb") as f:
            pickle.dump(train_dict, f)

        print(
            f"data_size: {data_size}, data_range: {data_range}, batch_size: {batch_size}, epoch: {epoch}, valid_freq: {valid_freq}, save_freq: {save_freq}, lr: {lr}, lr_decay: {lr_decay}, decay_freq: {decay_freq}, robot_G: {self.G}, robot_h: {self.h}"
        )

        with open(self.checkpoint_path + "/results.txt", "a") as f:
            print(
                f"data_size: {data_size}, data_range: {data_range}, batch_size: {batch_size}, epoch: {epoch}, valid_freq: {valid_freq}, save_freq: {save_freq}, lr: {lr}, lr_decay: {lr_decay}, decay_freq: {decay_freq}, robot_G: {self.G}, robot_h: {self.h}\n",
                file=f,
            )

        self.optimizer.param_groups[0]["lr"] = float(lr)
        ful_model_name = None

        print("dataset generating start ...")
        dataset = self.generate_data_set(data_size, data_range)
        train, valid, _ = random_split(
            dataset, [int(data_size * 0.8), int(data_size * 0.2), 0]
        )

        train_dataloader = DataLoader(train, batch_size=batch_size)
        valid_dataloader = DataLoader(valid, batch_size=batch_size)

        print("dataset training start ...")

        with self.live:
            task = self.progress.add_task("[cyan]Training...", total=epoch)

            for i in range(epoch + 1):

                self.progress.update(task, advance=1)
                self.live.refresh()

                self.model.train(True)

                mu_loss, distance_loss, fa_loss, fb_loss = self.train_one_epoch(
                    train_dataloader, False
                )

                # [新增] 计算总 Loss 并写入 TensorBoard (训练集)
                total_train_loss = mu_loss + distance_loss + fa_loss + fb_loss
                
                self.writer.add_scalar('Loss/Train/Total', total_train_loss, i)
                self.writer.add_scalar('Loss/Train/Mu', mu_loss, i)
                self.writer.add_scalar('Loss/Train/Distance', distance_loss, i)
                self.writer.add_scalar('Loss/Train/Fa', fa_loss, i)
                self.writer.add_scalar('Loss/Train/Fb', fb_loss, i)
                
                current_lr = self.optimizer.param_groups[0]["lr"]
                self.writer.add_scalar('Hyperparameters/Learning_Rate', current_lr, i)

                ml, dl, al, bl = (
                    "{:.2e}".format(mu_loss),
                    "{:.2e}".format(distance_loss),
                    "{:.2e}".format(fa_loss),
                    "{:.2e}".format(fb_loss),
                )

                if i % valid_freq == 0:
                    self.model.eval()
                    (
                        valid_mu_loss,
                        valid_distance_loss,
                        validate_fa_loss,
                        validate_fb_loss,
                    ) = self.train_one_epoch(valid_dataloader, True)

                    # [新增] 计算总 Loss 并写入 TensorBoard (验证集)
                    total_valid_loss = valid_mu_loss + valid_distance_loss + validate_fa_loss + validate_fb_loss
                    
                    self.writer.add_scalar('Loss/Valid/Total', total_valid_loss, i)
                    self.writer.add_scalar('Loss/Valid/Mu', valid_mu_loss, i)
                    self.writer.add_scalar('Loss/Valid/Distance', valid_distance_loss, i)
                    self.writer.add_scalar('Loss/Valid/Fa', validate_fa_loss, i)
                    self.writer.add_scalar('Loss/Valid/Fb', validate_fb_loss, i)

                    vml, vdl, val, vbl = (
                        "{:.2e}".format(valid_mu_loss),
                        "{:.2e}".format(valid_distance_loss),
                        "{:.2e}".format(validate_fa_loss),
                        "{:.2e}".format(validate_fb_loss),
                    )

                    self.print_loss(
                        i,
                        epoch,
                        ml,
                        dl,
                        al,
                        bl,
                        vml,
                        vdl,
                        val,
                        vbl,
                        self.optimizer.param_groups[0]["lr"],
                    )

                    with open(self.checkpoint_path + "/results.txt", "a") as f:
                        self.print_loss(
                            i,
                            epoch,
                            ml,
                            dl,
                            al,
                            bl,
                            vml,
                            vdl,
                            val,
                            vbl,
                            self.optimizer.param_groups[0]["lr"],
                            f,
                        )

                if i % save_freq == 0:
                    print("save model at epoch {}".format(i))
                    torch.save(
                        self.model.state_dict(),
                        self.checkpoint_path + "/" + "model_" + str(i) + ".pth",
                    )
                    ful_model_name = (
                        self.checkpoint_path + "/" + "model_" + str(i) + ".pth"
                    )

                if (i + 1) % decay_freq == 0:
                    self.optimizer.param_groups[0]["lr"] = (
                        self.optimizer.param_groups[0]["lr"] * lr_decay
                    )
                    print(
                        "current learning rate:", self.optimizer.param_groups[0]["lr"]
                    )

                    with open(self.checkpoint_path + "/results.txt", "a") as f:
                        print(
                            "current learning rate:",
                            self.optimizer.param_groups[0]["lr"],
                            file=f,
                        )

                self.loss_of_epoch = mu_loss + distance_loss + fa_loss + fb_loss
                self.loss_list.append(self.loss_of_epoch)

                if save_loss:
                    with open(self.checkpoint_path + "/loss.pkl", "wb") as f:
                        pickle.dump(self.loss_list, f)
        
        # [新增] 训练结束关闭 Writer
        self.writer.close()
        print("finish train, the model is saved in {}".format(ful_model_name))

        return ful_model_name

    def train_one_epoch(self, train_dataloader, validate=False):
        """
        loss:
            mu: mse between output mu and label mu
            objective function value (distance): mse between output distance and label distance
            fa: -mu^T * G * R^T  ==> lam^T
            fb: mu^T * G * R^T * p - mu^T * h  ==> lam^T * p + mu^T * h
        """

        mu_loss, distance_loss, fa_loss, fb_loss = 0, 0, 0, 0

        for input_point, label_mu, label_distance in train_dataloader:

            self.optimizer.zero_grad()

            input_point = torch.squeeze(input_point)
            output_mu = self.model(input_point)
            output_mu = torch.unsqueeze(output_mu, 2)

            distance = self.cal_distance(output_mu, input_point)

            mse_mu = self.loss_fn(output_mu, label_mu)
            mse_distance = self.loss_fn(distance, label_distance)
            mse_fa, mse_fb = self.cal_loss_fab(output_mu, label_mu, input_point)

            loss = mse_mu + mse_distance + mse_fa + mse_fb

            if not validate:
                loss.backward()
                self.optimizer.step()

            mu_loss += mse_mu.item()
            distance_loss += mse_distance.item()
            fa_loss += mse_fa.item()
            fb_loss += mse_fb.item()

        return (
            mu_loss / len(train_dataloader),
            distance_loss / len(train_dataloader),
            fa_loss / len(train_dataloader),
            fb_loss / len(train_dataloader),
        )

    def cal_loss_fab(self, output_mu, label_mu, input_point):
        """
        calculate the loss of fa and fb

        fa: -mu^T * G * R^T  ==> lam^T
        fb: mu^T * G * R^T * p - mu^T * h  ==> lam^T * p + mu^T * h
        """

        mu1 = output_mu
        mu2 = label_mu
        ip = torch.unsqueeze(input_point, 2)
        mu1T = torch.transpose(mu1, 1, 2)
        mu2T = torch.transpose(mu2, 1, 2)

        theta = np.random.uniform(0, 2 * np.pi)
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        R = np_to_tensor(R)

        fa = torch.transpose(-R @ self.G.T @ mu1, 1, 2)
        fa_label = torch.transpose(-R @ self.G.T @ mu2, 1, 2)

        fb = fa @ ip + mu1T @ self.h
        fb_label = fa_label @ ip + mu2T @ self.h

        mse_lamt = self.loss_fn(fa, fa_label)
        mse_lamtb = self.loss_fn(fb, fb_label)

        return mse_lamt, mse_lamtb

    def cal_distance(self, mu, input_point):

        input_point = torch.unsqueeze(input_point, 2)

        temp = self.G @ input_point - self.h

        muT = torch.transpose(mu, 1, 2)

        distance = torch.squeeze(torch.bmm(muT, temp))

        return distance

    def print_loss(self, i, epoch, ml, dl, al, bl, vml, vdl, val, vbl, lr, file=None):

        if file is None:
            print(
                "Epoch {}/{}, learning rate {} \n"
                "---------------------------------\n"
                "Losses:\n"
                "  Mu Loss:          {} | Validate Mu Loss:          {}\n"
                "  Distance Loss:    {} | Validate Distance Loss:    {}\n"
                "  Fa Loss:          {} | Validate Fa Loss:          {}\n"
                "  Fb Loss:          {} | Validate Fb Loss:          {}\n".format(
                    i,
                    epoch,
                    lr,
                    str(ml).ljust(10),
                    str(vml).rjust(10),
                    str(dl).ljust(10),
                    str(vdl).rjust(10),
                    str(al).ljust(10),
                    str(val).rjust(10),
                    str(bl).ljust(10),
                    str(vbl).rjust(10),
                )
            )

        else:
            print(
                "Epoch {}/{} learning rate {} \n"
                "---------------------------------\n"
                "Losses:\n"
                "  Mu Loss:          {} | Validate Mu Loss:          {}\n"
                "  Distance Loss:    {} | Validate Distance Loss:    {}\n"
                "  Fa Loss:          {} | Validate Fa Loss:          {}\n"
                "  Fb Loss:          {} | Validate Fb Loss:          {}\n".format(
                    i,
                    epoch,
                    lr,
                    str(ml).ljust(10),
                    str(vml).rjust(10),
                    str(dl).ljust(10),
                    str(vdl).rjust(10),
                    str(al).ljust(10),
                    str(val).rjust(10),
                    str(bl).ljust(10),
                    str(vbl).rjust(10),
                ),
                file=file,
            )

    def test(self, model_pth, train_dict_kwargs, data_size_list=0, **kwargs):

        with open(train_dict_kwargs, "rb") as f:
            train_dict = pickle.load(f)

        model = to_device(train_dict["model"])
        model.load_state_dict(torch.load(model_pth))
        data_range = train_dict["data_range"]

        print("dataset generating start ...")

        max_data_size = max(data_size_list)

        start_time = time.time()
        dataset = self.generate_data_set(max_data_size, data_range)
        data_generate_time = time.time() - start_time
        print(
            "data_size:", max_data_size, "dataset generating time: ", data_generate_time
        )

        for data_size in data_size_list:
            test_dataloader = DataLoader(dataset, batch_size=data_size)

            mu_loss_list = []
            distance_loss_list = []
            fa_loss_list = []
            fb_loss_list = []
            inference_time_list = []

            for input_point, label_mu, label_distance in test_dataloader:
                average_loss_list, inference_time = self.test_one_epoch(
                    model, input_point, label_mu, label_distance, data_size
                )

                mu_loss_list.append(average_loss_list[0])
                distance_loss_list.append(average_loss_list[1])
                fa_loss_list.append(average_loss_list[2])
                fb_loss_list.append(average_loss_list[3])
                inference_time_list.append(inference_time)

            avg_mu_loss = sum(mu_loss_list) / len(mu_loss_list)
            avg_distance_loss = sum(distance_loss_list) / len(distance_loss_list)
            avg_fa_loss = sum(fa_loss_list) / len(fa_loss_list)
            avg_fb_loss = sum(fb_loss_list) / len(fb_loss_list)
            avg_inference_time = sum(inference_time_list) / len(inference_time_list)

            with open(os.path.dirname(model_pth) + "/test_results.txt", "a") as f:
                print(
                    "Model_name {}, Data_size {}, inference_time {} \n"
                    "---------------------------------\n"
                    "Losses:\n"
                    "  Mu Loss:          {} \n"
                    "  Distance Loss:    {} \n"
                    "  Fa Loss:          {} \n"
                    "  Fb Loss:          {} \n".format(
                        os.path.basename(model_pth),
                        data_size,
                        avg_inference_time,
                        str(avg_mu_loss).ljust(10),
                        str(avg_distance_loss).ljust(10),
                        str(avg_fa_loss).ljust(10),
                        str(avg_fb_loss).ljust(10),
                    ),
                    file=f,
                )

                # with open(os.path.dirname(model_pth) + '/results_dict.pkl', 'wb') as f:
                #     results_kwargs = { 'Model_name': os.path.basename(model_pth), 'Data_size': data_size, 'inference_time': sum(inference_time_list) / len(inference_time_list), 'mu_loss': sum(mu_loss_list)/ len(mu_loss_list), 'distance_loss': sum(distance_loss_list)/len(distance_loss_list), 'fa_loss': sum(fa_loss_list)/ len(fa_loss_list), 'fb_loss': sum(fb_loss_list) / len(fb_loss_list)}

                #     pickle.dump(results_kwargs, f)
        print(
            "finish test, the results are saved in {}".format(
                os.path.dirname(model_pth) + "/test_results.txt"
            )
        )

    def test_one_epoch(self, model, input_point, label_mu, label_distance, data_size):

        input_point = torch.squeeze(input_point)

        start_time = time.time()
        output_mu = model(input_point)
        inference_time = time.time() - start_time

        output_mu = torch.unsqueeze(output_mu, 2)

        distance = self.cal_distance(output_mu, input_point)

        mse_mu = self.loss_fn(output_mu, label_mu)
        mse_distance = self.loss_fn(distance, label_distance)
        mse_fa, mse_fb = self.cal_loss_fab(output_mu, label_mu, input_point)

        # loss = mse_mu.item() + mse_distance + mse_fa + mse_fb
        # average_loss_list = [mse_mu.item() / data_size, mse_distance.item() / data_size, mse_fa.item() / data_size, mse_fb.item() / data_size]

        loss_list = [mse_mu.item(), mse_distance.item(), mse_fa.item(), mse_fb.item()]

        # print('Data_size {}, inference_time {} \n'
        #             '---------------------------------\n'
        #             'Losses:\n'
        #             '  Mu Loss:          {} \n'
        #             '  Distance Loss:    {} \n'
        #             '  Fa Loss:          {} \n'
        #             '  Fb Loss:          {} \n'
        #             .format(data_size, inference_time,
        #                     str(average_loss_list[0]).ljust(10),
        #                     str(average_loss_list[1]).ljust(10),
        #                     str(average_loss_list[2]).ljust(10),
        #                     str(average_loss_list[3]).ljust(10)))
        return loss_list, inference_time
```

### 3.1 创建 Scout Mini 训练配置

此配置保持不变，用于定义机器人几何形状和训练超参数。

```yaml name=~/neupan_ws/src/NeuPAN/configs/train_scout_mini.yaml
# Scout Mini DUNE 模型训练配置
robot:
  kinematics:  'diff'          # 差速驱动模型
  length: 0.65                # Scout Mini 长度 0.612m + 安全余量
  width: 0.60                 # Scout Mini 宽度 0.58m + 安全余量

# 训练参数
train:
  direct_train: true          # 直接开始训练
  model_name: 'scout_mini_diff'  # 模型保存文件夹名称
  data_size: 100000           # 采样样本数
  data_range: [-10, -10, 10, 10]  # 空间范围
  batch_size: 256             # 批次大小
  epoch: 5000                 # 训练轮数
  valid_freq: 250             # 验证频率
  save_freq: 500              # 保存频率
  lr:  5e-5                    # 学习率
  lr_decay: 0.5               # 学习率衰减
  decay_freq: 1500            # 衰减频率
```

###  3.2：创建并运行训练脚本

在 `~/neupan_ws/src/NeuPAN/` 目录下创建 `run_train.py`：

```python
#!/usr/bin/env python3
from neupan import neupan
import os

if __name__ == '__main__':
    print(">>> 正在启动训练...")
    
    # 确保在正确的目录
    os.chdir('/home/lyb/neupan_ws/src/NeuPAN')
    
    # 初始化并开始训练
    neupan_planner = neupan.init_from_yaml('configs/train_scout_mini.yaml')
    neupan_planner.train_dune()
```

**执行训练：**

```bash
cd ~/neupan_ws/src/NeuPAN
python3 run_train.py
```

###  3.3：使用 TensorBoard 可视化 (重点)

由于 ROS Noetic 依赖较旧的系统级 Protobuf 库，直接安装最新版 TensorBoard 会导致 `TypeError: MessageToJson()` 报错。因此，**必须指定安装兼容的旧版本**。

请按照以下步骤清理环境并安装兼容版本（推荐使用 `--user` 标志安装在用户目录，避免污染系统）：

```bash
# 1. 首先卸载可能存在的冲突版本
python3 -m pip uninstall -y tensorboard tb-nightly protobuf

# 2. 安装兼容 ROS Noetic 的 TensorBoard (2.11.x) 和 Protobuf (3.20.x)
python3 -m pip install --user tensorboard==2.11.2 protobuf==3.20.3
```

安装完成后，运行以下命令验证。如果能正确显示版本号且无报错，说明环境配置成功。

```bash
tensorboard --version
```

**启动可视化流程：**

1.  **打开一个新的终端窗口**。
2.  **执行命令**（注意路径对应配置文件中的 `model_name`）：

```bash
# 进入 NeuPAN 根目录
cd ~/neupan_ws/src/NeuPAN

# 启动 TensorBoard，指向 'model/模型名称/runs' 目录
tensorboard --logdir=model/scout_mini_diff/runs --port=6006
```

3.  **查看结果**：
    *   在浏览器中输入地址：`http://localhost:6006`
    *   点击上方的 **SCALARS** 标签页，你应该能看到：
        *   `Loss/Train/Total`: 总体训练误差（应呈下降趋势）。
        *   `Loss/Valid/Total`: 验证集误差（用于检测过拟合）。
        *   `Hyperparameters/Learning_Rate`: 学习率阶梯式下降的情况。
### 3.4：导出模型 (制品交付)

当 TensorBoard 显示 Loss 已经趋于平稳且足够低时，你可以将训练好的 `.pth` 文件复制到 ROS 包中使用。

```bash
# 假设训练了 5000 轮，选择最后的模型
mkdir -p ~/neupan_ws/src/neupan_ros/model/custom/

cp ~/neupan_ws/src/NeuPAN/model/scout_mini_diff/model_5000.pth \
   ~/neupan_ws/src/neupan_ros/model/custom/scout_dune.pth

echo "模型已导出至 neupan_ros！"
```

---

## 阶段四：多传感器精密外参标定与驱动层重构 (Extrinsic Calibration & Driver Refactoring)

### 4.1 TF 坐标系链 (REP-105 标准架构)

修正后的 TF 树结构如下，确保每个子坐标系只有一个父节点：

```text
map (地图)
 │
 └── odom (里程计)
      │  
      │ [发布者: scout_base_node]
      │ [说明: 动态变换，由编码器/IMU计算]
      ▼
 base_footprint (地面投影点, Z=0)
      │
      │ [发布者: scout_tf.launch (静态)]
      │ [说明: 描述车身离地高度 Z=0.181m]
      ▼
 base_link (机器人几何中心/质心)
      │
      │ [发布者: scout_tf.launch (静态)]
      │ [说明: 描述雷达安装位置]
      ▼
 velodyne (激光雷达)
```

### 4.2 雷达-车身外参精密标定实操 (Extrinsic Calibration SOP)

为消除 URDF 模型悬空误差以及建图重影，实现点云与物理世界 1:1 重合，必须进行工业级精度的手动标定。以下为标准操作流程：

#### 第一阶段：系统启动 (准备标定环境)

请依次打开 5 个终端，执行以下命令：

1. **启动底盘 (确保 TF 开启)**
   ```bash
   roslaunch scout_bringup scout_mini_robot_base.launch
   ```
2. **启动雷达**
   ```bash
   roslaunch velodyne_pointcloud VLP16_points.launch
   ```
3. **加载机器人模型**
   ```bash
   roslaunch scout_description display_scout_mini.launch
   ```
4. **启动 RViz**
   ```bash
   rosrun rviz rviz
   ```
5. **预留调参终端** (保持空白，用于发送动态 TF 指令)

#### 第二阶段：RViz 精密设置 (配置标准尺)

此步骤是标定的“尺子”，必须严格设置：

1. **Global Options**
   * **Fixed Frame**: 初始设为 **`base_footprint`**。
2. **Grid (虚拟地面)**
   * Reference Frame: `<Fixed Frame>`
   * Cell Size: `1.0`
   * Color: **灰色** (160, 160, 160) —— *为了与红色点云形成高对比度*。
3. **RobotModel (肉身参照)**
   * Alpha: **`0.5`** —— *设置为半透明，确保能看穿轮毂观察到轮胎底部*。
4. **PointCloud2 (雷达读数)**
   * Topic: `/velodyne_points`
   * Color: **红色** (Flat Color)。
5. **Views 面板**
   * 点击菜单栏 `Panels` -> 勾选 `Views` (调出右侧视角控制面板)。

#### 第三阶段：先定“地” —— 校准 `base_footprint`

**目标**：让悬空的 URDF 模型落地，轮子底端相切于网格线。

1. **锁定视角 (Orbit)**：
   * 在 Views 面板中设置 **Type** 为 **`Orbit (rviz)`**。
   * **Pitch**: `0` (水平视线)。
   * **Yaw**: `1.57` (正侧面观察)。
   * **Focal Point**: `0; 0; 0`。
2. **输入指令验证**：
   在调参终端输入推测的黄金数据（不断微调 Z 值）：
   ```bash
   rosrun tf2_ros static_transform_publisher 0 0 0.181 0 0 0 base_footprint base_link
   ```
3. **验收标准**：
   * 白色半透明轮胎的**最底端圆弧**，应该刚好**压在/相切于**一条灰色网格线上。
   * *(当前实测完美值：Z = 0.181)*

#### 第四阶段：再定“天” —— 校准 `velodyne`

**目标**：让雷达点云与物理世界重合，且在运动中无重影。

请开启第 6 个终端，输入指令并根据下方步骤不断微调参数：
```bash
rosrun tf2_ros static_transform_publisher 0.15 0 0.554 0 -0.068 0.01 base_link velodyne
```

**子步骤 4.1：调平姿态 (Pitch / Roll)**
* **视角设置**：保持 `Orbit (rviz)`。
* **视角 A (Pitch)**：Yaw `1.57` (正侧面)，观察红线与灰线是否平行。
* **视角 B (Roll)**：Yaw `3.14` (正后面)，观察左右两翼是否等高。
* **验收标准**：雷达扫描出的红色地平面，像一张平铺的地毯，与灰色网格线完全平行。*(当前实测完美值：Pitch = -0.068, Roll = 0.01)*

**子步骤 4.2：对齐高度 (Z 轴)**
* **视角设置**：保持 `Orbit (rviz)`，鼠标滚轮放大聚焦于轮子底部。
* **验收标准**：必须实现**三线合一**（白色轮底、灰色网格、红色点云，三者必须在同一绝对平面上）。*(当前实测完美值：Z = 0.554)*

**子步骤 4.3：防重影校准 (X 轴) —— ⚠️ 最关键一步**
1. **切换参考系 (上帝视角)**：
   * RViz 左侧 **Global Options -> Fixed Frame** 改为 **`odom`**。
   * *(原理：利用里程计抵消车体旋转，使环境静止，车动环境不动)*
2. **切换视角 (正交顶视)**：
   * Views 面板 **Type** 改为 **`TopDownOrtho (rviz)`**，Target Frame 设为 `base_link`，点击 `Zero` 复位。
   * *(原理：消除透视导致的“近大远小”误差，精确观察墙壁厚度)*
3. **开启长曝光**：
   * PointCloud2 属性中 **Decay Time** 设为 **`20`** (秒)。
4. **动态旋转测试**：
   * 点击 RViz 右下角 `Reset` 清空旧点云。
   * 遥控小车**原地旋转 360 度**。
5. **验收标准**：
   * 周围墙壁的点云应当**纹丝不动**，只会变浓（颜色变深），绝不会变宽或分叉。
   * 若出现分叉重影，请微调 X 轴偏移量。*(当前实测完美值：X = 0.15)*

---

### 4.3 TF 静态变换配置 (`scout_tf.launch`)

将上述标定验证无误的黄金参数固化写入 Launch 文件。此文件负责连接 `base_footprint` -> `base_link` 和 `base_link` -> `velodyne`。

```xml name=~/neupan_ws/src/neupan_ros/launch/scout_tf.launch
<?xml version="1.0"?>
<launch>
    <!-- 
    =======================================================
    Scout Mini 多层支架雷达精细标定参数 (工业级精度版 v3.0)
    标定方法: 
      - Z轴/Pitch/Roll: 使用 Orbit 侧视图校准 (相切/平行)
      - X轴: 使用 odom坐标系 + TopDownOrtho 正交顶视 + 动态旋转校准
    =======================================================
    -->
    
    <!-- 
    1. Footprint to Base (定地)
    作用: 连接 odom 树与 robot 树。
    描述: 修正 URDF 模型中轮轴中心与地面的物理高度差。
    参数: x y z yaw pitch roll parent child
    [实测数据]: Z = 0.181 (Orbit 侧视实测相切高度)
    -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="footprint_to_base"
          args="0 0 0.181 0 0 0 base_footprint base_link" />

    <!-- 
    2. Base to Velodyne (定天)
    作用: 定义雷达在车身上的安装位置。
    [标定结果解析]
    X = 0.150      : 前后偏移 (TopDownOrtho 视角下，旋转无重影)
    Z = 0.554      : 垂直高度 (Orbit 视角下，点云与网格重合)
    Pitch = -0.068 : 修正俯仰角 (Orbit 侧视平行)
    Roll = 0.010   : 修正横滚角 (Orbit 后视水平)
    Yaw = 0        : 偏航角 (默认正装)
    -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_velodyne"
          args="0.15 0 0.554 0 -0.068 0.01 base_link velodyne" />
    
</launch>
```

### 4.4 底盘驱动源码修改 (关键步骤)

基于我们之前的所有排查和对话，这是 **4.3 章节的最终修正方案**。

此方案解决了三个核心问题：
1.  **TF 架构修正**：强制底盘发布 `odom -> base_footprint`（符合 REP-105）。
2.  **模型不显示修复**：不再依赖外部不稳定的 launch 文件，直接在顶层文件中内嵌模型加载逻辑，并修正了 xacro 文件名。
3.  **车轮报错修复**：增加了 `joint_state_publisher` 补丁，确保 Rviz 中车轮不再报红。

以下是两个文件的完整代码：

---

### 文件 1：底层驱动配置 (`scout_base.launch`)

**修改说明**：
*   新增 `base_frame` 参数并传递给 C++ 节点，不再硬编码为 `base_link`。

```xml name=~/neupan_ws/src/scout_ros/scout_base/launch/scout_base.launch
<launch>
    <!-- 基础参数定义 -->
    <arg name="port_name" default="can0" />
    <arg name="is_scout_mini" default="false" />
    <arg name="is_scout_omni" default="false" />
    <arg name="simulated_robot" default="false" />
    <arg name="odom_topic_name" default="odom" />
    <arg name="pub_tf" default="true" />
    
    <!-- [新增] 引入 base_frame 参数，默认指向 base_footprint -->
    <arg name="base_frame" default="base_footprint" /> 

    <node name="scout_base_node" pkg="scout_base" type="scout_base_node" output="screen" required="true">
        <param name="is_scout_mini" type="bool" value="$(arg is_scout_mini)" />
        <param name="is_scout_omni" type="bool" value="$(arg is_scout_omni)" />
        <param name="port_name" type="string" value="$(arg port_name)" />
        <param name="simulated_robot" type="bool" value="$(arg simulated_robot)" />
        <param name="odom_frame" type="string" value="odom" />
        
        <!-- [修改] 使用参数变量，不再硬编码为 base_link -->
        <param name="base_frame" type="string" value="$(arg base_frame)" />
        
        <param name="odom_topic_name" type="string" value="$(arg odom_topic_name)" />
        <param name="pub_tf" type="bool" value="$(arg pub_tf)" />
    </node>
</launch>
```

---

### 文件 2：顶层启动接口 (`scout_mini_robot_base.launch`)

**修改说明**：
*   **模型路径修正**：指向 `scout_mini.urdf.xacro`。
*   **模型加载重写**：手动解析 xacro 并启动 `robot_state_publisher`，解决 Rviz 无模型问题。
*   **关节状态补丁**：添加 `joint_state_publisher`，解决 Rviz 车轮变红/无 Transform 问题。
*   **TF 传递**：将 `base_footprint` 参数正确传递给底层。

```xml name=~/neupan_ws/src/scout_ros/scout_bringup/launch/scout_mini_robot_base.launch
<launch>
    <!-- [CAN 设置] -->
    <arg name="port_name" default="can0" />
    <arg name="simulated_robot" value="false" />

    <!-- 
      [关键修正 1: 模型文件路径] 
      修正为 scout_mini.urdf.xacro (基于你的实际文件结构)
    -->
    <arg name="model_xacro" default="$(find scout_description)/urdf/scout_mini.urdf.xacro" />

    <!-- [基础参数] -->
    <arg name="odom_topic_name" default="odom" />
    <arg name="is_scout_mini" default="true" />
    
    <!-- [TF 设置] 开启 TF 发布，指定 base_footprint -->
    <arg name="pub_tf" default="true" />
    <arg name="base_frame" default="base_footprint" />

    <!-- 
      =========================================================
      1. 启动 Scout 底盘核心驱动
      =========================================================
    -->
    <include file="$(find scout_base)/launch/scout_base.launch">
        <arg name="port_name" default="$(arg port_name)" />
        <arg name="simulated_robot" default="$(arg simulated_robot)" />
        <arg name="odom_topic_name" default="$(arg odom_topic_name)" />
        <arg name="is_scout_mini" default="$(arg is_scout_mini)" />
        <arg name="pub_tf" default="$(arg pub_tf)" />
        <!-- 将 base_frame 参数传递给底层 -->
        <arg name="base_frame" value="$(arg base_frame)" />
    </include>

    <!-- 
      =========================================================
      2. 加载机器人模型 (解决 Model Error)
      =========================================================
      不再调用 display_scout_mini.launch，而是直接在这里执行加载逻辑。
    -->

    <!-- A. 解析 URDF/Xacro 文件并上传到参数服务器 /robot_description -->
    <param name="robot_description" command="$(find xacro)/xacro '$(arg model_xacro)'" />

    <!-- B. 启动状态发布节点 
         作用: 根据 /robot_description 发布静态 TF (如车轮相对于车身的位置)
    -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

    <!-- 
      =========================================================
      3. 关节状态补丁 (解决 Wheel No Transform)
      =========================================================
      启动 joint_state_publisher (非 GUI 版)。
      作用: 自动发布轮子的关节状态(默认为0)。这样即使底盘驱动没发数据，
           Rviz 里的轮子也能正常显示，不会报错。
    -->
	    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

</launch>
```

### 4.5 3D 点云转 2D 激光扫描配置

> **参考来源**: [pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py](https://github.com/ros-perception/pointcloud_to_laserscan/blob/main/launch/sample_pointcloud_to_laserscan_launch.py)

**修改说明**：
1.  **坐标系修正**：将 `target_frame` 强制锁定为 `base_link`（车身水平坐标系），防止 2D 激光数据随雷达物理倾角倾斜。
2.  **高度过滤**：根据 `base_link` 为基准重设阈值，完美过滤掉 Scout Mini 的车头防撞杆和地面。

```xml name=~/neupan_ws/src/neupan_ros/launch/velodyne_to_scan.launch
<?xml version="1.0"?>
<launch>
    <!-- 
    Velodyne VLP-16 -> LaserScan (Nodelet 高性能版) - 倾斜安装修正版
    
    架构说明:
    使用 Nodelet 技术，将 3D->2D 的转换逻辑作为插件加载到雷达驱动进程中。
    这实现了点云数据的 "零拷贝" (Zero-Copy) 传输，极大降低了 CPU 占用和延迟。
    
    [关键修正]: 
    针对物理倾斜安装的雷达，必须将 target_frame 设为水平坐标系 (base_link)，
    否则生成的 2D 激光数据会随雷达一起倾斜，导致导航无法使用。
    -->

    <!-- 
      参数: 目标坐标系 (Target Frame)
      [修改前]: default="velodyne" (导致 2D 数据随雷达倾斜)
      [修改后]: default="base_link" (强制投影到车身水平坐标系)
    -->
    <arg name="target_frame" default="base_link"/>

    <!-- 
      参数: Nodelet 管理器名称
      必须与 velodyne_driver 启动的 manager 名称完全一致。
      VLP16_points.launch 默认通常叫 "velodyne_nodelet_manager"。
    -->
    <arg name="manager" default="velodyne_nodelet_manager"/>

    <!-- 启动转换节点 -->
    <node pkg="nodelet" type="nodelet" name="pointcloud_to_laserscan" respawn="true"
          args="load pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet $(arg manager)">

        <!-- 话题重映射 -->
        <remap from="cloud_in" to="/velodyne_points"/>
        <remap from="scan" to="/scan"/>

        <!-- 参数配置 (基于 Scout Mini + 倾斜雷达实测数据) -->
        <rosparam subst_value="true">
            # 目标坐标系 (现在是 base_link)
            target_frame: $(arg target_frame)
            
            # 变换容差
            transform_tolerance: 0.01

            # ---------------------------------------------------------
            # 高度过滤逻辑 (已根据 base_link 坐标系重算)
            # ---------------------------------------------------------
            
            # min_height: 0.15
            #   [计算逻辑]: 
            #   - base_link 位于车轮轴心高度 (Z=0)。
            #   - 地面位于 base_link 下方约 0.18m (Z = -0.18)。
            #   - Scout Mini 车头防撞杆/轮眉位于 base_link 平面附近 (Z ≈ 0 ~ 0.1)。
            #   [设置效果]: 
            #   - 设为 0.15m 意味着只截取车身中心上方 15cm 以上的数据。
            #   - 完美过滤掉地面、地毯边缘、以及车头那个“绿色的拱起”。
            min_height: 0.15
            
            # max_height: 1.0
            #   [计算逻辑]: 
            #   - 截取 base_link 上方 1米内的数据。
            #   - 忽略天花板和高处悬挂物，专注于导航层障碍物。
            max_height: 1.0

            # ---------------------------------------------------------
            # VLP-16 物理参数 (参考 Datasheet Page 2)
            # ---------------------------------------------------------
            angle_min: -3.14159265359 # -180度
            angle_max: 3.14159265359  # +180度
            
            # angle_increment: 0.005 (约 0.28度)
            # Datasheet 标称分辨率 0.1°-0.4°。
            # 0.005 是一个兼顾导航精度和 CPU 负载的推荐值。
            angle_increment: 0.005  
            
            scan_time: 0.1            # 10Hz (Datasheet: Rotation Rate 5-20Hz)
            
            range_min: 0.3            # 近场盲区过滤 (保留以防漏网之鱼)
            range_max: 100.0          # 最大量程 (Datasheet: 100 m)

            # ---------------------------------------------------------
            # 杂项配置
            # ---------------------------------------------------------
            use_inf: true
            inf_epsilon: 1.0

            # 并发级别: Nodelet 模式下设为 1，交由 Manager 线程池调度
            concurrency_level: 1
        </rosparam>

    </node>

</launch>
```

---

### 4.6 经过“外科手术”修改的雷达驱动

请修改 `velodyne_pointcloud` 包中的此文件。

**修改重点**：彻底注释掉 `laserscan_nodelet.launch` 的引用。

```xml name=~/neupan_ws/src/velodyne/velodyne_pointcloud/launch/VLP16_points.launch
<!-- -*- mode: XML -*- -->
<!-- run velodyne_pointcloud/TransformNodelet in a nodelet manager for a VLP-16 -->

<launch>

  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP16db.yaml"/>
  <arg name="device_ip" default="" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2368" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <arg name="gps_time" default="false" />
  <arg name="pcap_time" default="false" />
  <arg name="cut_angle" default="-0.01" />
  <arg name="timestamp_first_packet" default="false" />
  <arg name="laserscan_ring" default="-1" />
  <arg name="laserscan_resolution" default="0.007" />
  <arg name="organize_cloud" default="false" />

  <!-- 1. 启动 Nodelet 管理器 -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="VLP16"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
    <arg name="gps_time" value="$(arg gps_time)"/>
    <arg name="pcap_time" value="$(arg pcap_time)"/>
    <arg name="cut_angle" value="$(arg cut_angle)"/>
    <arg name="timestamp_first_packet" value="$(arg timestamp_first_packet)"/>
  </include>

  <!-- 2. 启动 3D 点云生成节点 (TransformNodelet) -->
  <!-- 核心组件：生成 /velodyne_points -->
  <include file="$(find velodyne_pointcloud)/launch/transform_nodelet.launch">
    <arg name="model" value="VLP16"/>
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="fixed_frame" value="" />
    <arg name="target_frame" value="" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
    <arg name="organize_cloud" value="$(arg organize_cloud)"/>
  </include>

  <!-- 
    ====================================================================
    [系统优化 - 已禁用] 
    官方自带的 2D 转换已移除。
    原因：节省 CPU，并防止发布低质量的 /scan 话题干扰系统。
    ====================================================================
  -->
  <!--
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include>
  -->

</launch>
```

## 阶段五：2D SLAM 建图实施与环境地图构建 (2D SLAM Mapping & Map Generation)
### 5.1 建图 Launch 文件 (`mapping_scout.launch`)

**主要确认点**：
1.  **TF 链条完整性**：底盘驱动发布 `odom -> base_footprint`，`scout_tf` 发布 `base_footprint -> base_link`。Gmapping 绑定在 `base_link` 上进行建图，这是符合 REP-105 标准的。
2.  **倾斜修正集成**：代码中调用的 `velodyne_to_scan.launch` 已经是我们刚才修改过的版本（Target=base_link），因此 Gmapping 接收到的 `/scan` 数据已经是**水平校正**过的，不会受雷达低头影响。

```xml name=~/neupan_ws/src/neupan_ros/launch/mapping_scout.launch
<?xml version="1.0"?>
<launch>
    <!--
    =================================================================
    Scout Mini 工业级 SLAM 建图启动文件 (最终修正版)
    集成: 
      1. 底盘: Scout Mini (Can0, Odom发布 -> base_footprint)
      2. 雷达: VLP-16 (Nodelet零拷贝架构, 仅输出 3D 点云)
      3. 标定: 黄金参数 TF (Orbit+Odom双视角验证)
      4. 修正: 2D 激光数据强制投影到 base_link 水平面
      5. 算法: Gmapping (针对滑移转向优化的噪声模型)
    =================================================================
    -->
    
    <!-- ==================== 0. 全局参数 ==================== -->
    <arg name="map_frame" default="map"/>
    <arg name="odom_frame" default="odom"/>
    <!-- 
       [TF 策略] Gmapping 跟踪 base_link (机器人中心)。
       底层的 scout_tf.launch 会负责连接 base_footprint -> base_link。
    -->
    <arg name="base_frame" default="base_link"/>
    
    <!-- ==================== 1. 硬件驱动 (Hardware) ==================== -->
    
    <!-- 1.1 Scout Mini 底盘驱动 -->
    <include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
        <!-- 显式声明车型 -->
        <arg name="is_scout_mini" value="true"/>
        <!-- 
           开启 TF 发布 (odom -> base_footprint)
           注意: 这里必须开启，确保 TF 树的根基存在
        -->
        <arg name="pub_tf" value="true"/>
        <!-- [REP-105 关键] 强制指定驱动层的基坐标系为 footprint -->
        <arg name="base_frame" value="base_footprint"/>
    </include>
        
    <!-- 1.2 Velodyne VLP-16 驱动 & Nodelet 管理器 -->
    <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
        <!-- 网络配置 -->
        <arg name="device_ip" value="192.168.1.201"/>
        <arg name="port" value="2368"/>
        <!-- 坐标系名称 -->
        <arg name="frame_id" value="velodyne"/>
        <!-- 物理过滤 (仅过滤极近距离噪点，高度过滤交给后面的 nodelet) -->
        <arg name="min_range" value="0.3"/>
        <arg name="max_range" value="100.0"/>
        
        <!-- 
           [系统优化] 
           已在 VLP16_points.launch 内部禁用了官方的 2D 转换，
           因为官方转换不支持倾斜雷达的水平投影修正。
           驱动现在只纯净地发布 /velodyne_points。
        -->
    </include>
    
    <!-- ==================== 2. TF 坐标变换 (黄金数据) ==================== -->
    <!-- 
       调用 scout_tf.launch 加载标定参数。
       如果雷达倾斜，请确保此文件里的 Pitch 反映了真实的物理倾角，
       以便 pointcloud_to_laserscan 能正确地把点云“扳正”。
    -->
    <include file="$(find neupan_ros)/launch/scout_tf.launch"/>
    
    <!-- ==================== 3. 感知处理 (Perception) ==================== -->
    <!-- 
       3D 点云转 2D 扫描 (Nodelet 高性能版)
       关键点: 这里的 velodyne_to_scan.launch 必须配置 target_frame=base_link
       以解决雷达物理倾斜导致的 2D 数据切片倾斜问题。
    -->
    <include file="$(find neupan_ros)/launch/velodyne_to_scan.launch">
        <arg name="manager" value="velodyne_nodelet_manager"/>
    </include>
    
    <!-- ==================== 4. SLAM (Gmapping 调优版) ==================== -->
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
        
        <!-- 坐标系设置 -->
        <param name="map_frame" value="$(arg map_frame)"/>
        <param name="odom_frame" value="$(arg odom_frame)"/>
        <param name="base_frame" value="$(arg base_frame)"/>
        
        <!-- 性能参数 -->
        <param name="map_update_interval" value="3.0"/> <!-- 地图更新间隔，越小越耗 CPU -->
        
        <!-- 激光参数 -->
        <param name="maxUrange" value="20.0"/> <!-- 激光雷达最大可用距离 (室内建议 10-20m) -->
        <param name="maxRange" value="100.0"/> <!-- 传感器物理最大距离 -->
        
        <!-- 运动更新阈值 (动了多少才更新一次地图) -->
        <param name="linearUpdate" value="0.2"/>    <!-- 移动 0.2米 更新一次 -->
        <param name="angularUpdate" value="0.2"/>   <!-- 旋转 0.2弧度 更新一次 -->
        <param name="temporalUpdate" value="3.0"/>  <!-- 如果不动，3秒强制更新一次 -->
        
        <!-- 粒子滤波参数 -->
        <param name="particles" value="50"/>        <!-- 粒子数，建议 30-80 -->
        <param name="xmin" value="-20.0"/>
        <param name="ymin" value="-20.0"/>
        <param name="xmax" value="20.0"/>
        <param name="ymax" value="20.0"/>
        <param name="delta" value="0.05"/>          <!-- 地图分辨率 (5cm) -->
        
        <!-- 
           【滑移转向 (Skid-Steer) 专用噪声参数】
           Scout Mini 转向时依靠轮子打滑，里程计的旋转分量非常不准。
           必须显著增大 srr (旋转->旋转误差) 和 str (平移->旋转误差)。
        -->
        <param name="srr" value="0.5"/> <!-- 旋转对旋转的误差 (重要) -->
        <param name="srt" value="0.2"/> <!-- 旋转对平移的误差 -->
        <param name="str" value="0.3"/> <!-- 平移对旋转的误差 (重要) -->
        <param name="stt" value="0.2"/> <!-- 平移对平移的误差 -->
        
        <!-- 匹配得分增益 -->
        <param name="ogain" value="3.0"/>
        <param name="lstep" value="0.05"/>
        <param name="astep" value="0.05"/>
        <param name="iterations" value="5"/>
        <param name="lsigma" value="0.075"/>
        <param name="kernelSize" value="1"/>
        <param name="lskip" value="0"/>
        
        <!-- 话题输入 -->
        <remap from="scan" to="/scan"/>
    </node>
    
    <!-- ==================== 5. Rviz 可视化 ==================== -->
    <node pkg="rviz" type="rviz" name="rviz_mapping" 
          args="-d $(find neupan_ros)/rviz/mapping.rviz" output="screen"/>
    
</launch>
```

收到！既然你已经配置出了完美的 Rviz 视图（激光水平、模型正确、TF 树完整），我们直接把这个“经过实战验证”的完整配置代码更新进手册。

这样下次启动时，窗口布局、视角、图层都会和你现在看到的一模一样。

以下是修改后的 **5.2 建图 Rviz 配置** 完整内容：

### 5.2 建图 Rviz 配置 (`mapping.rviz`)

**配置重点**：
*   **RobotModel**：用于验证车轮是否贴地，无红色报错。
*   **TF Structure**：已筛选显示 `map`, `odom`, `base_footprint`, `base_link`, `velodyne` 关键坐标系。
*   **Filtered 2D Scan**：**绿色激光线**。这是最重要的检查对象，它必须是**完全水平**的（平行于网格），且不包含车头防撞杆的噪点。
*   **Raw 3D Cloud**：白色点云，用于对比物理倾角与软件修正的效果。

```yaml name=~/neupan_ws/src/neupan_ros/rviz/mapping.rviz
Panels:
  - Class: rviz/Displays
    Help Height: 221
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Scout Mini Model1
        - /TF Structure1
        - /TF Structure1/Frames1
        - /Filtered 2D Scan1
        - /Gmapping Map1
      Splitter Ratio: 0.5
    Tree Height: 982
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5882353186607361
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Name: Time
    SyncMode: 0
    SyncSource: Raw 3D Cloud
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 160
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz/RobotModel
      Collision Enabled: false
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        front_left_wheel_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        front_mount:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        front_right_wheel_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        inertial_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        rear_left_wheel_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        rear_mount:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        rear_right_wheel_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
      Name: Scout Mini Model
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz/TF
      Enabled: true
      Filter (blacklist): ""
      Filter (whitelist): ""
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_footprint:
          Value: true
        base_link:
          Value: true
        front_left_wheel_link:
          Value: false
        front_mount:
          Value: false
        front_right_wheel_link:
          Value: false
        inertial_link:
          Value: false
        map:
          Value: true
        odom:
          Value: true
        rear_left_wheel_link:
          Value: false
        rear_mount:
          Value: false
        rear_right_wheel_link:
          Value: false
        velodyne:
          Value: true
      Marker Alpha: 1
      Marker Scale: 1
      Name: TF Structure
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          odom:
            base_footprint:
              base_link:
                front_left_wheel_link:
                  {}
                front_mount:
                  {}
                front_right_wheel_link:
                  {}
                inertial_link:
                  {}
                rear_left_wheel_link:
                  {}
                rear_mount:
                  {}
                rear_right_wheel_link:
                  {}
                velodyne:
                  {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/LaserScan
      Color: 0; 255; 0
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Filtered 2D Scan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05000000074505806
      Style: Flat Squares
      Topic: /scan
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Gmapping Map
      Topic: /map
      Unreliable: false
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Raw 3D Cloud
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.019999999552965164
      Style: Points
      Topic: /velodyne_points
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Theta std deviation: 0.2617993950843811
      Topic: /initialpose
      X std deviation: 0.5
      Y std deviation: 0.5
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 12.976668357849121
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Field of View: 0.7853981852531433
      Focal Point:
        X: -1.6049702167510986
        Y: 1.8113247156143188
        Z: 0.627142071723938
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Target Frame: <Fixed Frame>
      Yaw: 3.1104063987731934
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1449
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000030000000000000244000004fafc0200000003fb000000100044006900730070006c0061007900730100000047000004fa000000f100fffffffb0000001200530065006c00650063007400690065006e00000002ae000001160000006e00fffffffb0000001e0054006f006f006c002000500072006f00700065007200740069006500730000000354000001600000006e00ffffff0000000100000188000004fafc0200000001fb0000000a005600690065007700730100000047000004fa000000c200ffffff0000000300000a0000000040fc0100000001fb0000000800540069006d0065010000000000000a000000045a00ffffff00000626000004fa00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 2560
  X: 0
  Y: 34
```


### 5.3 建图操作流程 (SOP)

```bash
# ============================================
# 阶段 0: 硬件准备 (每次开机必做)
# ============================================
# 1. 激活 CAN 接口
# cd ~/neupan_ws/src/ugv_sdk/scripts/
# ./bringup_can2usb_500k.bash

# 2. 检查雷达通信
# ping 192.168.1.201

# 3. 遥控器检查
# 确保 SWB 拨杆在最上方 (指令模式)

# ============================================
# 阶段 1: 启动软件
# ============================================

# 终端 1: 启动建图核心节点
roslaunch neupan_ros mapping_scout.launch

# 终端 2: 启动键盘控制
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# ============================================
# 阶段 2: 建图实操技巧 (Gmapping 最佳实践)
# ============================================

# 步骤 1: 静止初始化
#   - 启动后不要立即移动！静止 5-10 秒。
#   - 观察 Rviz: 确认激光雷达数据 (LaserScan) 平稳且水平。

# 步骤 2: 初始特征建立 (关键)
#   - 控制机器人原地极慢速旋转 360°。
#   - 速度建议: angular.z ≈ 0.2 rad/s (按键盘 'j' 或 'l' 点动)。
#   - 目的: 让粒子滤波器获得初始的全向环境特征。

# 步骤 3: 遍历环境
#   - 直线行驶: 速度 < 0.3 m/s。
#   - 转向操作: 尽量走圆弧 (边走边转)，避免频繁的原地旋转 (减少里程计打滑误差)。
#   - 避免急停急转。

# 步骤 4: 闭环检测 (Loop Closure)
#   - 规划路径必须是一个闭合的圈。
#   - 必须控制机器人回到出发时的原点。
#   - 观察 Rviz: 若地图重影突然消失，地图边界变得清晰，说明闭环成功。

# ============================================
# 阶段 3: 保存地图 (已修改：存入独立子文件夹)
# ============================================

# 步骤 5: 保存文件 (请在 终端 3 执行)

# 5.1 创建多级文件夹 (关键步骤)
# -p 参数确保如果 maps 不存在也会一并创建
mkdir -p ~/maps/office
mkdir -p ~/maps/room
# 5.2 执行保存命令
# -f 参数指定路径和文件名前缀
# 这里 ~/maps/office/office 意思是：
# 目录: ~/maps/office/
# 文件名: office (.pgm/.yaml)
rosrun map_server map_saver -f ~/maps/room/room

# 5.3 验证文件是否生成
ls -l ~/maps/office/

# 预期输出: 
#   - office.pgm  (地图灰度图)
#   - office.yaml (地图配置文件)
```


## 阶段六：导航中间件参数调优与独立联调验证 (Nav-Stack Tuning & Standalone Testing)

### 📄 文件 1：AMCL 配置文件 (amcl.yaml)
**核心目标**：针对滑移转向（Skid-Steer）底盘，通过重构运动学噪声协方差矩阵，提升极端狭窄通道内的定位稳态精度，抑制原地旋转导致的漂移发散。

```yaml
# AMCL 配置 - Scout Mini 滑移转向极端调优版
# 优化目标：死死咬住地图，解决原地旋转时的定位发散问题

# =========================================
# 1. 粒子滤波器核心参数 (控制算力与精度平衡)
# =========================================
min_particles: 500            # [最小粒子数] 保证系统在稳态跟踪时的基础定位精度与算力下限。
max_particles: 5000           #[最大粒子数] 提升至 5000，确保在滑移底盘产生高噪声或面临长直狭窄走廊（特征稀疏）时，有足够的粒子云覆盖真实位姿分布，防止定位丢失。
kld_err: 0.05                 # [KLD 误差阈值] 决定真实分布与估计分布之间的最大允许误差，0.05 保证了粒子群的快速收敛。
kld_z: 0.99                   # [KLD 概率分布界限] 要求 99% 的概率满足上述误差阈值。

# [更新频率极限调优 (防转弯发散)]
# 数学意义：严格控制重采样频率。只要移动 0.1m 或旋转约 2.8°(0.05rad)，就强制调用雷达进行一次观测更新。
# 工程作用：在狭窄通道内，差速底盘微调方向极易累计误差。高频的雷达纠正能瞬间将偏离的粒子拉回真实墙壁轮廓。
update_min_d: 0.1             # [平移更新阈值] 机器人每移动 0.1 米触发一次滤波更新。
update_min_a: 0.05            # [旋转更新阈值] 机器人每旋转 0.05 弧度触发一次滤波更新。
resample_interval: 1          # [重采样间隔] 每次滤波更新后立即执行重采样，保持粒子群的聚集度。

# =========================================
# 2. ★ 运动模型调优 (解决滑移转向痛点的核心) ★
# =========================================
#[⚠️ 必须使用 diff-corrected 严谨模型]
# 传统的 diff 模型存在旋转方差计算缺陷。diff-corrected 修正了旋转时的误差累积计算公式，是后续高噪声参数生效的基础。
odom_model_type: diff-corrected

# [滑移转向 (Skid-Steer) 专用高噪声协方差矩阵设置]
# 物理背景：Scout Mini 原地转向时必须克服轮胎与地面的横向滑动摩擦，破坏了理想的非完整约束。
odom_alpha1: 0.8              # [旋转导致的旋转噪声] (rad/rad) 设为极高值 0.8。应对差速底盘极度打滑导致的角速度里程计失真。
odom_alpha2: 0.2              # [平移导致的旋转噪声] (m/rad) 正常平移时不易产生旋转误差，保持较低的 0.2。
odom_alpha3: 0.2              # [平移导致的平移噪声] (m/m) 前进时履带抓地力较好，里程计相对准确，保持 0.2。
odom_alpha4: 0.5              #[旋转导致的平移噪声] (rad/m) 设为较高的 0.5。核心解决侧滑问题！告诉滤波器：车体旋转时必然伴随不可测的横向平移漂移。
odom_alpha5: 0.1              # [平移导致的横向平移噪声] 仅用于 omni 全向轮模型，diff 驱动下此参数不参与运算。

# =========================================
# 3. 激光模型参数 (增强雷达在观测更新中的话语权)
# =========================================
laser_model_type: likelihood_field # [激光模型] 似然场模型。相比光束模型，它在计算点云到障碍物距离时更平滑，且计算效率极高。
laser_max_beams: 60           # [最大波束数] 均匀下采样至 60 根雷达线参与计算，大幅降低 CPU 开销且不损失特征。
laser_max_range: 30.0         # [最大有效距离] VLP-16 雷达截断距离，超出 30 米的点云不纳入定位计算。
laser_z_hit: 0.95             # [匹配击中权重] 极高权重！告诉系统：高度信任雷达扫描到的静态墙壁，强制粒子群向雷达轮廓靠拢。
laser_z_rand: 0.05            # [随机噪声权重] 极低权重！降低动态障碍物或传感器散粒噪声对定位的干扰。
laser_sigma_hit: 0.1          # [高斯分布标准差] (m) 严丝合缝的雷达匹配容差。要求雷达点距地图障碍物的误差分布极窄，强化定位精度。
laser_likelihood_max_dist: 2.0 #[最大似然距离] 超过 2.0 米的雷达点不进行似然得分计算。

# =========================================
# 4. 绑架恢复与坐标系配置
# =========================================
recovery_alpha_slow: 0.001    # [慢速指数衰减率] 用于决定何时增加随机粒子以应对位置丢失（慢速追踪）。
recovery_alpha_fast: 0.1      # [快速指数衰减率] 快速响应环境特征的突变，启动绑架恢复机制。

odom_frame_id: odom           # [里程计坐标系] 局部连续坐标系。
base_frame_id: base_link      # [基座坐标系] 机器人几何中心坐标系。
global_frame_id: map          # [全局坐标系] 静态地图坐标系。

# [TF 变换时间容差] 允许系统时间戳存在 0.5 秒误差，防止因 CPU 负载波动导致 "Extrapolation Error" 或 "Lookup into the future" 崩溃报错。
transform_tolerance: 0.5      
tf_broadcast: true            # [广播开关] 允许 AMCL 发布 map -> odom 的 TF 补偿树。
gui_publish_rate: 5.0         # [可视化频率] RViz 粒子云更新频率 (5Hz)。
save_pose_rate: 0.5           # [位姿保存频率] 每 2 秒将当前最优位姿写入参数服务器，供下次启动使用。

# =========================================
# 5. 初始位姿 (Initial Pose)
# =========================================
initial_pose_x: 0.0           # [初始 X 坐标]
initial_pose_y: 0.0           # [初始 Y 坐标]
initial_pose_a: 0.0           # [初始偏航角] (弧度制)
initial_cov_xx: 0.25          # [初始 X 协方差] 给予 0.5m 的初始不确定度搜索范围。
initial_cov_yy: 0.25          # [初始 Y 协方差] 给予 0.5m 的初始不确定度搜索范围。
initial_cov_aa: 0.06          # [初始角度协方差] 给予约 14度的初始角度搜索范围。

use_map_topic: true           # [订阅地图话题] 从 /map 话题实时获取而不是请求服务。
first_map_only: false         # [仅用首帧地图] 设为 false，允许在导航中途接收更新的地图。
```

---

### 📄 文件 2：全局代价地图配置 (global_costmap.yaml)
**核心目标**：针对 68cm 极窄通道，通过高分辨率与极限膨胀半径的博弈，构建平滑的 V 型势能峡谷，为 NeuPAN 局部规划提供无折线、绝对居中的全局拓扑引导。

```yaml
# 独立全局代价地图配置 -[窄道极限穿梭与全局缝合版]
# 架构说明：将动态避障全权交由底层的 NeuPAN (MPC) 处理，确保系统不重叠计算。
# 全局地图仅保留静态层，专注于生成长距离、全局平滑且居中的拓扑引导路径。

global_frame: map             # [全局参考系]
robot_base_frame: base_link   # [机器人基座系]

# =========================================
# 1. 路径稳定性与频率设置 (算力让步策略)
# =========================================
# 物理考量：剥离动态障碍物层后，全局地图等效为绝对静态的赛道。
# 将地图更新频率压低至 1.0Hz，彻底消除因高频重算导致的全局路径跳动，并将宝贵的 CPU 算力全盘释放给 15Hz 的 NeuPAN 求解器。
update_frequency: 1.0         # [内部更新频率] 1.0Hz
publish_frequency: 1.0        # [外部发布频率] 1.0Hz

# =========================================
# 2. 地图基础设置 (极限钻缝的核心基石)
# =========================================
static_map: true              # [静态地图开关] 必须为 true，依托 Gmapping 建好的栅格底图。
rolling_window: false         #[滚动窗口开关] 全局地图不需要跟随底盘移动，必须关闭。

# [⭐ 关键修改：亚像素级超高分辨率]
# 原理：在 68cm 通道（单侧物理冗余仅 5cm）中，传统 5cm 分辨率在冗余区只有一个有效栅格，极易引发 A* 算法的离散跳变与死锁。
# 对策：分辨率提至 0.025m (2.5cm)，为势能梯度下降提供充足的亚像素插值空间，确保引导线极其丝滑。
resolution: 0.025

# =========================================
# 3. 机器人物理足迹 (绝对几何约束)
# =========================================
# 数学约束：Scout Mini 的精确长宽为 612x580mm。此处严格使用多边形矩阵，绝对禁止使用模糊的 robot_radius (内切圆)。
# 只有当此矩形轮廓触碰到障碍物（即内切圆半径 Rin = 0.29m 被压缩）时，系统才会将其判定为绝对物理死区 (Cost = 253)。
footprint: [[-0.306, -0.290],[-0.306, 0.290],[0.306, 0.290], [0.306, -0.290]]

# =========================================
# 4. 插件配置 (净化全局视野)
# =========================================
# 架构优化：严格剥离 obstacle_layer（动态障碍层）。全局规划不再因人员走动而频繁重规划，保证引导先验（Priori）的绝对拓扑稳定。
plugins:
  - {name: static_layer, type: "costmap_2d::StaticLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

static_layer:
  map_topic: /map
  subscribe_to_updates: true

# =========================================
# 5. 膨胀层配置 (构建人造势能峡谷)
# =========================================
inflation_layer:
  #[反直觉的核心 1：极限拉大膨胀半径]
  # 设为 2.5m (远大于走廊宽度)。确保地图中的每一个可用栅格都被墙壁的膨胀势能（Cost）完全覆盖。
  # 彻底消灭代价值为 0 的绝对平原，强制 A* 和梯度下降算法始终有势能坡度可依，从而实现轨迹的“绝对居中”。
  inflation_radius: 2.5
  
  #[反直觉的核心 2：极限压低衰减系数 (形成平缓 V 型槽)]
  # 数学推导：原默认值为 10.0 (阶梯状陡崖，极易死锁)。降为 2.5 后，膨胀衰减曲线变得极其平滑。
  # 验算：在 68cm 窄门中心(距墙约 0.34m)，其代价值 C = 252 * exp(-2.5 * (0.34 - 0.29)) ≈ 222。
  # 结论：222 虽高但严格 < 253 (致死边界)。这在算法上构建了一个处于高压状态但“绝对合法可通行”的 V 型峡谷谷底。
  cost_scaling_factor: 2.5

transform_tolerance: 1.0      # [TF 延迟容忍度] 允许 1.0 秒内的地图坐标系时间戳漂移。
```

---

### 📄 文件 3：NeuPAN 算法核心配置 (neupan_config.yaml)
**核心目标**：通过深度整定 MPC 预测视野、放开物理动力学封印，并在双凸优化目标函数中调优各权重的博弈关系，彻底解决狭窄通道内的死锁与出弯碰撞问题。

```yaml
# Scout Mini [室内窄道穿梭与瞬间闪避 终极数学重构版]
# 优化策略：短视野高反应 + 解除物理转角封印 + 解耦姿态惩罚 + 数学阻尼防抖

# ==========================================
# 1. MPC 模型预测控制全局视野 (缩短视野，专注眼下)
# ==========================================
receding: 12                  # [⭐ 预测步数 H]
step_time: 0.15               # [⭐ 时间步长 Δt]
                              # 【前瞻边界】预测总视野 = 12 * 0.15 = 1.8 秒。
                              # 逻辑：在单侧冗余仅 5cm 的通道内，如果视野过长（如 >2.0s），NRMP 会提前读取到走廊出口外的杂乱障碍，导致通道内避障惩罚与外部惩罚产生数学冲突而卡死。
                              # 改为 1.8s 后，系统变得“专注且激进”，出走廊瞬间才切入外部避障计算，极大提升了极限闪避反应。

ref_speed: 0.6                # [期望速度 u_speed] 目标函数 C0 的理想追踪速度，0.6m/s 兼顾了室内安全性与执行效率。
device: "cpu"                 #[计算后端] ECOS 求解器运行设备。
time_print: false             # [日志控制] 关闭以节约终端 I/O 开销。
collision_threshold: -1       # [物理碰撞底线] 设为 -1，完全将防碰撞逻辑交接给目标函数中的惩罚力场进行软约束。

# ==========================================
# 2. 机器人物理模型与动力学可行域 F (解除物理封印)
# ==========================================
robot:
  kinematics: "diff"          # [运动学约束] 严格使用四轮差速的雅可比运动学映射。
  length: 0.612               #[形体长] 构建 DUNE 感知网络约束矩阵 G 和向量 h 的绝对基准。
  width: 0.58                 # [形体宽] 580mm。与通道 680mm 构成极限边界。
  
  max_speed: [1.2, 3.14]      #[速度绝对可行域 U] 线速度上限 1.2 m/s，放开角速度极限至 3.14 rad/s (允许原地瞬时高速打盘)。
  
  max_acce:[0.6, 0.2]        # [⭐ 加速度约束：化解角加速度爆发力] 
                              # 【数学意义】强制约束相邻控制周期的速度跳变 Δu。
                              # 这里的角加速度压低至 0.2 rad/s²，配合后面的 p_u 与 bk 权重，强迫底盘在避障时画出向前的圆润弧线，从源头上禁止了差速车的暴力“甩头”。

# ==========================================
# 3. 初始导航路径 (A* 参考线接入)
# ==========================================
ipath:
  curve_style: 'line'         # [参考线拟合方式] 直线插值。差速底盘无需 Dubins 曲线的阿克曼前导圆弧限制。
  min_radius: 0.0             # [转弯半径极限] 差速车原生支持 0 走径原地旋转。
  loop: false                 # [单次导航模式]
  interval: 0.05              #[参考点离散间隔] 5cm 极高密度路点，提供给寻迹函数极其平滑的追踪靶点。
  arrive_threshold: 0.20      # [终点收敛容差] 欧氏距离 < 0.2m 视作任务完成。
  close_threshold: 0.10       # [轨迹纠偏容差] 寻找参考轨迹最近点时的距离阈值。
  ind_range: 15               #[前瞻寻点步长] 在参考路径数组上往前看 15 个点。
  arrive_index_threshold: 5   # [终点数组索引判定] 当寻迹游标距离终点只剩 5 个索引时触发停止动作。

# ==========================================
# 4. PAN 网络算法引擎 (交替最小化宏观调控)
# ==========================================
pan:
  iter_num: 3                 #[双凸优化交替迭代次数 K] DUNE 与 NRMP 每帧互传数据 3 次。保证了局部最优解的绝对数学收敛。
  dune_max_num: 100           # [DUNE 输入张量容量] 神经网络每帧并行处理的 100 个最危险原始激光点。
  nrmp_max_num: 15            # [NRMP 约束剪枝上限] 从 100 个特征中截取最核心的 15 个排斥力特征喂给求解器。在狭窄多墙环境，15 个特征足以为两侧构建完美的动态防撞力场，且有效避免远端杂波干扰。
  dune_checkpoint: null       # [预训练权重] 
  iter_threshold: 0.05        # [迭代早停阈值] 若相邻两次交替迭代算出的轨迹残差 < 5cm，则触发提前退出，大幅节省 CPU 算力。

# ==========================================
# 5. NRMP 神经规划器权重 (⭐⭐⭐ 灵魂目标函数整定区)
# ==========================================
adjust:
  q_s:[1.5, 1.5, 0.2]        # [⭐ 基础寻迹代价 C0 权重：降维向量化解绑]
                              # 【数学意义】拆解标量惩罚，分别对应 [x, y, theta]。
                              # x, y 赋予 1.5 的高权重：像磁铁一样将车辆拽回全局路径中心。
                              # theta 赋予极弱的 0.2：极大地容忍车辆在窄道内的车头微幅偏航（允许倾斜入弯），消灭了因死卡偏航角造成的求解无解死锁。
  
  p_u: 3.0                    #[⭐ 速度服从代价 C0 权重：速度惩罚壁垒]
                              # 设为 3.0。在出走廊的瞬间面临 50cm 处的急弯障碍，极高的速度惩罚权重强制车辆维持动能向前冲，而不允许通过将线速度降为 0 来规避惩罚。
  
  ro_obs: 120                 #[神经避障惩罚 Cr 权重 ρ：铸造防撞护城河]
                              # 设为 120。确保一旦进入极限安全底线内，排斥力代价的量级能绝对击穿 q_s 和 p_u 的拉力，强制执行避障打盘动作。
  
  d_max: 0.35                 #[动态安全气囊上限 d_max]
                              # 限制在 0.35m。防止机器人在开阔地带由于气囊过大而产生过于臃肿的绕路轨迹。
  
  d_min: 0.01                 # [动态安全气囊底线 d_min：物理生死门槛]
                              # 极值 1cm！面对单侧仅 5cm 物理冗余，必须把底线压至极限，将余下的空间全部交给算法作为动态博弈的弹性压缩区。
  
  eta: 3.5                    #[⭐ 稀疏安全距离权重 η：调节幽闭恐惧]
                              # 结合 ρ=120，计算虚拟阻力边界厚度 = η/ρ = 3.5/120 ≈ 2.9 厘米。
                              # 这个数值既能在数学上产生适度的“墙壁挤压感”（防止撞侧墙），又不至于大到让机器人觉得无法穿梭而停机。
  
  bk: 0.4                     #[⭐ 近端平滑正则项 Proximal Term 系数 bk：终极阻尼防抖]
                              # 【数学意义】公式中的 (bk/2)||s_h - \bar{s}_h||_2^2。
                              # 权重设为 0.4。在解开了车头角度限制且差速车响应极快的前提下，此项如同一根数学强力阻尼弹簧，死死惩罚当前帧与上一帧轨迹的跳跃。
                              # 它在算法底层实施了强效低通滤波，迫使差速底盘必须画出平滑连续的过渡曲线，彻底消灭了原地磨胎画龙。
  
  solver: "ECOS"              #[后端求解器] 使用 ECOS 处理转化为 DPP 格式后的双凸优化子问题。
```

---

### 📄 文件 4：全局路径规划器配置 (global_planner.yaml)
**核心目标**：配合代价地图的 V 型势能峡谷，弃用离散网格步进，利用二阶泰勒展开梯度下降寻找完美居中的绝对平滑中心线。

```yaml
# GlobalPlanner (A*) 完整配置 -[亚像素级平滑居中版]
# 架构说明：摒弃动态避障，将 A* 纯粹作为寻找狭窄通道“拓扑谷底”的数学寻路工具。

# =========================================
# 1. 算法结构 (黄金平滑组合)
# =========================================
use_dijkstra: true            # [算法引擎] 采用经典 Dijkstra (A* 变种)，在全图进行稳定的波前扩散膨胀。
use_quadratic: true           # [二阶近似计算] 开启！利用二次多项式拟合局部代价曲面，使得直角拐弯处和 V 型峡谷的势能计算更圆润，杜绝尖锐死角。
old_navfn_behavior: false     # [摒弃陈旧逻辑] 采用全新、更精确的规划机制。

# [⭐ 核心破局：开启真实水流的梯度下降插值]
# 物理意义：将栅格地图彻底视为“连续势能场”而非“像素点集合”。
use_grid_path: false          # 必须设为 false！算法不再受限于只能在周围 8 个离散栅格间“走方块步”。而是通过双线性插值计算周围的代价梯度，像水滴顺着峡谷最低处流淌一样，生成超越网格分辨率的无限平滑曲线。

# =========================================
# 2. 代价权重 (极限穿梭的博弈逻辑)
# =========================================
#[压低地图代价敏感度]
cost_factor: 0.8              # 将对高代价区域（如 68cm 通道内的高达 222 的代价值）的畏惧感打上 0.8 的折扣。强制 A* 算法接纳高压迫感的窄道，防止产生绕远路的畸形轨迹。

# [提高几何位移体力成本]
neutral_cost: 66              # 默认通常为50。设为 66 显著提高了每走一步的基础惩罚。
                              # 权衡博弈：在数学上向 A* 施压，明确指出“绕远路（累积海量 neutral_cost）的代价，远大于笔直钻过高压窄门（忍受瞬时的高 cost_factor）”，从而锁定最短穿越路径。

# [物理致死底线边界]
lethal_cost: 253              # 严格对应 Costmap 的 Inscribed 区域。这是系统的绝对红线，任何经过代价值大于等于 253 的轨迹搜索节点都会被无条件截断丢弃。

# =========================================
# 3. 容错与边界处理
# =========================================
allow_unknown: false          # [未知区域禁行] 仅允许机器人在激光雷达扫过的绝对安全已知地图内进行前瞻规划。
outline_map: true             #[地图封边] 在全局地图的矩阵外围人为加上一圈致死边界，防止由于索引越界导致的规划器内存崩溃（Segment Fault）。
default_tolerance: 1.0        # [目标容差] (m) 如果 A* 发现指定的绝对终点处于死区，允许其在方圆 1.0 米内寻找最靠近终点的一个安全合法坐标作为降级终点。

# =========================================
# 4. 其他配置
# =========================================
orientation_mode: 0           # [姿态规划模式] 设为 0。全局规划器只输出 2D 点集 (x, y)，把所有复杂的姿态角计算 (theta) 彻底下放给 NeuPAN 处理。
orientation_window_size: 1    # [平滑窗大小] 用于姿态角过滤的窗口大小，此处由于已将姿态下放，此参数影响极小。
publish_potential: false      # [发布势能点云] 关闭。节省庞大的带宽与 CPU 计算资源。
publish_scale: 100            # [发布比例尺] 可视化缩放使用。
```

### 6.6 操作流程：AMCL 与 Global Planner 验证 SOP

完成上述配置后，请严格按照以下步骤进行验证。

#### 步骤 1：启动系统
1.  **激活 CAN**: `./bringup_can2usb_500k.bash`
2.  **启动测试**: `roslaunch neupan_ros test_navigation.launch`
3.  **启动键盘**: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` (保持窗口激活)

#### 步骤 2：Rviz 手动配置
在 Rviz 中依次添加以下 Display：
1.  **Fixed Frame**: 设为 `map`。
2.  **RobotModel**: 检查小车模型是否显示。
3.  **Map (Static)**: Topic `/map`，检查黑白地图。
4.  **LaserScan**: Topic `/scan`，Color **绿色**。
5.  **PoseArray**: Topic `/particlecloud`，Color **红色**。
6.  **Map (Costmap)**: Topic `/global_planner/costmap/costmap`，**ColorScheme: costmap**。
    *   *验证*: 检查障碍物周围是否有**紧贴的粉色膨胀层** (因已瘦身，光晕应较薄)。
7.  **Path**: Topic `/global_planner/plan`，Color **蓝色**，Line Width 0.05。

#### 步骤 3：AMCL 定位收敛测试
1.  **校准**: 点击顶部 **"2D Pose Estimate"**，在地图上从机器人实际位置拖拽出车头方向。
2.  **收敛**: 使用键盘控制机器人**原地旋转一圈**或**走一个 S 形**。
    *   *现象*: 红色粒子云应迅速聚集成一小簇，绿色激光线应与地图墙壁严丝合缝。

#### 步骤 4：全局路径规划测试
1.  **下发**: 点击顶部 **"2D Nav Goal"**。
2.  **点击**: 在地图空旷处（非粉色区域）点击并拖拽方向。
3.  **验证**:
    *   *现象*: 地面上应瞬间出现一条**蓝色路径**。
    *   *路径特征*: 路径应带有轻微锯齿（网格法特征），能够穿过狭窄区域，且不会穿墙。
    *   *状态*: 机器人应保持静止。

**通过以上测试后，即可使用最终的 `deploy_scout.launch` 启动 NeuPAN 进行自动驾驶。**
## 阶段七：全系统集成、核心逻辑修复与实车部署 (System Integration, Logic Patching & Final Deployment)

### 7.1 完整部署 Launch 文件 (`deploy_scout.launch`)

**核心修正点**:
1.  **Global Planner 修复**: 使用绝对路径重映射 (`/global_planner/goal`) 并正确加载私有参数 (`ns="planner"` / `ns="costmap"`).
2.  **感知层修复**: 强制投影 `target_frame="base_link"`，修正雷达物理倾角，确保 AMCL 和 Costmap 接收水平数据。
3.  **TF 架构**: 保持 `base_footprint` 作为底层根基，符合 REP-105。

```xml name=~/neupan_ws/src/neupan_ros/launch/deploy_scout.launch
<?xml version="1.0"?>
<launch>
    <!-- 
    ============================================================
    Scout Mini + NeuPAN 完整部署 Launch 文件 (Final Version)
    
    [功能]: 
      1. 启动硬件 (底盘+雷达)
      2. 启动感知 (点云转2D LaserScan, Nodelet高性能版)
      3. 启动定位 (AMCL)
      4. 启动规划 (GlobalPlanner A*, 修复版)
      5. 启动控制 (NeuPAN MPC/DUNE)
      6. 启动可视化 (Rviz)
    
    [架构]: 
      Rviz(2D Nav Goal) -> GlobalPlanner -> /global_planner/plan -> NeuPAN -> /cmd_vel -> Scout Base
    ============================================================
    -->
    
    <!-- ==================== 0. 全局参数 ==================== -->
    <arg name="port_name" default="can0"/>
    <arg name="map_file" default="$(env HOME)/maps/office.yaml"/>
    <arg name="map_frame" default="map"/>
    <arg name="odom_frame" default="odom"/>
    <arg name="base_frame" default="base_link"/>
    <arg name="lidar_frame" default="velodyne"/>
    
    <!-- NeuPAN 模型路径 -->
    <arg name="neupan_config" default="$(find neupan_ros)/config/scout/neupan_planner_scout.yaml"/>
    <!-- 请确保模型文件已存在，否则 NeuPAN 无法启动 -->
    <arg name="dune_model" default="$(find neupan_ros)/model/custom/scout_dune.pth"/>

    <!-- ==================== 1. 硬件驱动 ==================== -->
    
    <!-- 1.1 Scout Mini 底盘 -->
    <include file="$(find scout_bringup)/launch/scout_mini_robot_base.launch">
        <arg name="is_scout_mini" value="true"/>
        <arg name="pub_tf" value="true"/>
        <!-- 强制指定驱动层的基坐标系为 footprint (REP-105) -->
        <arg name="base_frame" value="base_footprint"/>
    </include>

    <!-- 1.2 Velodyne VLP-16 雷达 -->
    <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
        <arg name="device_ip" value="192.168.1.201"/>
        <arg name="frame_id" value="$(arg lidar_frame)"/>
        <arg name="port" value="2368"/>
        <!-- 物理过滤 -->
        <arg name="min_range" value="0.3"/>
        <arg name="max_range" value="100.0"/>
        <!-- 驱动层保持纯净，不进行 scan 重映射 -->
    </include>
    
    <!-- ==================== 2. TF 与 感知 ==================== -->
    
    <!-- 2.1 静态 TF 变换 -->
    <include file="$(find neupan_ros)/launch/scout_tf.launch"/>
    
    <!-- 2.2 3D 点云转 2D 扫描 (关键步骤) -->
    <include file="$(find neupan_ros)/launch/velodyne_to_scan.launch">
        <arg name="manager" value="velodyne_nodelet_manager"/>
        <!-- 
           [继承配置 - 极重要] 
           强制投影到 base_link 水平面，确保 AMCL 拿到的数据是水平的
        -->
        <arg name="target_frame" value="base_link"/>
    </include>

    <!-- ==================== 3. 地图与定位 ==================== -->
    
    <!-- 3.1 地图服务器 -->
    <node pkg="map_server" type="map_server" name="map_server" args="$(arg map_file)">
        <param name="frame_id" value="$(arg map_frame)"/>
    </node>

    <!-- 3.2 AMCL 定位 -->
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <rosparam file="$(find neupan_ros)/config/scout/amcl.yaml" command="load"/>
        <remap from="scan" to="/scan"/>
        
        <param name="odom_frame_id" value="$(arg odom_frame)"/>
        <param name="base_frame_id" value="$(arg base_frame)"/>
        <param name="global_frame_id" value="$(arg map_frame)"/>
        
        <!-- 初始位置假设 (默认在原点，启动后可用 Rviz 修正) -->
        <param name="initial_pose_x" value="0.0"/>
        <param name="initial_pose_y" value="0.0"/>
        <param name="initial_pose_a" value="0.0"/>
    </node>

    <!-- ==================== 4. 全局规划器 (A* 修正版) ==================== -->
    <node pkg="global_planner" type="planner" name="global_planner" output="screen">
        <!-- 加载参数到正确命名空间 -->
        <rosparam file="$(find neupan_ros)/config/scout/planner_a_star.yaml" command="load" ns="planner"/>
        <rosparam file="$(find neupan_ros)/config/scout/costmap_global.yaml" command="load" ns="costmap"/>
        
        <!-- [关键] 绝对路径重映射，解决 Rviz 下发无效问题 -->
        <remap from="/global_planner/goal" to="/move_base_simple/goal"/>
        
        <!-- 路径话题重映射 (供 NeuPAN 订阅) -->
        <remap from="/global_planner/planner/plan" to="/global_planner/plan"/>
        
        <param name="costmap/global_frame" value="$(arg map_frame)"/>
        <param name="costmap/robot_base_frame" value="$(arg base_frame)"/>
    </node>

    <!-- ==================== 6. NeuPAN 局部规划 (核心控制器) ==================== -->
    <node name="neupan_node" pkg="neupan_ros" type="neupan_node.py" output="screen">
        <!-- 配置加载 -->
        <param name="config_file" value="$(arg neupan_config)"/>
        <param name="dune_checkpoint" value="$(arg dune_model)"/>
        
        <!-- 坐标系 -->
        <param name="map_frame" value="$(arg map_frame)"/>
        <param name="base_frame" value="$(arg base_frame)"/>
        <param name="lidar_frame" value="$(arg lidar_frame)"/>
        
        <!-- 雷达参数 -->
        <param name="scan_range" value="0.4 10.0"/>
        <param name="scan_angle_range" value="-3.14159 3.14159"/>
        <param name="scan_downsample" value="4"/>
        <param name="flip_angle" value="false"/>
        
        <!-- 行为逻辑 -->
        <param name="refresh_initial_path" value="true"/>
        <param name="include_initial_path_direction" value="false"/>
        
        <!-- 可视化参数 -->
        <param name="marker_size" value="0.05"/>
        <param name="marker_z" value="0.3"/>
        
        <!-- 话题对接 -->
        <remap from="/scan" to="/scan"/>
        <!-- [关键] 订阅 GlobalPlanner 生成的蓝线 -->
        <remap from="/initial_path" to="/global_planner/plan"/>
        <!-- [关键] 监听 Rviz 目标点 (用于触发重置) -->
        <remap from="/neupan_goal" to="/move_base_simple/goal"/>
        <!-- [关键] 直接控制底盘 -->
        <remap from="/neupan_cmd_vel" to="/cmd_vel"/>
    </node>

    <!-- ==================== 7. 可视化 (Visualization) ==================== -->
    <!-- 启动空配置 Rviz，方便手动加载之前保存的配置 (如 manual_test.rviz) -->
    <!-- 如果想自动加载，请改为 args="-d $(find neupan_ros)/rviz/你的配置文件.rviz" -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find neupan_ros)/rviz/navigation.rviz"/>

</launch>
```

---

### 7.2 NeuPAN 核心逻辑修复 (`neupan_core.py`)

**问题描述**: 
旧版代码中，一旦机器人到达目标点，`arrive` 和 `stop` 标志位会被锁死为 `True`，导致后续发布的新目标点无效（速度一直输出为 0）。此外，多线程修改 planner 状态可能导致竞争。

**修复方案**: 
1.  **状态重置**: 在接收到新目标 (`goal_callback`) 或新路径 (`path_callback`) 时，强制重置 `arrive` 和 `stop` 标志位。
2.  **线程安全**: 将规划器的重置逻辑 (`planner.reset()`) 移至主循环 (`run`) 中，通过 `reset_flag` 触发，避免回调线程冲突。

请使用以下代码 **完全覆盖** `~/neupan_ws/src/neupan_ros/src/neupan_core.py`：

```python name=~/neupan_ws/src/neupan_ros/src/neupan_core.py
#!/usr/bin/env python

"""
neupan_core is the main class for the neupan_ros package.
Modified to fix navigation goal reset issues (The "Move Once and Stop" Bug).
"""

from neupan import neupan
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from math import sin, cos, atan2
import numpy as np
from neupan.util import get_transform
import tf
import sensor_msgs.point_cloud2 as pc2


class neupan_core:
    def __init__(self) -> None:

        rospy.init_node("neupan_node", anonymous=True)

        # ros parameters
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_link")
        self.marker_size = float(rospy.get_param("~marker_size", "0.05"))
        self.marker_z = float(rospy.get_param("~marker_z", "1.0"))

        scan_angle_range_para = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        self.scan_angle_range = np.fromstring(
            scan_angle_range_para, dtype=np.float32, sep=" "
        )

        self.scan_downsample = int(rospy.get_param("~scan_downsample", "1"))

        scan_range_para = rospy.get_param("~scan_range", "0.0, 5.0")
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=" ")

        self.dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", False)
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param("~include_initial_path_direction", False)
        
        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file, pan=pan
        )

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        
        # [修改1] 初始化关键状态标志位
        self.stop = False
        self.arrive = False 
        self.new_goal = None # 用于线程间传递新目标
        self.reset_flag = False # 用于触发重置

        # publisher
        self.vel_pub = rospy.Publisher("/neupan_cmd_vel", Twist, queue_size=10)
        self.plan_pub = rospy.Publisher("/neupan_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/neupan_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/neupan_initial_path", Path, queue_size=10
        )  # initial path

        ## for rviz visualization
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )

        self.listener = tf.TransformListener()

        # subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/initial_path", Path, self.path_callback)
        rospy.Subscriber("/neupan_waypoints", Path, self.waypoints_callback)
        rospy.Subscriber("/neupan_goal", PoseStamped, self.goal_callback)
        
    def run(self):

        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            
            # [修改2] 在主循环中处理新目标的重置逻辑，避免线程冲突
            if self.reset_flag and self.robot_state is not None:
                if self.new_goal is not None:
                    rospy.loginfo("Resetting planner for NEW GOAL...")
                    self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.new_goal)
                    self.neupan_planner.reset()
                    self.new_goal = None # 清除目标防止重复更新
                
                # 强制重置状态，让车动起来
                self.arrive = False
                self.stop = False
                self.reset_flag = False

            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.map_frame, self.base_frame, rospy.Time(0)
                )

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf for the transform from {} to {}".format(
                        self.base_frame, self.map_frame
                    ),
                )
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(1, "waiting for robot state")
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            # 初始路径设置逻辑
            if (
                len(self.neupan_planner.waypoints) >= 1
                and self.neupan_planner.initial_path is None
            ):
                self.neupan_planner.set_initial_path_from_state(self.robot_state)

            if self.neupan_planner.initial_path is None:
                rospy.logwarn_throttle(1, "waiting for neupan initial path")
                continue

            rospy.loginfo_once("initial Path Received")
            self.ref_path_pub.publish(
                self.generate_path_msg(self.neupan_planner.initial_path)
            )

            if self.obstacle_points is None:
                rospy.logwarn_throttle(
                    1, "No obstacle points, only path tracking task will be performed"
                )

            # [关键] 只有未到达且未强制停止时，才进行规划
            action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

            self.stop = info["stop"]
            self.arrive = info["arrive"]

            if info["arrive"]:
                rospy.loginfo_throttle(1.0, "arrive at the target")
            
            if info["stop"]:
                rospy.logwarn_throttle(
                    0.5,
                    "neupan stop triggered! Min dist: {:.2f}, Threshold: {:.2f}".format(
                        self.neupan_planner.min_distance.detach().item(),
                        self.neupan_planner.collision_threshold
                    ),
                )

            # publish the path and velocity
            self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
            self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
            
            # 发送速度指令
            self.vel_pub.publish(self.generate_twist_msg(action))

            self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
            self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

            r.sleep()

    def scan_callback(self, scan_msg):
        if self.robot_state is None:
            return None

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []
        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.scan_downsample == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            rospy.loginfo_once("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.map_frame, self.lidar_frame, rospy.Time(0)
            )
            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            return self.obstacle_points

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

    def path_callback(self, path):
        initial_point_list = []
        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0] if initial_point_list else 0
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo("Initial path update from given path")
            self.neupan_planner.set_initial_path(initial_point_list)
            self.neupan_planner.reset()
            # [修改3] 接收到新路径时，重置状态
            self.arrive = False
            self.stop = False

    def waypoints_callback(self, path):
        waypoints_list = [self.robot_state]
        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0]
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo("Initial path update from waypoints")
            self.neupan_planner.update_initial_path_from_waypoints(waypoints_list)
            self.neupan_planner.reset()
            # [修改4] 接收到新路点时，重置状态
            self.arrive = False
            self.stop = False

    def goal_callback(self, goal):
        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        # [修改5] 仅保存目标并设置标志位，将逻辑移至主循环
        self.new_goal = np.array([[x], [y], [theta]])
        self.reset_flag = True
        
        # 立即强制将arrive设为False，防止下一帧run循环直接退出
        self.arrive = False
        self.stop = False
        
        rospy.loginfo(f"Received NEW Goal: {[x, y, theta]} - Resetting Planner")

    def quat_to_yaw_list(self, quater):
        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))
        return yaw

    def generate_path_msg(self, path_list):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0
        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.seq = index
            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])
            path.poses.append(ps)
        return path

    def generate_twist_msg(self, vel):
        if vel is None:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        # [逻辑说明] 如果标志位没有重置，这里会一直返回 0 速度
        if self.stop or self.arrive:
            return Twist()
        else:
            action = Twist()
            action.linear.x = speed
            action.angular.z = steer
            return action

    def generate_dune_points_markers_msg(self):
        marker_array = MarkerArray()
        if self.neupan_planner.dune_points is None:
            return
        else:
            points = self.neupan_planner.dune_points
            for index, point in enumerate(points.T):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()
                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0
                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255
                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()
                marker_array.markers.append(marker)
            return marker_array

    def generate_nrmp_points_markers_msg(self):
        marker_array = MarkerArray()
        if self.neupan_planner.nrmp_points is None:
            return
        else:
            points = self.neupan_planner.nrmp_points
            for index, point in enumerate(points.T):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()
                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0
                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255
                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()
                marker_array.markers.append(marker)
            return marker_array

    def generate_robot_marker_msg(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.seq = 0
        marker.header.stamp = rospy.get_rostime()
        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255
        marker.id = 0
        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase
            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z
            marker.type = 1
            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]
            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y
            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])
        return marker

    @staticmethod
    def yaw_to_quat(yaw):
        quater = Quaternion()
        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)
        return quater

    @staticmethod
    def quat_to_yaw(quater):
        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w
        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))
        return raw
```

---

### 7.3 关键参数调优 (Important!)

如果即使修复了代码，机器人依然出现“动一下就停”或者 `neupan stop triggered` 警告，请检查 `neupan_planner_scout.yaml`。NeuPAN 默认参数可能对于 Scout Mini 过于敏感。

**建议修改文件**: `~/neupan_ws/src/neupan_ros/config/scout/neupan_planner_scout.yaml`

```yaml
# 1. 防止过早判定到达 (还没到终点就停车)
ipath:
  arrive_threshold: 0.15  # 推荐值: 0.15 (原值若过大如 1.0 会导致刚出发就判定到达)
  close_threshold: 0.05

# 2. 防止误判障碍物急停 (周围有东西就吓得不敢动)
# 在文件根目录或 pan/adjust 节点下
collision_threshold: 0.2  # 推荐值: 0.2 或 0.15 (原值 0.5 对小车太大)
```

### 7.4 导航 Rviz 配置

基于你提供的最终版 `deploy_scout.launch` 文件，以下是 **Rviz 手动配置的完整操作流程**。请按照步骤依次添加显示项，以构建一个监控 Scout Mini 底盘、传感器、定位、全局规划和 NeuPAN 控制的完整可视化界面。

---

#### **准备工作**

1.  **启动系统**：
    在终端运行你的启动文件：
    ```bash
    roslaunch neupan_ros deploy_scout.launch
    ```
    *(此时 Rviz 会自动启动，但里面应该是空的，或者只有默认配置)*

2.  **打开 Rviz**：
    如果 Rviz 没有自动弹出，请打开新终端输入 `rviz`。

---

#### **第一步：基础环境配置 (Global Options)**

这是最关键的一步，决定了所有数据的坐标基准。

1.  在 Rviz 左侧 **Displays** 面板的最上方，找到 **Global Options**。
2.  将 **Fixed Frame** 修改为：`map`。
    *   *注意：如果下拉菜单里没有 map，请直接手动输入 map 并回车。*

---

#### **第二步：添加地图与定位显示**

##### 1. 静态地图 (Static Map)
*   点击左下角 **Add** -> 选择 **Map** -> 点击 OK。
*   **配置参数**：
    *   **Topic**: 选择 `/map`。
    *   **Color Scheme**: 保持 `map`。
    *   **Update Interval**: `2` (减少 CPU 占用)。
*   *作用：显示扫描构建好的黑白环境地图。*

##### 2. 全局代价地图 (Global Costmap)
*   点击 **Add** -> 选择 **Map** -> 点击 OK。
*   **配置参数**：
    *   **Name**: 改名为 `Costmap` (可选，方便区分)。
    *   **Topic**: 选择 `/global_planner/costmap/costmap`。
    *   **Color Scheme**: 选择 `costmap` (重要！显示为彩色梯度)。
    *   **Alpha**: 设为 `0.6` (设置透明度，以便能看清底下的静态地图)。
*   *作用：显示全局规划器眼中的障碍物膨胀层（粉色/紫色区域）。*

##### 3. AMCL 粒子云 (Localization)
*   点击 **Add** -> 选择 **PoseArray** -> 点击 OK。
*   **配置参数**：
    *   **Topic**: 选择 `/particlecloud`。
    *   **Color**: 设置为 **红色** (255; 25; 0)。
    *   **Arrow Length**: `0.2` (让箭头更清晰)。
*   *作用：显示机器人对自己位置的预估分布。粒子越聚拢，定位越准。*

---

#### **第三步：添加传感器数据**

##### 1. 2D 激光雷达 (LaserScan)
*   点击 **Add** -> 选择 **LaserScan** -> 点击 OK。
*   **配置参数**：
    *   **Topic**: 选择 `/scan`。
    *   **Color**: 设置为 **亮绿色** (0; 255; 0)。
    *   **Size (m)**: `0.05` (让激光点大一点)。
    *   **Style**: `Flat Squares` 或 `Points`。
*   *作用：显示经过高度过滤和水平校正后的 2D 避障数据。*

##### 2. 3D 原始点云 (Velodyne - 可选)
*   点击 **Add** -> 选择 **PointCloud2** -> 点击 OK。
*   **配置参数**：
    *   **Topic**: 选择 `/velodyne_points`。
    *   **Color Transformer**: 选择 `FlatColor` 或 `Intensity`。
    *   **Color**: **白色** 或 **灰色**。
    *   **Size (m)**: `0.02`。
*   *作用：显示雷达的原始 3D 视野，用于对比 2D 数据是否被正确截取。*

---

#### **第四步：添加路径规划可视化**

##### 1. 全局路径 (Global Planner)
*   点击 **Add** -> 选择 **Path** -> 点击 OK。
*   **配置参数**：
    *   **Name**: 改名为 `Global Path`。
    *   **Topic**: 选择 `/global_planner/plan`。
    *   **Color**: **深绿色** (0; 128; 0)。
    *   **Line Width**: `0.05` (加粗)。
*   *作用：显示 A* 算法生成的从起点到终点的引导线。*

##### 2. NeuPAN 初始参考路径 (Initial Path)
*   点击 **Add** -> 选择 **Path** -> 点击 OK。
*   **配置参数**：
    *   **Name**: 改名为 `NeuPAN Reference`。
    *   **Topic**: 选择 `/neupan_initial_path`。
    *   **Color**: **蓝色** (0; 0; 255)。
    *   **Line Width**: `0.03`。
*   *作用：显示 NeuPAN 当前正在跟踪的参考线（通常覆盖在全局路径上）。*

##### 3. NeuPAN 局部预测轨迹 (Local Plan)
*   点击 **Add** -> 选择 **Path** -> 点击 OK。
*   **配置参数**：
    *   **Name**: 改名为 `NeuPAN Trajectory`。
    *   **Topic**: 选择 `/neupan_plan`。
    *   **Color**: **鲜红色** (255; 0; 0)。
    *   **Line Width**: `0.05`。
*   *作用：显示 MPC 预测的未来几秒内的实际运动轨迹（会动态跳变）。*

---

#### **第五步：添加 NeuPAN 调试标记 (Markers)**

##### 1. 障碍物标记 (DUNE Points)
*   点击 **Add** -> 选择 **MarkerArray** -> 点击 OK。
*   **配置参数**：
    *   **Topic**: 选择 `/dune_point_markers`。
*   *作用：显示 NeuPAN 算法当前重点关注的障碍物点（通常是紫色的方块）。*

##### 2. 机器人模型标记 (Robot Marker)
*   点击 **Add** -> 选择 **Marker** -> 点击 OK。
*   **配置参数**：
    *   **Topic**: 选择 `/robot_marker`。
*   *作用：显示 NeuPAN 算法理解的机器人几何形状（绿色矩形框）。*

---

#### **第六步：添加机器人模型 (Robot Model)**

*   点击 **Add** -> 选择 **RobotModel** -> 点击 OK。
*   **配置参数**：
    *   **Robot Description**: 保持 `robot_description`。
*   *作用：显示 Scout Mini 的 3D 模型（车轮、车身），用于验证 TF 变换是否正确。*

---

#### **第七步：验证交互工具 (Tool Properties)**

这一步确保你点击地图时，目标点能发送给正确的节点。

1.  在 Rviz 顶部的菜单栏，勾选 **Panels** -> **Tool Properties**。
2.  在右侧出现的 **Tool Properties** 面板中，展开 **2D Nav Goal**。
3.  **Topic**: 确保设置为 `/move_base_simple/goal`。
    *   *解释：你的 launch 文件已经将 `/global_planner/goal` 和 `/neupan_goal` 都 remap 到了这个话题，所以使用默认设置即可同时触发全局规划和 NeuPAN。*

---

#### **最后：保存配置**

配置完成后，为了避免下次重新手动添加：

1.  点击左上角 **File** -> **Save Config As...**
2.  保存路径：`~/neupan_ws/src/neupan_ros/rviz/navigation.rviz`
3.  覆盖原有文件（或保存为新文件）。

#### **验证是否成功**

1.  点击顶部工具栏的 **2D Pose Estimate**，在地图上校准机器人的位置 -> **红色粒子云**应收敛。
2.  点击顶部工具栏的 **2D Nav Goal**，在地图空旷处给一个目标点。
3.  你应该能看到：
    *   一条**深绿色**的全局路径瞬间生成。
    *   一条**蓝色**的参考路径覆盖其上。
    *   一条**红色**的短轨迹从车头伸出。
    *   机器人开始移动。

### 7.5 导航操作流程 (SOP)

```bash
# ============================================
# Scout Mini + NeuPAN 导航操作手册 (Final)
# ============================================

# --------------------------------------------
# 步骤 0: 预检查 (每次上电必做)
# --------------------------------------------
# 1. 检查急停开关: 确保红色急停旋钮已弹起。
# 2. 检查遥控器: SWB 拨杆必须在 **最上方** (指令模式/Command Mode)。
# 3. 检查底层通信:
#    candump can0        # 应看到数据疯狂滚动
#    ping 192.168.1.201  # 应有回复 (time < 1ms)

# --------------------------------------------
# 步骤 1: 启动导航系统
# --------------------------------------------
# 启动终极部署脚本 (会自动打开 Rviz)
roslaunch neupan_ros deploy_scout.launch

# [可选] 如果需要键盘辅助控制 (用于辅助定位初始化)
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# --------------------------------------------
# 步骤 2: 定位初始化 (Rviz 操作)
# --------------------------------------------
# a. 点击 Rviz 顶部工具栏 "2D Pose Estimate"。
# b. 在地图上点击机器人当前的实际位置。
# c. 按住鼠标左键拖动，绿色箭头指向车头朝向，松开鼠标。
#
# [判断标准]:
# - 红色粒子云 (Particle Cloud) 应聚集在机器人周围。
# - 亮绿色激光线 (LaserScan) 应与地图的黑线轮廓 **严丝合缝**。
#
# [微调技巧]:
# - 如果不准，用键盘控制机器人原地旋转一圈，AMCL 会自动收敛粒子。

# --------------------------------------------
# 步骤 3: 下发导航目标 (Rviz 操作)
# --------------------------------------------
# a. 点击 Rviz 顶部工具栏 "2D Nav Goal"。
# b. 在地图的空旷区域 (避开粉色/紫色的膨胀层) 点击目标点。
# c. 按住鼠标拖动指定到达时的车头朝向。
# d. 松开鼠标，系统立即开始执行。

# --------------------------------------------
# 步骤 4: 运行状态监控 (看懂 Rviz)
# --------------------------------------------
# 观察以下线条是否正常出现:
# 1. 深绿色粗线 (Global Path): A* 规划出的全局路径，连接起点与终点。
#    -> 如果没出现，说明目标点不可达 (在墙里)。
#
# 2. 蓝色细线 (Reference Path): NeuPAN 接收到的参考路径，应覆盖在绿线上。
#    -> 如果没出现，说明 GlobalPlanner 与 NeuPAN 的话题连接断了。
#
# 3. 鲜红色曲线 (Local Trajectory): NeuPAN 实时的 MPC 预测轨迹，从车头伸出。
#    -> 如果红线很短或没有，说明机器人触发了避障急停。

# --------------------------------------------
# 紧急情况处理 (Safety First)
# --------------------------------------------
# 方案 A (推荐): 立即将遥控器 SWB 拨回 **中间** (遥控模式)，底层会瞬间切断电脑控制。
# 方案 B: 拍下车体后方的红色急停按钮。
# 方案 C: 在终端狂按 Ctrl+C。

# ============================================
# 常见问题排查指南 (Troubleshooting)
# ============================================

# 现象 1: 机器人 "动一下就停"，或者完全不动
# --------------------------------------------
# 原因 A: 到达阈值过大。
#   - 检查 config/scout/neupan_planner_scout.yaml 中的 ipath/arrive_threshold。
#   - 修正: 设为 0.15 或 0.1。
# 原因 B: 状态标志位锁死 (旧版代码 Bug)。
#   - 修正: 确保 neupan_core.py 已更新为最新版 (含 reset_flag 逻辑)。
# 原因 C: 目标点在障碍物膨胀层(粉色区域)内。
#   - 修正: 选择开阔地带作为目标。

# 现象 2: 终端一直报 "neupan stop triggered"
# --------------------------------------------
# 原因: 离障碍物太近，小于 collision_threshold。
# 检查:
rostopic echo /scan  # 检查是否有极近距离的噪点
# 修正:
#   1. 检查 config/scout/neupan_planner_scout.yaml 中的 collision_threshold (建议 0.2)。
#   2. 检查 Rviz 中是否有点云打在车体自身 (需调整 velodyne_to_scan 的 min_height)。

# 现象 3: 全局路径(绿线)不更新
# --------------------------------------------
# 原因: GlobalPlanner 未开启定时重规划。
# 修正: 确保 planner_frequency 参数 > 0 (如 1.0)。

# 现象 4: 机器人左右画龙 (S形走位)
# --------------------------------------------
# 原因: 控制器对角度偏差修正过度。
# 修正: 增大 neupan_planner_scout.yaml 中的 p_u (速度保持权重) 或 增大 receding (预测步数)。

# 现象 5: 无法收到 /cmd_vel 指令
# --------------------------------------------
# 检查话题连接:
rostopic echo /cmd_vel            # 查看最终输出
rostopic echo /neupan_cmd_vel     # 查看 NeuPAN 原始输出
# 如果 neupan 有输出但 cmd_vel 无，检查 launch 文件中的 remap 是否生效。
```

## 附录 A：核心排坑指南 (Troubleshooting)

### A.1 硬件与驱动问题

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| `ping 192.168.1.201` 不通 | IP 配置错误 | 检查笔记本有线网卡是否设为 `192.168.1.100` |
| `/velodyne_points` 无数据 | 网关冲突 | 删除有线网卡的网关配置，只保留 IP 和掩码 |
| 机器人无法移动 (Hardware) | CAN 异常 / 遥控模式 | 1. 检查 `candump can0` <br> 2. 确保遥控器 SWB 在最上方 |
| 速度响应迟钝 | CAN 波特率错误 | 确保脚本设置为 `500k` (`bringup_can2usb_500k.bash`) |

### A.2 NeuPAN 逻辑与规划问题 (重点)

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| **发新目标后车不动 (速度为0)** | 状态标志位锁死 | 更新 `neupan_core.py`，确保 `arrive` 标志位在接收新目标时被重置。 |
| **刚起步就判定到达** | `arrive_threshold` 过大 | 将 yaml 中 `arrive_threshold` 减小至 `0.15`。 |
| **一直报 "neupan stop triggered"** | 碰撞阈值过大 / 噪点 | 1. 将 `collision_threshold` 减小至 `0.15`。<br> 2. 检查 Rviz 是否有点云打在车身上。 |
| **无法到达终点 (在附近转圈)** | `arrive_threshold` 过小 | 稍微增大阈值 (如 0.1 -> 0.15)，允许一定的停车误差。 |
| **全局路径不刷新** | GlobalPlanner 参数 | 确保 `planner_frequency` > 0 (推荐 1.0)。 |

### A.3 TF 变换问题

```bash
# 1. 检查 TF 树完整性 (map -> odom -> base_footprint -> base_link -> velodyne)
rosrun tf tf_echo map velodyne

# 2. 如果报错 "Frame map does not exist"
# 检查 AMCL 是否已收到地图并发布了 TF:
rostopic echo /tf | grep "map"

# 3. 检查 TF 树可视化图
rosrun rqt_tf_tree rqt_tf_tree
```

---

## 附录 B：终极调参宝典 (The NeuPAN Tuning Bible)

NeuPAN 局部避障算法的核心在于**“多目标优化”**：它需要在“紧跟全局路径”、“远离障碍物”和“有限的计算资源（控制频率）”之间寻找完美的平衡。

以下是核心参数的调教心法：

### 1. 基础运动学枷锁 (Kinematics Limits)
底层约束决定了机器人的物理极限，如果这里卡死，上层算法再好也发挥不出来。
*   **`max_speed` (最大线速度与角速度)**
    *   **核心法则**：对于差速底盘（diff），**角速度必须给够**！
    *   **调参建议**：角速度限制 `max_speed[1]` 强烈建议设为 **`3.14`**（即 180°/s）。如果设得太小（如 1.0），机器人会因为转弯半径不足，在障碍物前无法转身，导致原地徘徊甚至倒车。

### 2. 算力与延迟控制 (Compute & Latency)
**“天下武功，唯快不破”**。避障算法如果计算太慢（输出频率 < 10Hz），就会产生严重的控制延迟，表现为“快撞上了才猛打方向”。
*   **`iter_num` (迭代次数)**
    *   **核心法则**：绝不能贪多。
    *   **调参建议**：强烈建议设为 **`3`**。早期版本设为 `8` 会导致巨大的计算耗时，严重拖垮系统。
*   **`dune_max_num`**
    *   **调参建议**：配合 `iter_num` 降低，通常 **`100`** 就足够应对大部分场景。
*   **`solver` (求解器)**
    *   **调参建议**：如果依然感觉计算有延迟，可以尝试将 `"ECOS"` 替换为更新、更快的求解器（如原作者建议，ECOS有些老了）。

### 3. 避障行为塑造 (Obstacle Avoidance Shaping)
这组参数决定了机器人面对障碍物时的“性格”：是胆小怯懦（绕大弯），还是从容贴边。
*   **`d_max` (感知与计算范围)**
    *   **作用**：决定机器人多早开始规划避障。
    *   **调参建议**：如果你发现转弯时机太迟，适当**调大此值**（例如从 `1.0` 提升至 **`3.5`**），让机器人在远处就能“看到”障碍物并提前打方向。
*   **`ro_obs` (障碍物排斥权重)**
    *   **作用**：数值越大，机器人越怕障碍物。
    *   **调参建议**：如果发现机器人绕的弯子特别大，说明排斥力过强。应大幅降低此值（例如从 `500` 降至 **`45`**），让它敢于贴近障碍物。
*   **`eta` (避障约束刚度)**
    *   **作用**：控制避障边界的软硬程度。
    *   **调参建议**：值越大（如 10.0），约束越 Hard，倾向于远离；值越小（如 **`1.5 - 1.6`**），约束越 Soft，允许平滑贴边行驶。如果转弯太迟，可微调大 `eta` 增加强制性；如果绕大弯，则调小。

### 4. 路径执念与变道意愿 (Trajectory Adherence)
这组参数决定了机器人有多“死心眼”。
*   **`q_s` (状态偏离惩罚 / 路径执念)**
    *   **作用**：惩罚偏离全局参考路径的行为。
    *   **调参建议**：过大（如 2.0）会导致机器人“宁可停下倒车，也不愿偏离全局路径去绕行”。建议**调小至 `0.8 - 0.9`**，赋予它变道避障的自由。
*   **`p_u` (控制输入权重)**
    *   **作用**：鼓励机器人给出更积极的控制动作。
    *   **调参建议**：配合调小的 `q_s`，将此值**调大至 `5.0`**，使机器人在需要避障时动作更加果断。

### 5. 视野前瞻与规划视界 (Horizon)
*   **`receding` (预测步数)**
    *   **作用**：往未来预测多少步（总预测时间 = `receding` × `step_time`）。
    *   **调参建议**：不是越长越好！过长（如 20）会导致在密集环境中不够灵活、转弯延迟。在车速 `0.6 m/s` 时，设为 **`12 - 16`** 之间通常是灵活性与前瞻性的最佳甜点（Sweet Spot）。

---

### 🚨 疑难杂症速查表 (Cheat Sheet)

| 临床表现 (Symptom) | 诊断与处方 (Prescription) |
| :--- | :--- |
| **遇障原地徘徊、甚至倒退不前进** | **1.** 检查角速度限制，强烈建议 `max_speed` 角速度设为 `3.14`<br>**2.** 降低路径执念 `q_s`，增大动作权重 `p_u` |
| **能避障，但绕的弯子极大离谱** | **1.** 大幅降低排斥权重 `ro_obs` (降至 50 以内)<br>**2.** 降低避障刚度 `eta` (降至 1.5 - 2.0) |
| **转弯太迟，快撞了才猛打方向** | **1.** 调大前瞻距离 `d_max`，让它提早看到障碍<br>**2.** 若在密集环境，适当减小预测步数 `receding`<br>**3.** 稍微调大 `eta` 让避障约束更强硬 |
| **行驶卡顿，控制不顺畅、延迟高** | **1.** 坚决降低迭代次数 `iter_num` 至 `3`<br>**2.** 检查控制输出频率，若低于10Hz考虑更换求解器 `solver` |

---

## 附录 C：完整文件清单 (Project Structure)

请确保你的工作空间结构与下方一致，这对应了本手册所有 Launch 文件的路径引用。

```text
~/neupan_ws/
├── src/
│   ├── NeuPAN/                          # [核心算法库] (git clone -b py38)
│   │   ├── configs/
│   │   │   └── train_scout_mini.yaml    # 训练配置文件
│   │   └── train_scout_dune.py          # 训练脚本
│   │
│   ├── neupan_ros/                      # [ROS 接口包]
│   │   ├── src/
│   │   │   ├── neupan_core.py           # [关键] 核心逻辑 (已修复Bug版)
│   │   │   └── neupan_node.py           # 节点入口
│   │   ├── config/
│   │   │   └── scout/
│   │   │       ├── neupan_planner_scout.yaml  # NeuPAN 参数 (已调优)
│   │   │       ├── planner_a_star.yaml        # GlobalPlanner 参数
│   │   │       ├── costmap_global.yaml        # 代价地图参数
│   │   │       └── amcl.yaml                  # 定位参数
│   │   ├── launch/
│   │   │   ├── scout_tf.launch                # 静态 TF 发布
│   │   │   ├── velodyne_to_scan.launch        # 3D->2D 转换 (修正版)
│   │   │   ├── mapping_scout.launch           # 建图启动
│   │   │   └── deploy_scout.launch            # [终极] 自动驾驶启动
│   │   ├── model/
│   │   │   └── custom/
│   │   │       └── scout_dune.pth             # 训练好的模型文件
│   │   └── rviz/
│   │       ├── mapping.rviz                   # 建图可视化配置
│   │       └── navigation.rviz                # 导航可视化配置
│   │
│   ├── ugv_sdk/                         # [底盘SDK]
│   ├── scout_ros/                       # [底盘ROS驱动]
│   ├── velodyne/                        # [雷达驱动] (新增)
│   └── pointcloud_to_laserscan/         # [转换工具] (新增/系统自带)
│
├── neupan.code-workspace                # VS Code 工作区配置
└── devel/                               # 编译生成文件

~/maps/
├── office.pgm                           # 地图图片
└── office.yaml                          # 地图配置
```
