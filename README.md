# ICAN 应急救援机器人挑战赛（陆地障碍物清除任务）

参赛资料 + ROS 2 仿真工作空间。仓库同时承载**竞赛文档**和**示例代码**，队友 clone 后按下方步骤即可构建运行。

> 详细任务拆解见 [任务说明书-每个任务到底做什么.md](任务说明书-每个任务到底做什么.md)
> 判分口径见 [判分细则V1.0_文本版.txt](判分细则V1.0_文本版.txt) 与 [裁判系统判分细则 PDF](应急救援机器人挑战赛裁判系统判分细则V1.0.pdf)

---

## 仓库结构

```
.
├── demo_shoot_ws/demo_shoot_ws/     # ROS 2 工作空间（核心）
│   ├── assets/point_targets/         # 10 张地面候选贴图（5 类 × 2）
│   ├── docs/                         # 仿真环境说明、协作说明、分工表
│   ├── packages/                     # 仿真 deb 包
│   ├── scripts/                      # 安装 / 运行 / 换贴图脚本
│   └── src/                          # 三个示例 ROS 2 包
│       ├── ican_example_navigation/    # Nav2 导航
│       ├── ican_example_ring_aim/      # 环形靶瞄准
│       └── ican_example_mission/       # 任务主流程
├── .vscode/                          # VS Code 配置（隐藏构建产物、WSL 终端）
├── analyze_wheels.py                 # 麦轮 STL 分析（判断四个轮子是否同款）
├── make_explain_image.py             # 渲染"素材原图 vs 相机实拍"对比图
├── 任务说明书-每个任务到底做什么.md
├── 判分细则V1.0_文本版.txt
├── 团队分工表-含任务说明.xlsx
└── ican-rescue-robot.code-workspace   # 双击即可用 VS Code 打开本仓库
```

---

## 地面贴图是干什么的（高频疑问）

`assets/point_targets/` 里的 10 张图 = **5 个类别 × 每类 2 张候选**：

| 类别 | 中文 | 靶位号 |
|---|---|---|
| `collapsed_ruins` | 坍塌废墟 | 1 |
| `landslide` | 滑坡塌方 | 2 |
| `trapped_people` | 被困人员 | 3 |
| `damaged_vehicle` | 受损车辆 | 4 |
| `rescue_camp` | 救援营地 | 5 |

**用途一：贴在仿真场地 0~4 号地面点上，作为识别目标。**
`scripts/randomize_ground_textures.sh` 每次从每类各抽 1 张、随机分配到 5 个点位，所以**每类恰好出现一次** —— 目的是不让你把「几号点是哪类」背下来。

**用途二：旋转靶叶片的贴图**（靶位 1~5 ↔ 上述 5 类）。

**用途三：识别模型的数据集候选原图。**

### ⚠️ 关键：原图不能直接当训练数据

相机装在车前 0.08 m、**高仅 0.128 m、光轴水平无俯角**，地面图在画面里是被**斜视压缩到 20%~30% 的一条斜带**，还叠加 `rotate 90`。

用原图训练 = 训练了一个错误的输入分布。**必须在仿真里按同样几何抓 `/camera/image` 采帧**，每类 ≥50 张。

`make_explain_image.py` 就是用来直观展示这个差异的（会生成 `说明图-相机看到的地面图片.png`）。

### 换贴图

```bash
source scripts/setup_env.sh
./scripts/randomize_ground_textures.sh                # 随机换一轮
./scripts/randomize_ground_textures.sh --seed 1234    # 固定结果，可复现
./scripts/randomize_ground_textures.sh --dry-run      # 只看映射不修改
```

首次使用需要先配共享写权限（脚本要写 `/opt/ican_shoot_sim`）：

```bash
./scripts/randomize_ground_textures.sh --setup-permissions --add-user "$USER"
```

**替换后必须重启 Gazebo。** 任何测试报告都要写清 `commit + seed`。

---

## 环境要求

- Ubuntu 22.04 LTS amd64（**WSL2 可用**）
- ROS 2 Humble
- Gazebo Classic
- Python 3.10

---

## 快速开始（WSL / Ubuntu）

### 1. clone 到 Linux 文件系统

```bash
cd ~
git clone https://github.com/xxjj111hu/ican-rescue-robot.git
```

> ⚠️ **不要放在 `/mnt/c/` 下构建。** 跨文件系统的 IO 很慢，`inotify` 文件监控容易失效，
> `colcon build` 会又慢又莫名报错。放在 `~/` 这种 Linux 自己的文件系统里。

### 2. 安装 ROS 2 与仿真环境（首次）

```bash
cd ~/ican-rescue-robot/demo_shoot_ws/demo_shoot_ws
bash scripts/00_install_ros2_gazebo.sh
bash scripts/01_install_sim_deb.sh
```

### 3. 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/ican-rescue-robot/demo_shoot_ws/demo_shoot_ws
colcon build --symlink-install
source install/setup.bash
```

成功标志：`Summary: 3 packages finished`

### 4. 检查环境

```bash
bash scripts/check_sim_env.sh
```

16 项全部 `[OK]` 即可。

### 5. 启动仿真

```bash
bash scripts/run_sim.sh
# 无图形界面：
bash scripts/run_sim.sh gui:=false
```

### 6. 跑示例任务

```bash
bash scripts/run_example_mission.sh
```

会开三个终端标签，分别启动仿真、Nav2 导航、示例程序。

---

## 常用接口

| 名称 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制小车（麦轮支持横向 `linear.y`） |
| `/odom` | `nav_msgs/msg/Odometry` | 里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 二维雷达，0.10~12.0 m |
| `/camera/image` | `sensor_msgs/msg/Image` | 前向相机 800×800 @30 Hz |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| `/shooter/shoot` | `std_srvs/srv/Trigger` | 发射一发子弹 |
| `/referee/current_target` | `std_msgs/msg/String` | 射击前通知当前点位（`shoot_1`…`shoot_4`） |
| `/referee/detections` | `std_msgs/msg/String` | 识别话术上报 |

完整说明见 [docs/仿真环境与接口说明.md](demo_shoot_ws/demo_shoot_ws/docs/仿真环境与接口说明.md)

---

## 判分要点（务必牢记）

1. **识别本身不计分**，但它是「到达点位有效」的前置 —— **识别失败 = 该点 0 分**
2. **话术必须逐字正确**，错一个字该点就 0 分
3. **顺序不能颠倒**：先识别发话术 → 再进点位。反过来车到了也算未到达
4. 射击只要该点到达有效就能正常计分，**不受识别成败影响**

---

## 协作约定

### 提交信息格式

```
[任务ID] 做了什么
```

例如 `[B-03] 识别节点接入 /camera/image`。

好处：`git log --grep "B-03"` 能一次查出该任务的全部改动，文档、验收、答辩引用的是同一批证据。
**一次提交只对应一个任务 ID。**

### 日常流程

```bash
git pull                    # 开工前先同步
# …… 改代码 ……
git add -A
git commit -m "[任务ID] 说明"
git push                    # 收工
```

**不要手动拷贝文件进仓库。** `git push` 一次性同步全部内容；手动拷贝会漏文件，导致「参数是新的、别的模块是旧的」，效果对不上。

### 分支建议

不要都直接推 `main`：

```bash
git checkout -b feat/nav2-params    # 开自己的分支
# …… 改 ……
git push -u origin feat/nav2-params # 推分支
# 然后在 GitHub 上发 Pull Request，至少 1 人 review 后合入 main
```

### 里程碑 tag

- `v0.1-arch` —— A-03 接口冻结
- `v0.9-integration` —— E-03 通过
- `v1.0-final` —— F-04 冻结，用 `git archive` 打包提交

**tag 是「这条测试结论对应哪一版代码」的唯一答案。**

---

## 常见坑

| 坑 | 说明 |
|---|---|
| 用原图训练识别模型 | 相机视角是斜视压缩的，必须仿真采帧，见上文 |
| 到点后再识别 | 停在点位上时地面图在车正下方，**画面里看不到**；必须在接近段（距点 0.3~0.8 m）抓帧 |
| 话术发错 | 话题名少一个 `s`（`/referee/detection`）；QoS 不匹配；先到点后识别 |
| 压缩贴图时改名 | 脚本按文件名硬引用，**压缩可以，改名不行** |
| 在 `/mnt/c/` 下构建 | 慢且容易出错，必须放 Linux 文件系统 |
| 忘了 `source` | `colcon: command not found` → 先 `source /opt/ros/humble/setup.bash` |
| 换贴图后不重启 Gazebo | 不生效 |

---

## 同步到 WSL 网络的备注

若 WSL 内 `git clone/push` 报 `GnuTLS recv error (-110): The TLS connection was non-properly terminated`，
说明 WSL 连不上 GitHub。常见原因是 Windows 上的代理软件只监听 `127.0.0.1`，而 WSL2 有独立网络栈访问不到。

解决办法：在代理软件里打开「**允许局域网连接 / Allow LAN**」，然后在 WSL 中：

```bash
hostip=$(ip route show default | awk '{print $3}')
git config --global http.proxy  "http://$hostip:7890"
git config --global https.proxy "http://$hostip:7890"
```

---

## 未纳入版本库的大文件

以下文件因体积过大被 `.gitignore` 排除（GitHub 单文件上限 100 MB）：

- `陆地清障.mp4`（约 100 MB）—— 演示视频，需另行分发
- `demo_shoot_ws.zip`（约 36 MB）—— 与 `demo_shoot_ws/` 内容重复

换行符策略见 [.gitattributes](.gitattributes)：仓库内统一存 LF，
仅 `.cmd`/`.bat` 在 Windows 检出时转 CRLF，以免 Ubuntu 上的 `.sh` 因 CRLF 报错。
