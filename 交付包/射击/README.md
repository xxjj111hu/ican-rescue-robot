# 射击模块（D-01 ~ D-05）

> 负责人：胡文静　优先级 P0
> ROS 包：`sim_ws/src/ican_shoot/`

---

## 一、当前状态

| 任务 | 内容 | 状态 |
|---|---|---|
| **D-01** | 环形靶瞄准算法 | ✅ **视觉已验证**（靶心误差 2.4 px），闭环与开火已实现 |
| **D-02** | 环形靶命中环数刷分 | ⏳ 待实弹标定（`dry_run:=false` 后实测环数） |
| **D-03** | 旋转靶 1~5 号指定靶位 | 🔬 方案已定（Gazebo 可读靶位姿态），待实现 |
| **D-04** | 移动靶 6/7/8 号命中 | 🔬 待实现 |
| **D-05** | 射击稳定性与多靶联调 | ⏳ 等 D-02~D-04 |

---

## 二、关键实测数据（写算法前先看这些，别凭感觉设阈值）

### 2.1 环形靶（①号点，1.54 m 距离，800×800 图）

**标准十环靶纸**，从外到内：`黑边 → 红(6) → 白(7) → 蓝(8) → 黑(9) → 黄心(10)`

实测颜色（**仿真光照偏暗，V 只有 90~130，不能按标准靶纸的亮色设阈值**）：

| 环 | HSV 实测 |
|---|---|
| 黄心 (10) | H 21~31, S ≥112, V ≥93（仅约 26 px，直径约 6 px） |
| 红 (6) | H ≤5, S ≥164, V ≥107 |
| 蓝 (8) | H 101~105, S ≥141, V ≥114 |
| 白 (7) | S <45, V ≥96 |

**像素尺寸与环数换算**：

```
靶直径 ≈ 82 px（半径 41 px），覆盖 6~10 环共 5 个环带
每环 ≈ 8.2 px
```

| 目标 | 允许横向偏差 |
|---|---|
| ≥8 环（蓝环内） | ≤ **24.6 px** |
| ≥9 环 | ≤ **16.4 px** |
| 10 环（黄心） | ≤ **8.2 px** |

> ⚠️ 官方示例 `ring_aim_node.py` 的容差是 ±25 px，**恰好卡在 8 环边界**，
> 想要 9~10 环必须收到 ±8~10 px。本模块默认 `tol_px = 8`。

### 2.2 靶位坐标（`land_shoot.world` 权威值）

| 靶 | 位置 | 尺寸/说明 |
|---|---|---|
| ① `target_fixed` | (1.575, -1.35, 0.26) | 0.27×0.27 m 十环靶纸 |
| ② `target_wheel` | (1.575, -2.70, 0.26) | 5 个叶片各 0.05 m，转速 0.24166 rad/s |
| ③ `target_moving_3` | (1.125, -2.97, 0.26) | 板 0.18×0.07 m，沿 x ±0.15 m、0.03 m/s（周期 20 s） |
| ④ `target_moving_4` | (1.125, 0.27, 0.26) | 同上，反相位 |

### 2.3 射击接口

```
服务 /shooter/shoot  (std_srvs/srv/Trigger)   ← 另有 /shooter/fire 等价
话题 /shooter/shot   (std_msgs/String)        ← 调用服务后由仿真自动发布，程序不用自己发
弹道：从 shout_link 前方 0.25 m、高 0.26 m、30 m/s 射出，Gazebo 物理判命中
```

---

## 三、已完成：D-01 视觉（离线验证过）

`ring_detect.py` 用 **HSV 三色质心法**：

- 为什么不用轮廓圆拟合：红环被白环数字打断成碎弧，圆形度只有 0.15，会被过滤掉
- **对称环带的质心就是圆心** —— 用红/蓝/黄三色的质心加权融合，红环 99 分位距离作靶半径
- **ROI 只取上 55%** —— 下方彩色地面贴图会让红/黄掩码的 r99 涨到 300~440 px（实测 0.60 时完全失效）
- 黄心阈值 S ≥120 —— 靶板是米色(S 60~90)，阈值放低会把整块靶板并进来，质心偏 30 px

**离线验证结果**（`python 交付包/射击/脚本/test_ring_detect.py`）：

```
point1_ring.png  ✅ 靶心(408.6, 356.1) 半径41.5  dx=+8.6  conf=0.85  估环9.0
真值(410,358) r=41 → 靶心误差仅 2.4 px，半径误差 +0.5 px
```

2.4 px ≈ 0.3 环，足够打 9~10 环。

---

## 四、怎么测 D-01 / D-02

### 4.1 准备

```bash
cd ~/ican-repo
git fetch win && git merge win/main
cd sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ican_shoot --symlink-install
source install/setup.bash
```

### 4.2 起环境（仿真 + Nav2，不起任务节点）

```bash
bash ~/ican-repo/sim_ws/start_all.sh --no-recog --no-mission
```

### 4.3 开到①号点（车头正对环形靶）

```bash
python3 ~/ican-repo/交付包/识别/脚本/goto_pose.py --x 0.031 --y -1.360 --yaw 0.0
```

### 4.4 先只瞄准不开火（安全验证）

```bash
ros2 launch ican_shoot ring_shoot.launch.py dry_run:=true
```

另开终端触发：

```bash
ros2 service call /ican_shoot/fire_ring std_srvs/srv/Trigger
```

**期望**：服务在几秒内返回
```
success=True
message='dry_run：瞄准完成 dx=+1.2px，估算 9.9 环，耗时 3.4s，未实弹'
```

同时可看瞄准过程：

```bash
ros2 topic echo /ican_shoot/status
ros2 topic echo /ring_target/center_offset
```

### 4.5 确认瞄准稳定后，实弹

```bash
ros2 launch ican_shoot ring_shoot.launch.py dry_run:=false
ros2 service call /ican_shoot/fire_ring std_srvs/srv/Trigger
```

观察 `/shooter/shot` 是否收到射击消息。**`/shooter/shot` 只说明打出去了，
不代表打中哪儿** —— 环数要看 Gazebo 里靶纸的弹孔或裁判系统反馈。

**D-02 的标定方法**：连续打 20 发，记录每发 `dx` 与落点环数，
拟合"dx → 环数"的实际关系，据此修正 `tol_px` 或加固定偏置。

---

## 五、待解决：D-03 / D-04 的靶位识别

### 5.1 已排除的方案：标准 ArUco 解码

旋转靶是 5 个 AR 码排成一圈，但每个码在图上只有 **约 14×14 px**，
且带透视畸变。用 OpenCV 全部 16 个内置字典 × 4 种放大倍数尝试，**均无法可靠解码**
（唯一的 `DICT_4X4_1000 id=988` 是误检）。

移动靶的 3 个码更小（板 0.18 m 上排 3 个码，1.06 m 距离）。

### 5.2 已确认的有利条件：Gazebo 能直接读靶位姿态

```bash
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: 'target_wheel'}"
→ pose (1.575, -2.7, 0.26)
  orientation (x=0.8104, y=0, z=0, w=0.5859)   ← 绕 x 轴 108.3°
  success=True
```

**旋转靶的相位可以直接读**，不必解 AR 码。据此可做：

1. 高频采样 `orientation` → 得到转角 θ(t) 与转速
2. 叶片 5 个等间隔 72°，编号 1~5 按角度顺序（**待确认**）
3. 目标叶片到达射线位置的时间可预测 → **在正确时刻开火**，无需转车头

**待确认**：叶片编号与角度顺序的对应关系。探针命令：

```bash
ros2 service call /gazebo/get_model_list std_srvs/srv/Empty
```

若叶片是独立实体（如 `target_wheel_1`..`target_wheel_5`），编号即得出。

### 5.3 兜底方案

若无法确定编号：按"叶片每到中心打一发"，接受 1/5 的命中指定靶位的概率。
**不推荐** —— 首击锁定会把该点成绩钉死。

---

## 六、已知坑（都踩过）

| 坑 | 后果 | 规避 |
|---|---|---|
| 用官方示例的 ±25 px 容差 | 只能打 8 环 | 收到 ±8 px |
| 靶心数字"10"干扰取样 | 颜色采样得到偏暗假值 | 用区域聚类而非单点采样 |
| ROI 取到地面贴图 | 红/黄掩码爆成 300+ px 巨块，检测全废 | ROI 取上 55% |
| 黄心阈值 S 太低 | 米色靶板被并入，质心偏 30 px | S ≥120 |
| `cv2.imread` 读中文路径 | 静默返回 None | 用 `np.fromfile` + `imdecode` |
| Trigger 服务"先返回后更新" | **无效**，response 已发出 | 阻塞式服务 + 多线程执行器 |
| `/clock` 停滞时 ROS 定时器不触发 | 节点静默不动作 | 用墙钟 + `time.sleep` |

---

## 七、③号任务点的车头朝向（已修）

```
③ 点=(1.120,-1.914)  移动靶3=(1.125,-2.970)
  原 waypoints 与任务说明书都写 yaw=+90°  → 车头朝 +y，对着的是"靶4"
  正确应为             yaw=-90°          → 车头朝 -y，对着靶3
```

③号点位于两个移动靶正中间（到靶3、靶4 都是 1.055 m），朝向错 180° 会打到
**④号点的靶**，按细则「首击锁定」该点 10 分被错误锁定。

**已在 `sim_ws/src/ican_nav_tasks/config/waypoints.yaml` 中修正。**
