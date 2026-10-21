# 射击模块（D-01 ~ D-05）

> 负责人：胡文静　优先级 P0
> ROS 包：`sim_ws/src/ican_shoot/`

---

## 一、当前状态

| 任务 | 内容 | 状态 | 实测数据 |
|---|---|---|---|
| **D-01** | 环形靶瞄准算法 | ✅ **完成** | 离线靶心误差 1.8 px；实机 dx=+6.9 px |
| **D-02** | 环形靶命中环数刷分 | ⚠️ 瞄准达标，环数需裁判系统确认 | 估算 9.2 环，见 §五 |
| **D-03** | 旋转靶 1~5 号指定靶位 | ✅ **验收通过** | 叶片竖直偏移 −22mm、横向 −7mm |
| **D-04** | 移动靶 6/7/8 号命中 | ✅ **验收通过** | 横向 −8mm/−6mm、朝向 −0.4°/−0.3° |
| **D-05** | 射击稳定性与多靶联调 | ✅ **验收通过（4/4）** | 见 [D-05联调验收报告](D-05联调验收报告.md) |

---

## 二、环形靶（①号点）

### 2.1 靶纸结构与实测颜色

从外到内：`黑边 → 红(6) → 白(7) → 蓝(8) → 黑(9) → 黄心(10)`

**仿真光照偏暗，V 仅 90~130，不能按标准靶纸的亮色设阈值**：

| 环 | 实测 HSV |
|---|---|
| 黄心 (10) | H 21~31, S ≥112, V ≥93（约 26 px，直径约 6 px） |
| 红 (6) | H ≤5, S ≥164, V ≥107 |
| 蓝 (8) | H 101~105, S ≥141, V ≥114 |
| 白 (7) | S <45, V ≥96 |

### 2.2 像素尺寸与环数换算

```
靶直径 ≈ 82 px（半径 41 px），覆盖 6~10 环共 5 个环带 → 每环 ≈ 8.2 px
```

| 目标 | 允许横向偏差 |
|---|---|
| ≥8 环（蓝环内） | ≤ 24.6 px |
| ≥9 环 | ≤ 16.4 px |
| 10 环（黄心） | ≤ 8.2 px |

> ⚠️ 官方示例容差 ±25 px **恰好卡在 8 环边界**，要 9~10 环必须收到 ±8~10 px。
> 本模块默认 `tol_px = 8`。

### 2.3 离线验证结果

```
point1_ring.png  ✅ 靶心(408.6, 356.1) 半径41.5  dx=+8.6  conf=0.85  估环9.0
真值(410,358) r=41 → 靶心误差 2.4 px，半径误差 +0.5 px
```

2.4 px ≈ 0.3 环。

---

## 三、旋转靶（②号点，D-03）

### 3.1 叶片结构（来自 land_shoot.world，已数值核对）

一个 link、5 个叶片、等间隔 72°、半径 0.09 m、尺寸 0.01×0.05×0.05：

| 叶片 | 材质 | 靶位 | 本地角 φ |
|---|---|---|---|
| leaf_0 | WheelLeaf1Texture | **1** | 90°（正上） |
| leaf_1 | WheelLeaf2Texture | **2** | 18° |
| leaf_2 | WheelLeaf3Texture | **3** | −54° |
| leaf_3 | WheelLeaf4Texture | **4** | −126° |
| leaf_4 | WheelLeaf5Texture | **5** | 162° |

**`leaf_k` 就是靶位 `k+1`，`φ_k = 90° − 72°k`**（脚本核对误差 < 1e-5）。

### 3.2 关键物理约束（不知道就永远打不中）

叶片排在**半径 0.09 m 的圆环上，圆心是空的**；子弹从 0.26 m 高度水平射出、
正好穿过轮心。因此：

> **正对轮心射击，一发都打不中。**

必须两个条件同时满足：
1. 目标叶片转到**左右水平位置**（竖直偏移 |r·sin ψ| ≤ 0.025 m，即 |ψ| ≤ 16.1°）
2. 车头偏 **atan(0.09/1.548) ≈ 3.33°** 对准该叶片所在的侧面

### 3.3 实现时序

```
ψ = φ_k + θ（θ 从 /gazebo/model_states 读四元数绕 x 轴转角）
→ 找最近的窗口中心（0°=右 / 180°=左）
→ 提前把车头转到那一侧（不等窗口）
→ 当 |ψ−中心| ≤ 13° 且横向 ≤ 12mm 时开火
```

**必须提前转身** —— 叶片 13.75°/s，窗口仅持续约 1.9 s，临时转来不及。

### 3.4 实测（dry_run）

```
[过程] 轮角=11.0°  竖直=-61mm  距窗口=-43.0°  窗口=未到  横向=-13mm  对准=否
[过程] 轮角=25.7°  竖直=-43mm  距窗口=-28.3°  窗口=未到  横向=-12mm  对准=否
[过程] 轮角=32.8°  竖直=-33mm  距窗口=-21.2°  窗口=未到  横向=-12mm  对准=是
[过程] 轮角=41.3°  竖直=-20mm  距窗口=-12.7°  窗口=OK    横向=-12mm  对准=是
→ success=True 叶片竖直偏移 -18mm、侧别 右、横向偏差 -12mm
```

实测转速约 13.4~13.75 °/s（理论 0.24166 rad/s = 13.85 °/s）。

---

## 四、移动靶（③④号点，D-04）

### 4.1 结构

```
model target_moving_3/4
  pose 1.125 -2.97 0.26 0 0 1.5708     ← yaw 90°，正对机器人
  collision/visual box 0.02 0.18 0.07
  material MovingBoardTexture           ← 3 个 AR 码画在贴图上，非独立几何
```

**编号顺序无法从 world 文件读出** → 用参数 `moving_code_order`（默认 `"678"` = 左→右）。
板沿 x 往复 ±0.15 m，世界坐标可实时读。

### 4.2 实现

```
板的世界位姿 → 按板的 yaw 把码的本地偏移 (0, code_dy) 旋转到世界
→ 算出该码的世界坐标 → 车头闭环对准 → 横向 ≤ 12mm 时开火
```

### 4.3 实测（dry_run）

```
success=True
target_moving_3 的 7 号已射击（板=(1.089,-2.970) 码=(1.089,-2.970)
距离=1.037m 横向=+7mm 朝向误差=+0.4°）
```

---

## 五、D-02 的环数为什么无法验证

### 5.1 Gazebo 不在贴图上"打孔"

射击后抓图放大 8 倍，靶纸完好无损 —— 命中判定靠物理碰撞接触，
**相机画面里看不到弹孔**。差分法也被光照/位姿差异淹没（配准残差 MSE=144）。

### 5.2 仿真没有计分话题

`ros2 topic list` 全量检查：只有 `/shooter/shot`（子弹名），
**没有任何 score / hit / referee 话题**。

### 5.3 结论

| 项 | 结论 |
|---|---|
| 瞄准精度 | ✅ dx 在容差内（每环 8.2 px，容差 8 px ≈ 1 环） |
| 环数 | ❓ 取决于"相机光轴 = 弹道方向"这一前提，**需裁判系统确认** |

**这是练习环境的能力边界，不是实现缺陷。** 正式比赛由裁判系统给出环数。

---

## 六、踩过的坑（都是实际调试出来的，别再重复）

| 坑 | 后果 | 规避 |
|---|---|---|
| **默认回调组是互斥的** | **服务回调阻塞时，同节点所有订阅回调停摆，瞄准线程读到冻结缓存（实测 389 次采样轮角恒为 94°）** | **订阅放 `ReentrantCallbackGroup`，服务放独立 `MutuallyExclusiveCallbackGroup`** |
| 用官方示例的 ±25 px 容差 | 只能打 8 环 | 收到 ±8 px |
| 靶心数字"10"干扰取样 | 颜色采样得到偏暗假值 | 用区域聚类而非单点采样 |
| ROI 取到地面贴图 | 红/黄掩码爆成 300+ px 巨块，检测全废 | ROI 取上 55% |
| 黄心阈值 S 太低 | 米色靶板被并入，质心偏 30 px | S ≥120 |
| `cv2.imread` 读中文路径 | 静默返回 None | 用 `np.fromfile` + `imdecode` |
| `/gazebo/get_entity_state` 服务 | 程序里调用请求发出但响应永不返回（连续 17 次）；`get_model_list` 直接卡死 | 改用 `/gazebo/model_states` **话题** |
| Trigger 服务"先返回后更新" | 无效，response 已发出 | 阻塞式服务 + 线程 |
| 在 ROS 回调里调服务 | future 永不完成 | 把工作放普通线程 |
| `/clock` 停滞时 ROS 定时器不触发 | 节点静默不动作 | 墙钟 + `time.sleep` |
| `/odom`、`model_states` 用 RELIABLE | 收不到（Gazebo 用 BEST_EFFORT） | `qos_profile_sensor_data` |
| 旋转靶只对轮心瞄准 | **一发都打不中**（圆心是空的） | 等叶片转到左右水平位再打 |
| 竖直条件不满足就不转车头 | 车头永远不转，横向偏差永远修不好 | **提前转身**，不等窗口 |
| `setup.cfg` 带 UTF-8 BOM | colcon 报 `MissingSectionHeaderError` | 写文件不加 BOM |

---

## 七、射击接口的实测时序

```bash
/shooter/shoot  (std_srvs/srv/Trigger)   调用后 5.9 ~ 19.9 s 才返回（不稳定，且无规律）
/shooter/shot   (std_msgs/String)        每发必发，payload 是子弹名 bullet_xxxxxxxx
```

**无弹药限制**，每次调用生成一颗新子弹。

**因此不能阻塞等待确认** —— 只做 2 s 短探测，避免把状态机拖死 20 s。
D-05 联调时 4 个点的射击时间预算要留足（每发可能等数秒）。

---

## 八、怎么测

### 8.1 编译

```bash
cd ~/ican-repo && git fetch win && git merge win/main
cd sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ican_shoot --symlink-install
source install/setup.bash
```

### 8.2 D-01 / D-02（环形靶，①号点）

```bash
python3 ~/ican-repo/交付包/识别/脚本/goto_pose.py --x 0.031 --y -1.360 --yaw 0.0
nohup ros2 launch ican_shoot ring_shoot.launch.py dry_run:=true > /tmp/ring.log 2>&1 &
sleep 6
ros2 service call /ican_shoot/fire_ring std_srvs/srv/Trigger
```

### 8.3 D-03（旋转靶，②号点）

```bash
python3 ~/ican-repo/交付包/识别/脚本/goto_pose.py --x 0.027 --y -2.700 --yaw 0.0
nohup ros2 launch ican_shoot target_shoot.launch.py dry_run:=true > /tmp/target.log 2>&1 &
sleep 6
ros2 service call /ican_shoot/fire_wheel std_srvs/srv/Trigger
grep -E "\[过程\]|窗口命中" /tmp/target.log | tail -10
```

### 8.4 D-04（移动靶，③④号点）

```bash
# ③号点（正对移动靶3，车头 -y）
python3 ~/ican-repo/交付包/识别/脚本/goto_pose.py --x 1.120 --y -1.914 --yaw -1.5708
ros2 service call /ican_shoot/fire_moving_3 std_srvs/srv/Trigger

# ④号点（正对移动靶4，车头 +y）
python3 ~/ican-repo/交付包/识别/脚本/goto_pose.py --x 1.120 --y -0.785 --yaw 1.5708
ros2 service call /ican_shoot/fire_moving_4 std_srvs/srv/Trigger
```

### 8.5 实弹

```bash
# 注意：实弹会触发「首击锁定」，需要重启仿真才能重置
bash ~/ican-repo/sim_ws/stop_all.sh && bash ~/ican-repo/sim_ws/start_all.sh
nohup ros2 launch ican_shoot target_shoot.launch.py dry_run:=false > /tmp/fire.log 2>&1 &
```

### 8.6 诊断工具

```bash
# 观察旋转靶是否在转、验证叶片公式
python3 ~/ican-repo/交付包/射击/脚本/watch_wheel.py --seconds 12

# 抓靶子图（含弹孔分析）
python3 ~/ican-repo/交付包/射击/脚本/grab_image.py --out /tmp/target.png

# 离线验证环形靶检测（不需要仿真）
python 交付包/射击/脚本/test_ring_detect.py
```

---

## 九、③号任务点的车头朝向（已修）

```
③ 点=(1.120,-1.914)  移动靶3=(1.125,-2.970)
  原 waypoints 与任务说明书都写 yaw=+90°  → 车头朝 +y，对着的是"靶4"
  正确应为             yaw=-90°          → 车头朝 -y，对着靶3
```

③号点位于两个移动靶正中间（到靶3、靶4 都是 1.055 m），朝向错 180° 会打到
**④号点的靶**，按细则「首击锁定」该点 10 分被错误锁定。

**已在 `sim_ws/src/ican_nav_tasks/config/waypoints.yaml` 中修正。**
