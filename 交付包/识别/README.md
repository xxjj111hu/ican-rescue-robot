# 识别模块交付包

> 负责人：龙影　最后更新：2026-10-20
> 说明：本目录是**文档 / 模型 / 脚本 / 数据集说明**的交付物。
> ROS 2 节点源码已迁入工作空间 **`sim_ws/src/ican_recognize/`**，以便 `colcon build` 直接编译
> （原位置 `节点/ican_recognize/` 已移除，避免两份代码漂移）。

## 一、任务 → 交付物对照（编号已对齐《团队分工表》）

| 编号 | 任务 | 交付物 | 位置 | 状态 |
|---|---|---|---|---|
| **B-01** | 地面图片数据集整理与标注 | **1062 张**仿真抓帧 + YOLO 标签 | `数据集说明.md`、`数据/`、`脚本/auto_collect.py`、`脚本/collect_and_prelabel.sh`、`脚本/check_dataset.py` | ✅ 每类 ≥100 个框（要求 ≥50）；已按**贴图变体**补齐 4 个稀有变体 |
| **B-02** | 地面图片分类模型训练与调优 | 权重 + 训练曲线 + 混淆矩阵 | `模型/best.pt`（ican_v10）、`模型/results.csv`、`模型/*.png` | ✅ 检测 **mAP50 0.881**（v6 为 0.739）；验收实测见 B-05 |
| **B-03** | 识别节点接入仿真并实时推理 | 可 colcon 构建的节点 | `sim_ws/src/ican_recognize/` | ✅ 13.9 Hz（空载）；**已补距离门控** |
| **B-04** | 识别话术与裁判上报对接 | 话术表 + 上报节点 | `话术逐字对照表.md` + `scripts_table.py` | ✅ 5/5 逐字一致；已加强自检 |
| **B-05** | 识别鲁棒性测试与失败兜底 | 测试报告 + 兜底策略 | `鲁棒性测试报告.md`、`兜底策略说明.md` | ✅ **4 个随机 seed，151/160 = 94.4%**（95.0/92.5/97.5/92.5），**4/4 全部 ≥90%**；原始记录见 `报告/acc_seed*_v10_4pt.txt` |

## 二、目录内容

```
识别/
  README.md                 ← 本文件
  数据集说明.md              ← 采集协议、规模、类别分布、覆盖性
  话术逐字对照表.md          ← 我方实现 vs 细则表 5（已修正 PDF 交错文本导致的误判）
  鲁棒性测试报告.md          ← 6 轮实测数据与结论
  兜底策略说明.md            ← 多层兜底机制与未做部分
  数据/
    images/train  images/val   ← 1062 张 800x800 JPG
    labels/train  labels/val   ← 1062 个 YOLO txt
    data.yaml                  ← 训练配置（path 为相对路径）
    README.md
  模型/
    best.pt                 ← ican_v10，YOLO11n，640 输入，mAP50 0.881
    混淆矩阵.png / 混淆矩阵_归一化.png
    训练曲线.png / results.csv / 验证集预测示例.jpg
  脚本/
    check_dataset.py        ← 数据集自检（交付前必须先跑，全 OK 再推）
    infer.py                ← 独立推理（不依赖 ROS）
    acceptance_run.py       ← 验收口径自动测试（5 点位 ×10 次）
    auto_collect.py         ← 自动采集（换贴图 seed → 开车 → 抓帧）
    goto_pose.py            ← /odom 闭环定点驾驶（0.7 m 停车位，误差 3~4 cm）
    build_yolo_from_labels.py / combine_all.py  ← 标注 → 数据集打包
    check_dataset.py        ← 【新增】数据集验收自检（每类框数、类别顺序、标注合法性）
  报告/
    acc_*.txt               ← 6 轮验收原始输出
    eval_*.txt              ← 单帧/距离扫描原始输出

## 交付前必做

```bash
python3 交付包/识别/脚本/check_dataset.py --root 交付包/识别/数据
# 看到「结果: OK 全部通过」再提交
```
```

## 三、如何使用

### 0. 安装依赖（只需一次）

```bash
# ROS 侧：nav_msgs / cv_bridge / python3-opencv / python3-numpy（一般已随 ROS 装好）
# pip 侧：
pip3 install -r sim_ws/src/ican_recognize/requirements.txt
# 等价于：pip3 install "numpy<2" ultralytics
```

> ⚠️ **必须锁 `numpy<2`**：ROS 2 Humble 的 `cv_bridge` 按 numpy 1.x ABI 编译，
> 升到 2.x 会让它在 `imgmsg_to_cv2` 时崩溃，且报错不直观。
> ⚠️ `ultralytics` 会连带装 torch + torchvision（CPU 版约 300~500 MB）。

**装完后必须把 setuptools 退回系统版本**（`ultralytics` 会把它升到 8x，连带破坏 `colcon build`）：

```bash
pip3 uninstall -y setuptools
python3 -c "import setuptools; print('setuptools', setuptools.__version__)"   # 期望 59.6.0
```

> 不退回会这样报错：
> - `colcon-core requires setuptools<80` （依赖冲突）
> - `TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'`
>   （setuptools 79.x 需要 packaging>=22，而 Ubuntu 22.04 是 21.3）
>
> 系统自带 59.6.0 与 packaging 21.3 配套，且满足 colcon 的 `>=30.3.0,<80`。

验证：

```bash
python3 -c "import cv_bridge, ultralytics, numpy, setuptools; \
            print('numpy', numpy.__version__, '| setuptools', setuptools.__version__)"
```

### 1. 准备模型（只需一次）

```bash
mkdir -p ~/.ros/ican_recognize
cp 交付包/识别/模型/best.pt ~/.ros/ican_recognize/best.pt
```

> 节点默认读 `~/.ros/ican_recognize/best.pt`，不再是写死的个人路径。
> 也可用 `ICAN_YOLO_MODEL` 环境变量或 `model_path` 参数覆盖。

### 2. 只看模型效果（不需要 ROS）

```bash
cd 交付包/识别/脚本
python3 infer.py --source 某张图.jpg --save 输出/
python3 infer.py --source 某目录/
```

### 3. 在仿真里跑识别节点

节点现在就在工作空间里，直接编译：

```bash
cd ~/ican-repo/sim_ws            # 或你自己的 sim_ws 路径
colcon build --packages-select ican_recognize
source install/setup.bash
ros2 launch ican_recognize recognize.launch.py
```

参数文件：`sim_ws/src/ican_recognize/config/recognize_params.yaml`

节点输出五个话题：

| 话题 | 内容 |
|---|---|
| `/referee/detections` | 话术（RELIABLE），5 条与细则表 5 逐字一致 |
| `/detection/result` | 逐帧 JSON（class/conf/infer_ms/dist/point/gated/**rotate_target**/**moving_target**），用于验证 ≥10 Hz |
| `/ican_recognize/status` | **【新增】** 门控状态 JSON：`idle / in_window / published / timeout`，供 mission 判断 |
| `/ican_recognize/target` | **【新增】** **射击靶位 JSON**（判分细则表 7）：`class / rotate_target / moving_target / conf / point / dist / source / script`。射击节点订阅它即可知道该瞄哪个靶；兜底发布时 `source` 为 `forced:vote` 或 `forced:best_frame`，便于复盘区分"真识别对"和"蒙对了" |
| `/ican_recognize/debug_image` | 画框图（带距离标注） |

**靶位映射（表 7）**：

| 类别 | 旋转靶 | 移动靶 |
|---|---|---|
| 坍塌废墟 `collapsed_ruins` | 1 号 | 6 号 |
| 滑坡塌方 `landslide` | 2 号 | 6 号 |
| 被困人员 `trapped_people` | 3 号 | 7 号 |
| 受损车辆 `damaged_vehicle` | 4 号 | 7 号 |
| 救援营地 `rescue_camp` | 5 号 | 8 号 |

映射表在 `scripts_table.py` 里带冻结基准，`validate_targets()` 自检；节点启动时会打印
`[靶位自检] 表 7 映射通过…`，改错立即报错。

### 4. 数据集自检（B-01 验收）

```bash
python3 交付包/识别/脚本/check_dataset.py --root dataset_all
```

检查项：5 类齐全、`data.yaml` 的 names 与 `CLASS_ORDER` 一致、每类 ≥50 框、
标签格式合法（5 字段、归一化 0~1、class id 在范围内）、图片与标签一一对应。

### 5. 跑验收口径测试

```bash
ros2 launch ican_recognize recognize.launch.py
python3 交付包/识别/脚本/acceptance_run.py --dist 0.8 --per-point 10
```

## 四、实测指标（如实）

| 指标 | 数值 | 验收线 | 结论 |
|---|---|---|---|
| **验收口径（ican_v10，4 任务点 ×10 次，4 个随机贴图 seed）** | **95.0 / 92.5 / 97.5 / 92.5%，合计 151/160 = 94.4%** | ≥90% | ✅ **4/4 全部达标** |
| 验收口径（ican_v9，同口径 4 个 seed） | 92.5 / 92.5 / 92.5 / 92.5%，合计 148/160 = 92.5% | ≥90% | ✅ 4/4 达标 |
| 验收口径（ican_v6，早期不同口径对照） | 均值 81.5% | ≥90% | ✗ |
| 节点输出频率 | **27~31 Hz**（关闭调试图） | ≥10 Hz | ✅ |
| `infer_ms` | 15.5 ms | — | 模型本身可跑 ~64 Hz |
| 话术逐字一致 | 5/5 | 逐字一致 | ✅ |
| 检测 mAP50 | **0.881（ican_v10，1062 张）** / 0.880（v9，612 张） | — | 任务书"离线准确率≥95%"为**分类**口径，本项目按"4 任务点 ×10 次在线验收"评估；mAP 对补弱变体的收益不敏感，详见 `鲁棒性测试报告.md` §5 |

> **统计口径说明**：早期曾按「5 点位 ×10 次」统计（含 0 号点），但《判分细则 V1.0》§1.1
> 明确任务点为 1→2→3→4 共 4 个，且随机化脚本日志注明「0 号点仅用于视觉贴图，不参与导航」。
> 上表已统一为 **4 任务点**口径（分母 40），并以 `报告/acc_seed*_v10_4pt.txt` 原始记录为准。
> 早期 5 点位口径的数字与本表不完全可比。
| **门控端到端**（1 号点，停车 0.70 m） | 识别 `landslide` 正确、**0.71 m 处发布**、置信度 0.62~0.66、投票 7/7 | — | ✅ 详见 `鲁棒性测试报告.md` §6 |
| 混淆矩阵 | 同类不同变体（如救援营地白天/夜景）仍有互串 | 无明显类间混淆 | ⚠️ 补数据后显著改善，未完全消除 |

## 五、本轮补全的内容（B-01~B-05）

| 项 | 补了什么 | 解决什么风险 |
|---|---|---|
| B-01 | `脚本/check_dataset.py` | 数据集验收从"文档描述"变成"可执行核对" |
| B-02 | ~~`class_min_conf` 每类独立阈值（被困人员/受损车辆抬到 0.60）~~ **已撤销** | 按《判分细则》§1.3「发错与不发同为 0 分」，抬高阈值无计分收益，反而减少猜对机会。改为超时强制发布最优猜测 |
| B-03 | **距离门控**：订阅 `/odom`，只在 **0.60~0.90 m** 窗口接受投票（实测标定）；离开窗口清空投票与锁存 | 避免在 1.4 m 处（准确率仅 40%）就发出错误话术 |
| B-03 | 模型路径改为 `~/.ros/ican_recognize/`，移除 `/home/longying01/...` 硬编码 | 队友（含 WSL 用户）能直接跑 |
| B-03 | 节点迁入 `sim_ws/src/`，新增 `config/recognize_params.yaml` | `colcon build` 编得到；参数可调 |
| B-04 | `validate_scripts()` 增加**冻结基准逐字比对** + 半角标点检查 | 任何人误改话术立即自检失败 |
| B-04 | 修正 `话术逐字对照表.md`（原"细则原文"列是 PDF 交错文本，误判为"5/5 有差异"） | 防止有人去"修"本来正确的字符串 |
| B-05 | `/ican_recognize/status` 状态话题 + **`detect_timeout_sec` 超时兜底** | mission 能感知"识别失败"，继续下一点，全程不中断 |
| B-05 | **超时强制发布最优猜测**（`force_publish_on_timeout`） | 按细则「发错与不发同为 0 分」，赌一次能拿 10 分并保住该点射击分 |
| B-03 | **调试图默认关闭 + `debug_every` 降频** | 实测开调试图每帧多花 127 ms，把 25 Hz 拖到 7 Hz；关闭后回到 24~27 Hz，满足 ≥10 Hz |

## 六、仍需主流程（mission）配合

| 要求 | 说明 |
|---|---|
| 停在 **0.7~0.8 m** 处（不能停在贴图上） | 见 `脚本/goto_pose.py` 的 `STANDOFF_POSES`；**当前 `sim_ws/src/ican_nav_tasks/config/waypoints.yaml` 用的是贴图坐标，与本节冲突，联调前必须统一** |
| 到点后等 **2~3 秒**再走 | 给多帧投票收敛时间 |
| 订阅 `/ican_recognize/status` 处理 `timeout` | 收不到话术时记录"识别失败"并继续下一点 |

## 七、复现训练

```bash
python3 脚本/build_yolo_from_labels.py --src label_ws3 --out dataset_yolo_c --drop-empty
python3 脚本/combine_all.py
yolo detect train data=dataset_all/data.yaml model=模型/best.pt \
     imgsz=640 epochs=50 batch=8 device=cpu name=ican_v7
```

> 下一轮建议：针对「被困人员 ↔ 受损车辆」补采 200~300 张刻意混淆的帧（同类光照/同距离/相近背景），
> 并对这两类做 hard-negative mining，再评估混淆矩阵。
