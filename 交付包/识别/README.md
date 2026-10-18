# 识别模块交付包

> 对应任务：地面图片数据集整理与标注 / 分类模型训练与调优 / 识别节点接入仿真 /
> 识别话术与裁判上报对接 / 识别鲁棒性测试与失败兜底
> 生成时间：2026-10-18

## 一、任务 → 交付物对照

| 编号 | 任务 | 交付物 | 位置 | 状态 |
|---|---|---|---|---|
| A-03 | 地面图片数据集整理与标注 | 5 类数据集 + 标签文件 | `数据/`（见 `数据集说明.md`） | ✅ 每类 ≥100 个框，覆盖 0~4 点位 |
| B-01 | 分类模型训练与调优 | 模型权重 + 推理脚本 | `模型/best.pt`、`脚本/infer.py` | ⚠️ 未达离线 95%，实测见下 |
| B-02 | 识别节点接入仿真 | 可 colcon 构建的节点包 | `节点/ican_recognize/` | ✅ `/detection/result` 实测 13.9 Hz |
| B-03 / D-01 | 识别话术与裁判上报 | 向 `/referee/detections` 发话术的节点 | `节点/ican_recognize/.../scripts_table.py` + `话术逐字对照表.md` | ✅ 5/5 逐字一致 |
| B-04 | 鲁棒性测试与失败兜底 | 测试报告 + 兜底策略说明 | `鲁棒性测试报告.md`、`兜底策略说明.md` | ⚠️ 均值 91.5%，单轮最低 84% |

## 二、目录内容

```
识别/
  README.md                 ← 本文件
  数据集说明.md              ← 采集协议、规模、类别分布、覆盖性
  话术逐字对照表.md          ← 我方实现 vs 判分细则表 5
  鲁棒性测试报告.md          ← 6 轮实测数据与结论
  兜底策略说明.md            ← 多层兜底机制与未做部分
  模型/
    best.pt                 ← ican_v6，YOLO11n，640 输入
    混淆矩阵.png / 混淆矩阵_归一化.png
    训练曲线.png / results.csv / 验证集预测示例.jpg
  节点/
    ican_recognize/         ← ROS 2 ament_python 包（可 colcon build）
  脚本/
    infer.py                ← 独立推理（不依赖 ROS）
    acceptance_run.py       ← 验收口径自动测试（5 点位 ×10 次）
    auto_collect.py         ← 自动采集（换贴图 seed → 开车 → 抓帧）
    goto_pose.py            ← /odom 闭环定点驾驶（误差 3~4 cm）
    build_yolo_from_labels.py / combine_all.py  ← 标注 → 数据集打包
  报告/
    acc_*.txt               ← 6 轮验收原始输出
    eval_*.txt              ← 单帧/距离扫描原始输出
```

## 三、如何使用

### 1. 只看模型效果（不需要 ROS）

```bash
cd 识别/脚本
python3 infer.py --source 某张图.jpg --save 输出/
python3 infer.py --source 某目录/
```

### 2. 在仿真里跑识别节点

```bash
# 把 节点/ican_recognize 放进 colcon 工作空间 src/ 下
colcon build --packages-select ican_recognize
source install/setup.bash
ros2 run ican_recognize recognize_node
# 或
ros2 launch ican_recognize recognize.launch.py
```

节点输出：
- `/referee/detections`：话术（RELIABLE），5 条与细则表 5 逐字一致
- `/detection/result`：逐帧 JSON（class / conf / infer_ms），用于验证 ≥10 Hz
- `/ican_recognize/debug_image`：画框图

### 3. 跑验收口径测试

```bash
ros2 run ican_recognize recognize_node --ros-args -p min_publish_interval:=0.4
python3 识别/脚本/acceptance_run.py --dist 0.8 --per-point 10
```

## 四、实测指标（如实）

| 指标 | 数值 | 验收线 | 结论 |
|---|---|---|---|
| 单帧识别（0.8 m，5 点位 ×3 次） | 15/15 = 100% | — | ✅ |
| 验收口径（5 点位 ×10 次，6 轮） | 68%~100%，均值 **91.5%** | ≥90% | ⚠️ 均值达标，单轮最低 84% |
| 节点输出频率 | 13.9 Hz | ≥10 Hz | ✅ |
| 话术逐字一致 | 5/5 | 逐字一致 | ✅ |
| 离线 mAP50（检测口径） | 0.739（ican_v6） | — | 说明：任务书写的"离线准确率≥95%"与检测口径不完全对应，本项目按"0.8 m 单帧 + 5×10 验收"评估 |
| 混淆矩阵 | 被困人员 ↔ 受损车辆 仍互串 | 无明显类间混淆 | ⚠️ 未完全满足 |

## 五、已知限制

1. **类别混淆**：「被困人员」与「受损车辆」外观接近，在 100~200 px 宽的斜视条带上区分度低。
2. **距离敏感**：必须工作在 0.8 m 左右；1.4 m 时准确率降到 40%。
3. **随机贴图**：每场比赛 5 个点位随机分配 5 类贴图，模型只依赖外观，不依赖位置（已用 9 个 seed 验证）。
4. **CPU 训练**：当前模型在 CPU 上训练（50 轮约 80 分钟），有 GPU 可显著提升。

## 六、复现训练

```bash
python3 脚本/build_yolo_from_labels.py --src label_ws3 --out dataset_yolo_c --drop-empty
python3 脚本/combine_all.py
yolo detect train data=dataset_all/data.yaml model=模型/best.pt \
     imgsz=640 epochs=50 batch=8 device=cpu name=ican_v7
```
