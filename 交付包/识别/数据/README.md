# 地面图片识别数据集（5 类 / 912 张）

> 对应任务：地面图片数据集整理与标注
> 采集：仿真 `/camera/image`（800×800），`scripts/auto_collect.py` + `scripts/collect_and_prelabel.sh`
> 标注：全部人工在 labelImg 校正（YOLO 检测框），无伪标签

## 规模

| 划分 | 图片 | 标签 | 框数 |
|---|---|---|---|
| train | 634 | 634 | 820 |
| val | 278 | 278 | 330 |
| **合计** | **912** | **912** | **1150** |

每类框数均 ≥100，满足「每类不少于 50 张」。

## 目录

```
数据/
  images/train/*.jpg      images/val/*.jpg       # 800x800
  labels/train/*.txt      labels/val/*.txt       # YOLO: class cx cy w h（归一化）
  data.yaml                                      # path 为相对路径，可直接训练
```

## 类别顺序（与 `data.yaml`、话术表一致）

| id | 英文名 | 中文 |
|---|---|---|
| 0 | trapped_people | 被困人员 |
| 1 | rescue_camp | 救援营地 |
| 2 | collapsed_ruins | 坍塌废墟 |
| 3 | damaged_vehicle | 受损车辆 |
| 4 | landslide | 滑坡塌方 |

## 自检

```bash
python3 交付包/识别/脚本/check_dataset.py --root 交付包/识别/数据
# 期望输出：结果: OK 全部通过
```

## 采集覆盖（可复现）

| 维度 | 取值 |
|---|---|
| 随机贴图 seed | 101/102/104/105/106/107/108/109/110/111/112/113/200/203/204 等 |
| 点位 | 0~4 号（全覆盖） |
| 相机到贴图距离 | 0.7 / 0.8 / 0.9 / 1.1 / 1.4 m |
| 车头偏角 | -20° / 0° / +20° |
| 划分方式 | 按 seed 划分（验证集来自训练时未见过的 seed） |

## 关键经验：按「贴图变体」平衡，而不是按类别

每类有 2 张候选图，共 10 张。早期按类别统计看似平衡（每类 120+ 框），
但按变体统计发现 4 个变体只有 20~30 框，实测 0~40% 全错：

| 变体 | 补前 | 补后 |
|---|---|---|
| rescue_camp_01（夜景白帐篷） | 20 | 50 |
| collapsed_ruins_02 | 20 | 50 |
| trapped_people_02 | 28 | 58 |
| landslide_01 | 30 | 60 |

补完后验收均值从 81.5%（v6）提升到 **91.0%（v9）**。

## 训练

```bash
yolo detect train data=交付包/识别/数据/data.yaml \
     model=交付包/识别/模型/best.pt imgsz=640 epochs=35 batch=16 device=cpu name=ican_v9
```
