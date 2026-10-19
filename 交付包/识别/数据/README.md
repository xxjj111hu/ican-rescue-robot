# 地面图片识别数据集（5 类）

> 对应任务：地面图片数据集整理与标注
> 采集：仿真 `/camera/image`（800×800），`scripts/auto_collect.py` 自动换贴图 + 定点抓帧
> 标注：全部人工在 labelImg 校正（YOLO 检测框），无伪标签

## 规模

| 划分 | 图片 | 标签 | 框数 |
|---|---|---|---|
| train | 434 | 434 | 612 |
| val | 178 | 178 | 208 |
| **合计** | **612** | **612** | **820** |

每类框数（训练集）：被困人员 120、救援营地 117、坍塌废墟 119、受损车辆 127、滑坡塌方 129 —— 均 ≥100，满足「每类不少于 50 张」。

## 目录

```
dataset_all/
  images/train/*.jpg      images/val/*.jpg       # 800x800
  labels/train/*.txt      labels/val/*.txt       # YOLO: class cx cy w h（归一化）
  data.yaml                                      # YOLO 训练配置
```

## 类别顺序（与 `data.yaml`、话术表一致）

| id | 英文名 | 中文 |
|---|---|---|
| 0 | trapped_people | 被困人员 |
| 1 | rescue_camp | 救援营地 |
| 2 | collapsed_ruins | 坍塌废墟 |
| 3 | damaged_vehicle | 受损车辆 |
| 4 | landslide | 滑坡塌方 |

## 采集覆盖（可复现）

| 维度 | 取值 |
|---|---|
| 随机贴图分配 | 9 个 seed：101/102/104/105/106/107/108/109/110 |
| 点位 | 0~4 号（全部覆盖） |
| 相机到贴图距离 | 0.8 / 1.1 / 1.4 m |
| 车头偏角 | -20° / 0° / +20° |
| 划分方式 | 按 seed 划分（验证集来自训练时未见过的 seed），避免同分布自测 |

所有贴图材质自带 `rotate 90`，画面天然包含该旋转，无需额外处理。

## 直接训练

```bash
yolo detect train data=dataset_all/data.yaml model=yolo11n.pt \
     imgsz=640 epochs=50 batch=8 device=cpu name=ican_v6
```

## 说明

- 原始采集帧共 2430 张（未全部标注），本目录只含**已标注并用于训练**的 612 张。
- 数据集体积约 88 MB，单张 800×800 JPEG 约 144 KB。
- 识别模型权重与推理脚本见 `交付包/识别/`。
