"""识别话术常量表（唯一来源）。

来源：《应急救援机器人挑战赛裁判系统判分细则 V1.0》第 4 章 表 5
     （陆地障碍物清除任务点识别）
纪律：代码里禁止拼接、禁止改写；改一个字都要重新和细则逐字比对。
裁判匹配关键词见 KEYWORDS，发布前必须自检话术里包含该关键词。

注意：判分细则 PDF 是多栏排版，直接复制文本会把版面标签
（"陆地障碍物清除任务""点识别"）混进句子中间。下面 _FROZEN_SCRIPTS
是人工按语义还原后的基准串，validate_scripts() 会强制比对；
若有人改动了 CLASS_SCRIPT，自检立即失败，逼回细则重新核对。
"""

# 类别名 -> 识别输出话术（逐字抄自细则表 5，勿手改）
CLASS_SCRIPT = {
    "trapped_people": "已识别到被困人员，救援通道存在障碍，开始清除障碍，实施被困人员救援。",
    "rescue_camp": "已识别到救援营地，前方存在障碍，开始清除障碍，建立救援通道。",
    "collapsed_ruins": "已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。",
    "damaged_vehicle": "已识别到受损车辆，车辆阻碍救援通道，开始清除障碍，实施被困人员救援。",
    "landslide": "已识别到滑坡塌方，落石阻断救援道路，开始清除障碍，打通救援通道。",
}

# 冻结基准：与 CLASS_SCRIPT 逐字一致，任何改动都会让自检失败
_FROZEN_SCRIPTS = {
    "trapped_people": "已识别到被困人员，救援通道存在障碍，开始清除障碍，实施被困人员救援。",
    "rescue_camp": "已识别到救援营地，前方存在障碍，开始清除障碍，建立救援通道。",
    "collapsed_ruins": "已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。",
    "damaged_vehicle": "已识别到受损车辆，车辆阻碍救援通道，开始清除障碍，实施被困人员救援。",
    "landslide": "已识别到滑坡塌方，落石阻断救援道路，开始清除障碍，打通救援通道。",
}

# 类别名 -> 裁判匹配关键词（细则表 5 最后一列）
KEYWORDS = {
    "trapped_people": "已识别到被困人员",
    "rescue_camp": "已识别到救援营地",
    "collapsed_ruins": "已识别到坍塌废墟",
    "damaged_vehicle": "已识别到受损车辆",
    "landslide": "已识别到滑坡塌方",
}

# 类别顺序（必须与 ican_dataset.yaml 的 names 一致；labelImg 的 classes.txt 也必须是这个顺序）
CLASS_ORDER = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]


# ===== 地面图片 -> 射击靶位 ID =====
# 来源：《判分细则 V1.0》第 5 章 表 7「地面图片与靶位 ID 对应关系」
#   旋转靶 1 号 -> 坍塌废墟      移动靶 6 号 -> 坍塌废墟、滑坡塌方
#   旋转靶 2 号 -> 滑坡塌方      移动靶 7 号 -> 被困人员、受损车辆
#   旋转靶 3 号 -> 被困人员      移动靶 8 号 -> 救援营地
#   旋转靶 4 号 -> 受损车辆
#   旋转靶 5 号 -> 救援营地
#
# 用途：识别出地面图片类别后，据此决定该打哪个靶位。
# 注意「首击锁定」规则（细则 1.3(3)）：命中第一发后该点位不再计分，
# 若第一发打错靶（如本该打旋转靶 2 却打了 6），这一发就把该点位成绩用掉了。
ROTATE_TARGET_ID = {
    "collapsed_ruins": 1,
    "landslide": 2,
    "trapped_people": 3,
    "damaged_vehicle": 4,
    "rescue_camp": 5,
}

MOVING_TARGET_ID = {
    "collapsed_ruins": 6,
    "landslide": 6,
    "trapped_people": 7,
    "damaged_vehicle": 7,
    "rescue_camp": 8,
}

# 冻结基准：表 7 的 8 个靶位，任何改动都会让自检失败
_FROZEN_ROTATE = {
    "collapsed_ruins": 1,
    "landslide": 2,
    "trapped_people": 3,
    "damaged_vehicle": 4,
    "rescue_camp": 5,
}
_FROZEN_MOVING = {
    "collapsed_ruins": 6,
    "landslide": 6,
    "trapped_people": 7,
    "damaged_vehicle": 7,
    "rescue_camp": 8,
}


def target_ids(cls_name):
    """返回 {'rotate': 旋转靶号, 'moving': 移动靶号}；未知类别返回 None。"""
    return {
        "rotate": ROTATE_TARGET_ID.get(cls_name),
        "moving": MOVING_TARGET_ID.get(cls_name),
    }


def validate_targets():
    """表 7 靶位映射自检：5 类全覆盖、与冻结基准一致、靶位号在 1~8 内。"""
    problems = []
    for cls in CLASS_ORDER:
        if cls not in ROTATE_TARGET_ID:
            problems.append(f"{cls}: 表 7 缺少旋转靶映射")
        if cls not in MOVING_TARGET_ID:
            problems.append(f"{cls}: 表 7 缺少移动靶映射")
    for name, table, frozen in (
        ("ROTATE_TARGET_ID", ROTATE_TARGET_ID, _FROZEN_ROTATE),
        ("MOVING_TARGET_ID", MOVING_TARGET_ID, _FROZEN_MOVING),
    ):
        for cls, tid in table.items():
            if cls not in CLASS_ORDER:
                problems.append(f"{name}: 类别 {cls} 不在 CLASS_ORDER 里")
            if not isinstance(tid, int) or not (1 <= tid <= 8):
                problems.append(f"{name}: {cls} 的靶位号 {tid} 超出 1~8")
            expect = frozen.get(cls)
            if expect is not None and tid != expect:
                problems.append(
                    f"{name}: {cls} 靶位号 {tid} 与冻结基准 {expect} 不一致\n"
                    f"      → 请回《判分细则 V1.0》表 7 重新核对后再改"
                )
    # 旋转靶 1~5 必须一一对应，不能重复
    rotate_ids = list(ROTATE_TARGET_ID.values())
    if sorted(rotate_ids) != [1, 2, 3, 4, 5]:
        problems.append(f"旋转靶号不是 1~5 各一次：{sorted(rotate_ids)}")
    return problems


def validate_scripts(strict=False):
    """返回问题列表；空列表表示 5 条话术自检通过。

    strict=True 时额外要求与 _FROZEN_SCRIPTS 逐字一致（默认也检查，
    因为错一个字 = 该点 10 分归零，这个风险不值得给"灵活性"留口子）。
    """
    problems = []
    for cls, keyword in KEYWORDS.items():
        script = CLASS_SCRIPT.get(cls)
        if not script:
            problems.append(f"{cls}: 缺少话术")
            continue
        if keyword not in script:
            problems.append(f"{cls}: 话术缺少裁判匹配关键词「{keyword}」")
        if "\n" in script or script.strip() != script:
            problems.append(f"{cls}: 话术含换行或首尾空白")
        if not script.endswith("。"):
            problems.append(f"{cls}: 话术未以句号结尾")
        # 全角标点检查：混入半角逗号/句号会导致话术不一致
        if ", " in script or "," in script:
            problems.append(f"{cls}: 含半角逗号，应为全角「，」")
        frozen = _FROZEN_SCRIPTS.get(cls)
        if frozen and script != frozen:
            problems.append(
                f"{cls}: 与冻结基准不一致\n"
                f"      当前: {script}\n"
                f"      基准: {frozen}\n"
                f"      → 请回《判分细则 V1.0》表 5 重新逐字核对后再改"
            )
    for cls in CLASS_SCRIPT:
        if cls not in CLASS_ORDER:
            problems.append(f"{cls}: 不在 CLASS_ORDER 里，类别顺序会错")
    for cls in (_FROZEN_SCRIPTS if strict else ()):
        if cls not in CLASS_SCRIPT:
            problems.append(f"{cls}: 冻结基准里有但 CLASS_SCRIPT 缺失")
    return problems


if __name__ == "__main__":
    issues = validate_scripts(strict=True) + validate_targets()
    if issues:
        print("自检失败：")
        for i in issues:
            print("  -", i)
        raise SystemExit(1)
    print("5 条话术自检通过：均含裁判匹配关键词、与冻结基准逐字一致、标点规范。")
    print("表 7 靶位映射自检通过：")
    for cls in CLASS_ORDER:
        t = target_ids(cls)
        print(f"  {cls:<16} 旋转靶 {t['rotate']} 号   移动靶 {t['moving']} 号")
