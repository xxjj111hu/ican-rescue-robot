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
    issues = validate_scripts(strict=True)
    if issues:
        print("自检失败：")
        for i in issues:
            print("  -", i)
        raise SystemExit(1)
    print("5 条话术自检通过：均含裁判匹配关键词、与冻结基准逐字一致、标点规范。")
