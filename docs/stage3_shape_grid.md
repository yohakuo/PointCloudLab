# 阶段 3：均匀占据栅格与可靠外轮廓

## 用途与输入

阶段 3 从阶段 2 的四份规范点云估计多组 INSPIRE → FAST 粗变换。默认 `--shape-method occupancy_grid`；`--shape-method sampled_points` 保留原随机投影点搜索，供同一输入下对比。两条路线共用白板平面拟合、四个 90° 竞争方向、宽松几何门限和 SE(3) 去重。

本文记录阶段一的顺序搜索结果，复现时须指定 `--search-method sequential`。阶段二已将占据栅格的默认搜索改为 `joint`，详见 [联合搜索规划与结果](stage3_joint_search.md)。

`configs/registration.yaml` 的 `coarse_registration.geometry.shape_grid` 集中管理栅格参数。改动参数后，应重跑阶段 3 并重新查看候选，不沿用旧候选 ID 或人工验收结论。

## 形状表达与评分

1. 将对象点投影到各自拟合的白板平面，按 `grid_size_m=0.003` 建格。每个**有点格**只作为一个空间位置参与匹配；保存格中心、原始点数、离白板有符号距离的 25%／50%／75% 分位数和可信度。无点格是未知，不计作“对象不存在”的证据。
2. 可信度由饱和的格内点数和邻域已观测支持决定。稀疏点云还会按格内高度降低浅层外围的权重；ROI 坐标边界附近的格按 `crop_confidence_factor` 降权。当前数据的 ROI 边界降权格数为 0，该机制仍保留给发生裁剪的输入。
3. 稀疏点云的外轮廓从高起伏格形成的最大连通核心估计，再扩展一格。轮廓构造时的闭运算和填孔只估计外侧包络，**不会将空洞填为有点格**。因此标记点造成的内部孔洞不会变成匹配目标；低高度外围仍保留在占据格和高度记录中，但不会直接定义可靠外轮廓。
4. 角度和平移搜索使用双向截尾距离：占据格权重 0.35、可靠外轮廓权重 0.55、近邻已观测格的高度差权重 0.10。高度差上限为 20 mm，避免 FAST 深度混合主导分数。新路线的粗排序沿用该栅格评分并加上白板诊断项；点级 Chamfer 仅作宽松门限和独立诊断。两路线的原始粗分数**不能直接比较**。

高起伏分位数、稀疏判定阈值、邻域支持、ROI 边界距离、截尾比例及各项权重都可在 `shape_grid` 配置块中查到。新候选的 `provenance.shape_method` 标明路线；旧路线保持原候选 ID 和矩阵。

## 运行与产物

当前 `configs/dataset.json` 的默认输出目录 `outputs/stage_03_candidates` 已有旧正式产物，且数据清单中的选择 ID 指向它。对当前数据做对比时使用独立目录：

```powershell
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset.json --shape-method sampled_points --output-dir .\outputs\stage_03_sampled_baseline
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset.json --shape-method occupancy_grid --search-method sequential --output-dir .\outputs\stage_03_occupancy_grid
.\.venv\Scripts\python.exe .\scripts\compare_stage3_shapes.py
```

两个目录分别包含 `candidate_report.json`、`canonical_candidates.json`、`candidate_summary.csv`、候选矩阵及 `candidate_comparison.png`。栅格目录另含：

| 文件 | 内容 |
| --- | --- |
| `shape_grids.json` | 每个已观测格的中心、点数、高度分位数、可信度，以及外轮廓；不列出未知格 |
| `shape_grid_preview.png` | 两侧观测格的高度、透明度所示可信度和可靠外轮廓 |
| `baseline_comparison.md/json` | 相同输入哈希校验、候选数量、角度、同一栅格指标复评和最近旧候选的变换差 |

比较其他目录时，可向 `compare_stage3_shapes.py` 传入 `--baseline` 和 `--grid`。它会检查两次运行使用相同的阶段 2 输入哈希；文件写入 `--grid` 指定目录。

## 当前数据的对比

| 指标 | 旧随机投影点 | 新占据栅格 |
| --- | ---: | ---: |
| 原始／规范候选 | 32／16 | 32／8 |
| 连续轮廓角修正 | 9° | 0° |
| 排名 1 的平面内方向 | 279° | 270° |
| 排名 1 用同一栅格指标复评 | 3.86 mm | 3.35 mm |

source 有 322 个观测格；target 有 905 个观测格，其中 421 个高起伏连通支持格用于估计可靠外轮廓。四个 90° 方向均保留，得分接近。新分数改善只说明该形状指标下的拟合变化，不证明最终朝向或精配准质量。完整候选对应关系见 `outputs/stage_03_occupancy_grid/baseline_comparison.md`。

## 后续流程边界

当前阶段 4 的输入校验硬性要求 **恰好 16 个**规范候选及 16 份矩阵；本次新路线有 8 个，因此不能直接作为阶段 4 输入。阶段 4 的默认路径也仍指向 `outputs/stage_03_candidates`。在决定采用新路线继续之前，需要先调整并验证阶段 4 的固定数量假设，再对新 `candidate_comparison.png` 完成人工复核。当前数据清单的 `approvals.stage3_reviewed=true` 和 `selection` 中的候选 ID 属于旧正式产物，不能视为对新候选的验收或选择。
