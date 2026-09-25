# 阶段二计划：平面内角度与平移联合搜索

## 实施与验收

1. 保持阶段一栅格、高度、可信度和评分定义，保留 `sequential` 基线以及随机点路线。
2. 对完整 360° 角度网格逐角搜索平移；PCA 只提供坐标系和额外初值，不决定角度。按最近的 0/90/180/270° 分成四个半开方向区间。
3. 每分支经角度和平移联合非极大抑制保留多个种子，以多层角度×平移网格细化。保留父解，细化不得恶化评分。去重与数量截断保护每个法向下的四个方向分支。
4. 所有范围、步长、种子数和抑制距离集中配置；记录全周逐角最优平移、种子及细化轨迹、评价次数、边界命中和耗时。
5. 测试已知旋转平移、四分支覆盖、角度环绕、确定性、缓存评分一致性和不合法配置。相同阶段二输入、相同形状度量下重跑顺序基线与联合搜索，使用独立目录比较覆盖、误差和耗时。

本项只执行阶段 3 粗配准，不替换已验收的正式输出，不沿用旧候选选择。

## 已实现的搜索流程

默认 `occupancy_grid + joint`，随机点默认仍为 `sampled_points + sequential`；联合搜索不支持随机点表达。

全周粗扫采用 `[angle_start_deg, angle_start_deg + 360)`，并补入 PCA 的 0/90/180/270° 初值。每个角度围绕 `median(target_cells) - median(rotated_source_cells)` 搜索平移，使用阶段一的相同栅格、可靠外轮廓、高度和可信度评分。无点格保持未知。

角度按最近的 0/90/180/270° 分为四个半开区间，例如 0° 分支为 `[315°, 360°) ∪ [0°, 45°)`。每个区间独立挑选多个低分种子：角度差和平移距离同时小于 NMS 门限的种子被抑制。每层细化都枚举角度与二维平移的组合，并保留父解。细化可以跨越 0°，但不能跨入另一方向分支；全局平移窗口也是硬边界。

SE(3) 去重仅在同一法向和方向分支内合并，候选数量截断先为各分支保留一个名额。数量上限不足以容纳所有分支，或几何门限使某个分支全部失效时，运行会明确失败。两个种子若收敛到近似相同的变换，可以合并为一个规范候选；不会为了凑数输出重复变换。

缓存目标侧近邻树减少重复建树，但保留原始正向坐标计算。规则栅格的等距离近邻在浮点变换后可能改变排序，逆变换查询曾使加权截尾分数产生微小差异，因此未采用这一优化。新增规则栅格测试验证加速评分与原评分逐值一致。

## 可配置参数

配置位置：`configs/registration.yaml` → `coarse_registration.geometry.joint_search`。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `angle_start_deg` / `angle_step_deg` | 0° / 5° | 完整一周的起点与粗步长，粗步长须 ≤45° |
| `translation_window_m` / `translation_step_m` | ±0.012 / 0.003 m | 每角度的两轴平移范围与粗步长 |
| `seeds_per_branch` | 2 | 每个方向分支最多保留的不同种子，至少配置 2 |
| `nms_angle_deg` / `nms_translation_m` | 6° / 0.004 m | 种子角度和平移抑制阈值 |
| `refinement_levels[0]` | ±5°、1°；±3 mm、1.5 mm | 第一层角度/平移局部搜索 |
| `refinement_levels[1]` | ±1°、0.25°；±1.5 mm、0.5 mm | 第二层局部搜索 |

窗口网格包含零和两侧端点，不因范围不能被步长整除而越界。CLI 支持 `--joint-angle-step-deg`、`--joint-seeds-per-branch`，原有 `--geometry-search-window-m` 和 `--geometry-search-step-m` 在 joint 模式下覆盖联合搜索的平移参数。其余参数通过配置文件调整。

## 运行与比较

本机 Open3D 多线程白板 RANSAC 在相同随机种子下仍出现不同拟合结果，进而改变栅格。为隔离搜索变量，以下命令在启动 Python 前固定线程数，并由比较脚本严格核验两次输入哈希、栅格全部字段、坐标系、评分参数及线程设置；不一致时拒绝生成结论。此设置只作用于当前 PowerShell 会话，不修改系统配置。

```powershell
$env:OMP_NUM_THREADS = '1'
.\.venv\Scripts\python.exe scripts/03_generate_candidates.py --dataset configs/dataset.json --shape-method occupancy_grid --search-method sequential --output-dir outputs/stage_03_grid_sequential
.\.venv\Scripts\python.exe scripts/03_generate_candidates.py --dataset configs/dataset.json --shape-method occupancy_grid --search-method joint --output-dir outputs/stage_03_grid_joint
.\.venv\Scripts\python.exe scripts/03_generate_candidates.py --dataset configs/dataset.json --shape-method occupancy_grid --search-method joint --joint-angle-step-deg 10 --geometry-search-step-m 0.006 --output-dir outputs/stage_03_grid_joint_fast
.\.venv\Scripts\python.exe scripts/compare_stage3_search.py --joint outputs/stage_03_grid_joint outputs/stage_03_grid_joint_fast
```

三个独立输出目录包含完整候选、矩阵、预览及报告。比较目录 `outputs/stage_03_search_comparison` 包含 `search_comparison.md/json/png`。每次联合运行的 `candidate_report.json` → `geometry_route.audit` 记录：

- `angle_profile`：完整粗扫的每角度最佳平移、分数及平移边界命中。
- `seeds`：各分支种子、逐层细化前后分数和最终结果。
- `coarse_evaluations` / `total_evaluations`：评价次数。
- `coarse_elapsed_s` / `search_elapsed_s`：搜索计时；顶层 `elapsed_s` 包括拟合和绘图。

## 当前数据的执行结果（2026-09-24）

固定 `OMP_NUM_THREADS=1` 后，三种运行使用相同的 322 个 source 格与 907 个 target 格，比较脚本核验所有格的高度、可信度及轮廓一致。阶段一历史运行的 target 为 905 格，不能将该历史运行与本次结果直接相减；本次已重新运行顺序基线以控制拟合差异。

| 项目 | 顺序基线 | 联合默认（5° / 3 mm） | 联合较大步长（10° / 6 mm） |
| --- | ---: | ---: | ---: |
| 完整粗扫角度数 | 先居中估角，再在固定方向搜平移 | 72 | 36 |
| 粗搜索角度×平移组合 | 不适用 | 5,832 | 900 |
| 含细化的评分次数 | 未计数 | 11,560 | 6,628 |
| 原始 / 规范候选 | 32 / 7 | 8 / 4 | 8 / 4 |
| 最终保留方向分支 | 4 | 4 | 4 |
| 首位平面内角度 | 270° | 270° | 270° |
| 首位栅格误差 | 3.214 mm | 3.214 mm | 3.214 mm |
| 首位点级 Chamfer | 4.837 mm | 4.837 mm | 4.837 mm |
| 搜索耗时 | 1.29 s | 16.54 s | 9.41 s |
| 整次耗时 | 8.11 s | 22.22 s | 15.08 s |

联合方法在本数据上扩展了搜索覆盖，没有取得明显误差改善。每分支两个种子经细化后被 SE(3) 阈值合并为一个规范候选，因此候选总数减少，但四个方向均保留。四方向的最优栅格误差约为 3.214–3.230 mm，仍然接近。较大步长配置搜索耗时下降约 43%，首位匹配误差相同；这是一组数据的观测，不代表其他数据也适用该步长。

三种最终配置各进行了重复运行：规范候选 ID、顺序及矩阵全部逐值相同（7/7、4/4、4/4）；候选矩阵、JSON 和 CSV 序列化一致性检查通过。最终候选均未命中平移硬边界。时间采用最后一次同机依次运行值，不是统计基准。

完整的每方向角度、平移和误差见 [执行对比报告](../outputs/stage_03_search_comparison/search_comparison.md)，覆盖图见 [搜索对比图](../outputs/stage_03_search_comparison/search_comparison.png)。

## 验证与限制

合成测试覆盖已知 37° 与 359° 变换、跨零角度、平移恢复、四方向保留、每层评分不恶化、范围端点、配置错误、确定性和 SE(3) 数量截断。评分测试覆盖非均匀可信度、高度项及规则格点等距情形。

完整测试集：`python -m unittest discover -s tests`，83 项通过；`git diff --check` 通过。

当前数据没有真实位姿标签。栅格评分和点级 Chamfer 是匹配诊断，不能据此判断最终朝向唯一。四个方向的区间中心相差 90°，独立优化后候选角度不要求严格相差 90°。全周有限网格与局部细化也不保证连续空间的全局最优。

阶段 4 仍要求恰好 16 个规范候选，本次联合输出不能直接传入；继续精配准前需要调整该接口并复核新候选。本项未改动阶段 4、正式输出或数据清单的旧选择 ID。
