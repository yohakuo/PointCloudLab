# PointCloudLab

实验版本保存和逐次记录方法见 [实验记录说明](docs/experiment_records.md)。

模块化的 INSPIRE 2 → FAST-LIVO2 点云配准项目。当前有效流程连续编号为阶段 1～5：检查、分割、几何粗配准、精配准、导出。

当前阶段 3 默认使用均匀占据栅格、可靠外轮廓与平面内角度／平移联合搜索。粗配准优化计划中的“阶段一（形状表达）”“阶段二（联合搜索）”均已完成，二者都属于流程阶段 3，与流程阶段 1 的检查、阶段 2 的分割不同。

## 更换点云数据：统一入口

各阶段现在共用一份数据清单。当前正式数据见 `configs/dataset.json`；新数据请复制 `configs/dataset.example.json`，通常只需修改：

- `dataset_id`：本批数据的唯一名称；
- `inputs`：source/target 完整云、ROI，以及手工 target 白板/对象六个入口；
- `output_root`：务必给新数据使用独立目录，例如 `outputs/scan_20260916`，防止混用旧阶段产物；
- `approvals.stage3_reviewed`：检查本批次阶段 3 候选图后改为 `true`，否则阶段 4 会拒绝运行；
- `selection`：阶段 4 人工复核后再填写父候选和精配准候选 ID，之前保持 `null`。

所有相对路径均以项目根目录为基准。先做只读检查：

```powershell
.\.venv\Scripts\python.exe .\scripts\00_validate_dataset.py --dataset .\configs\dataset_new.json
```

之后各阶段传入同一份清单；默认产物和前置报告路径由 `output_root` 推导。先运行到粗配准：

```powershell
.\.venv\Scripts\python.exe .\scripts\01_inspect_data.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\02_segment_regions.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset_new.json
```

检查候选后，再按阶段 4/5 的前置条件运行后续步骤。**当前阶段 4 仍要求恰好 16 个规范候选，默认联合搜索不保证生成 16 个；当前数据仅生成 4 个，不能直接连跑精配准与导出。** 使用新候选前须调整数量校验并重新人工复核。

```powershell
.\.venv\Scripts\python.exe .\scripts\04_refine_candidates.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\05_export_results.py --dataset .\configs\dataset_new.json
```

阶段 2 的预览确认、阶段 3/4 的人工验收和阶段 5 的显式候选选择仍按下文执行。清单提供默认值，原有 `--source-full`、`--target-full`、`--source-board` 等参数仍可临时覆盖。阶段 5 核验当前清单、前置报告及 `selection` 的一致性。

## 坐标与单位约定

- source：INSPIRE 2 `.ply`，原始单位 mm；读取后以 `(0,0,0)` 为缩放中心执行 `XYZ_m = 0.001 * XYZ_mm`。
- target：FAST-LIVO2 `.pcd`，单位 m。
- 固定方向：`INSPIRE_TO_FAST`，矩阵定义为 `p_FAST,m = T_FAST<-INSPIRE(m) p_INSPIRE,m`。
- 阶段 2 的 ROI 只负责选点，不重新居中、不归一化、不修改坐标；白板孔洞保持原样。

配置位于 `configs/registration.yaml`。该文件采用 JSON 语法（也是合法 YAML），因此无需新增 YAML 解析依赖。

## 阶段 1 运行顺序

先列出候选文件：

```powershell
.\.venv\Scripts\python.exe .\scripts\01_inspect_data.py --list-candidates
```

数据目录存在多个候选时，必须显式指定文件：

```powershell
.\.venv\Scripts\python.exe .\scripts\01_inspect_data.py `
  --source .\data\raw\20260913\20260913_195812_pc.ply `
  --target .\data\raw\20260913\09132052.pcd `
  --source-roi .\data\processed\20260913_195812_pc_roi.ply `
  --target-roi .\data\processed\09132052roi.pcd
```

若没有预先保存的 ROI，可改用米制 AABB；它只选点，不平移或重新居中坐标：

```powershell
.\.venv\Scripts\python.exe .\scripts\01_inspect_data.py `
  --source-full <INSPIRE完整PLY> --target-full <FAST完整PCD> `
  --source-roi-bounds-m XMIN XMAX YMIN YMAX ZMIN ZMAX `
  --target-roi-bounds-m XMIN XMAX YMIN YMAX ZMIN ZMAX
```

输出位于 `outputs/stage_01_inspection/inspection_report.json` 和 `stage_01_inspection.log`。报告分别保留原生单位统计和米制工作坐标统计；清洗只移除非有限坐标和（规模允许时）完全重复的 XYZ。

运行测试：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## 阶段 2：区域分割（target 手工文件优先）

正式运行（四个基准输入和两份 target 手工文件均显式给出；两个手工参数缺一即报错，CLI 路径优先于配置）：

```powershell
.\.venv\Scripts\python.exe .\scripts\02_segment_regions.py `
  --source-full .\data\raw\20260913\20260913_195812_pc.ply `
  --target-full .\data\raw\20260913\09132052.pcd `
  --source-roi .\data\processed\20260913_195812_pc_roi.ply `
  --target-roi .\data\processed\09132052roi.pcd `
  --target-board-file .\data\processed\target_board_manual.pcd `
  --target-object-file .\data\processed\target_object_manual.pcd
```

配置示例已把 `target_selection_mode` 设为 `manual_files`，并提供 `target_manual_files.board_file/object_file`。手工模式直接读取并规范化保存两份手工云；不会调用 target 自动选侧、距离阈值提取或用 DBSCAN 替换选区。`target_roi` 只作坐标/边界参考，`target_full` 只作显式坐标相容性参考，二者均不会静默替换手工输入。若确需复现旧自动流程，必须显式使用 `--target-selection-mode automatic` 且不得同时传手工文件。

source 行为不变：`source_full` 以原点乘 `0.001` 后拟合白板，`source_roi` 使用该平面提取中央共有对象，因此完整 source 中的右侧定位磁铁不会进入 `source_object_points`。target 手工文件按 FAST 米制原坐标读取，不缩放、不平移、不重新居中、不归一化，也不执行配准。

报告记录每份手工文件的绝对路径、格式、字段、大小、SHA-256、原始/有效点数、NaN/Inf、重复点、质心、XYZ 范围和包围盒，并检查：路径/内容相同、精确重叠、FAST 坐标相容性、白板稳健平面拟合及覆盖、对象点数/连通分量/投影范围/ROI 边界接触，以及对象到白板平面的有符号距离分位数。距离只作几何诊断，不能解释为物理厚度；重叠或异常不会触发自动删点或重选。

输出位于 `outputs/stage_02_segmentation`：四类规范点云、`segmentation_preview.png`、`segmentation_report.json` 和日志。预览固定为平面局部相机方向，包含 source/target 白板与对象投影、两份 target 原始手工云、白板拟合覆盖以及 target 有符号距离侧视图，图中明确颜色、图例和米制单位。报告始终写明 `registration_performed=false` 和 `transform_estimated=false`。

状态约定：不可读/空云、同一路径、相同内容、坐标异常或白板无法可靠拟合为 `failed`；明显重叠或几何可疑为 `needs_manual_confirmation`；所有自动检查通过仍保持 `needs_manual_confirmation`，并要求先检查预览。只有用户检查本次预览后，才可用同一命令追加以下参数：

```powershell
  --confirm-target-manual-preview
```

该参数不能覆盖硬错误、重叠或几何警告。退出码 `0` 表示完成可审查运行（包括 `needs_manual_confirmation`），`2` 表示硬失败。

人工检查重点：source 白板是否确实来自 `source_full`；source 对象是否仅为中央共有对象且不含定位磁铁；target 白板是否覆盖宽广、允许孔洞但几何纯净；target 对象轮廓/正面是否完整；两份 target 云是否错分、重叠或混入背景；对象是否接触 ROI 边界；是否保留足以区分平面内方向的不对称特征。


## 阶段 3：几何多候选粗配准

阶段 3 直接核验阶段 2 报告和四份规范点云，然后运行白板/对象投影几何路线。不会搜索法向/FPFH 参数，也不会计算 FPFH 或执行特征全局配准。默认形状表达为 3 mm 均匀占据栅格：每个有点格参与一次匹配，并保留离白板的高度四分位数、点数和可信度。稀疏目标的外轮廓由高起伏连通核心估计，外围低高度观测仍保留但降权；孔洞中的无点格仍是未知。方法、参数、产物字段及当前数据的完整对比见 [阶段 3 栅格说明](docs/stage3_shape_grid.md)。

### 默认运行与搜索方式

新数据集通过独立清单运行，产物保存到该清单的 `output_root/stage_03_candidates`。当前正式数据的默认目录 `outputs/stage_03_candidates` 已有旧产物；对当前数据做实验请使用后面的独立输出目录命令。

```powershell
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset_new.json
```

占据栅格默认使用联合搜索：完整 360° 粗扫，每个角度寻找最佳平移；0°/90°/180°/270° 四个方向分支各保留多个候选种子，再进行两层角度×平移局部细化。PCA 只提供坐标系和初值，不预先决定角度。去重及数量截断保护各法向下的四个方向分支；相近种子仍可合并，不以重复候选凑数。

| 形状表达 | 搜索方式 | 用途 |
| --- | --- | --- |
| `occupancy_grid` | `joint`（默认） | 栅格与可靠外轮廓上的角度／平移联合搜索 |
| `occupancy_grid` | `sequential` | 优化阶段一基线：先定角度，再搜索平移 |
| `sampled_points` | `sequential`（该表达的默认值） | 原随机投影点基线 |

`sampled_points + joint` 不支持。候选 ID 使用 `s3_geo_*` 和 `s3_can_*`。算法细节见 [联合搜索规划与实现](docs/stage3_joint_search.md)。

### 联合搜索参数

配置位置为 `configs/registration.yaml` 的 `coarse_registration.geometry.joint_search`；形状表达与评分参数仍位于同级 `shape_grid`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `angle_start_deg` / `angle_step_deg` | 0° / 5° | 全周粗扫起点与角度步长 |
| `translation_window_m` / `translation_step_m` | 0.012 / 0.003 m | 每个角度围绕稳健中心平移的两轴 ±范围与步长 |
| `seeds_per_branch` | 2 | 每个方向分支最多保留的不同种子 |
| `nms_angle_deg` / `nms_translation_m` | 6° / 0.004 m | 联合角度和平移的种子抑制阈值 |
| `refinement_levels[0]` | 角度 ±5°、步长 1°；平移 ±3 mm、步长 1.5 mm | 第一层细化 |
| `refinement_levels[1]` | 角度 ±1°、步长 0.25°；平移 ±1.5 mm、步长 0.5 mm | 第二层细化 |

可通过 `--joint-angle-step-deg`、`--joint-seeds-per-branch` 临时覆盖角度步长及种子数。`--geometry-search-window-m` 和 `--geometry-search-step-m` 在 joint 模式下覆盖联合搜索的平移范围及步长；其余参数通过配置文件调整。

### 对比顺序搜索与联合搜索

下面的命令使用当前正式数据清单，在三个独立目录运行顺序基线、默认联合搜索和较大步长联合搜索。启动 Python 前固定线程数，以避免本机多线程白板拟合差异改变栅格；设置只作用于当前 PowerShell 会话。

```powershell
$env:OMP_NUM_THREADS = '1'
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset.json --shape-method occupancy_grid --search-method sequential --output-dir .\outputs\stage_03_grid_sequential
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset.json --shape-method occupancy_grid --search-method joint --output-dir .\outputs\stage_03_grid_joint
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset.json --shape-method occupancy_grid --search-method joint --joint-angle-step-deg 10 --geometry-search-step-m 0.006 --output-dir .\outputs\stage_03_grid_joint_fast
.\.venv\Scripts\python.exe .\scripts\compare_stage3_search.py --joint .\outputs\stage_03_grid_joint .\outputs\stage_03_grid_joint_fast
```

比较脚本核验输入哈希、栅格、高度与可信度、平面坐标系、评分参数及线程设置一致，否则拒绝比较。输出位于 `outputs/stage_03_search_comparison/search_comparison.md/json/png`；其他数据可显式传入 `--baseline`、`--joint` 和 `--output-dir`。

2026-09-24 当前数据的实测结果如下，两种联合配置都完整保留四个方向：

| 指标 | 顺序基线 | 联合默认（5° / 3 mm） | 联合较大步长（10° / 6 mm） |
| --- | ---: | ---: | ---: |
| 原始 / 规范候选 | 32 / 7 | 8 / 4 | 8 / 4 |
| 全周粗扫角度×平移组合 | 不适用 | 5,832 | 900 |
| 含细化的评分次数 | 未计数 | 11,560 | 6,628 |
| 首位栅格误差 | 3.214 mm | 3.214 mm | 3.214 mm |
| 首位点级 Chamfer | 4.837 mm | 4.837 mm | 4.837 mm |
| 搜索耗时 | 1.29 s | 16.54 s | 9.41 s |
| 整次耗时 | 8.11 s | 22.22 s | 15.08 s |

联合方法扩大了搜索覆盖，但本数据未出现明显误差改善。每分支两个种子在细化、去重后合并为一个规范候选。较大步长的搜索耗时降低约 43%，首位误差相同；耗时为同机单次观测，不能直接推广到其他数据。三个配置重复运行的候选 ID、顺序和矩阵均逐值一致；完整测试集 83 项通过。

当前没有真实位姿标签，匹配误差不能证明最终方向唯一。阶段一历史运行采用不同拟合结果，本次已重新运行顺序基线，历史数值不应与该表直接相减。完整结果见 [执行对比报告](outputs/stage_03_search_comparison/search_comparison.md) 和 [覆盖图](outputs/stage_03_search_comparison/search_comparison.png)；这些本地产物位于 Git 忽略的 `outputs/`，新检出项目需先运行上述命令生成。

### 产物与审查

每个运行目录包含 `candidate_report.json`、`raw_candidates.json`、`canonical_candidates.json`、`candidate_summary.csv`、`candidate_matrices/` 和候选对比图。占据栅格路线另含 `shape_grids.json`（只记录已观测格）及 `shape_grid_preview.png`。

联合搜索报告的 `geometry_route.audit` 记录全周逐角最佳平移 `angle_profile`、种子与细化轨迹 `seeds`、评分次数、搜索耗时及平移边界命中。检查 `candidate_comparison.png` 后再决定如何继续；粗排名不自动选定最终变换。

### 复现优化阶段一：随机投影点与栅格对比

对比旧随机投影点路线时，显式使用两个独立输出目录（不会覆盖正式阶段 3 产物）：

```powershell
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --shape-method sampled_points --output-dir .\outputs\stage_03_sampled_baseline
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --shape-method occupancy_grid --search-method sequential --output-dir .\outputs\stage_03_occupancy_grid
.\.venv\Scripts\python.exe .\scripts\compare_stage3_shapes.py
```

新目录另含 `shape_grids.json`（仅列出已观测格，高度和可信度；无点格为未知）、`shape_grid_preview.png` 与 `baseline_comparison.md/json`。阶段一历史运行的旧／新规范候选为 16／8，轮廓角修正为 9°／0°；两路线的粗分数定义不同，需用对比报告中的同一栅格度量复评。栅格候选仍需检查 `candidate_comparison.png`，不能由粗排名确定最终方向。

## 粗配准扰动稳定性实验

实验入口为 [study_coarse_perturbations.py](scripts/study_coarse_perturbations.py)，配置为 [coarse_stability.json](configs/coarse_stability.json)，方案和判读见 [实验说明](docs/coarse_stability.md)。它使用阶段 2 原始观测支持，按配对清单扰动后重新进行白板拟合、坐标系构建、栅格与轮廓提取、模型拟合、搜索、细化、评分和去重。包含无扰动参考，以及空间采样、分割边界、白板重采样和联合扰动各两档。原随机投影点、栅格顺序、栅格联合、联合候选后模型重评分、模型参与搜索五种方法使用相同扰动输入。模型拟合或候选支持不足时按配置回退栅格评分，记录原因。

启动前固定线程；先试运行，再执行四次重复。输出写入独立目录，不覆盖正式候选：

```powershell
$env:OMP_NUM_THREADS = '1'
$env:OPENBLAS_NUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'
.\.venv\Scripts\python.exe .\scripts\study_coarse_perturbations.py --pilot --output-dir .\outputs\coarse_stability_pilot
.\.venv\Scripts\python.exe .\scripts\study_coarse_perturbations.py --output-dir .\outputs\coarse_stability
```

运行中断后可用 `--resume` 接续同一配置。`outputs/coarse_stability/report.md`、`summary.json`、逐次 `runs/`、配对 `paired_inputs/` 和三张统计图均由脚本生成。报告按实际三维旋转匹配方向，并将同一扫描仪参考点投影到固定雷达白板平面；位置波动是重复性，不是绝对精度。无真实位姿标签，方向竞争的获胜频率不是方向正确概率。方向仍需人工复核；精配准当前要求 16 个规范候选，而联合搜索候选数不足，须独立处理衔接。

本次正式实验 165/165 次有效，模型拟合和评分回退均为零。栅格联合路线在 32 次扰动中有 29 次近似并列；八个组别/强度单元的最差分支位置偏移 p90 为 4.49–9.05 mm，均超过预设的 3 mm 操作阈值。模型后重评分保持 33/33 个配对候选池不变，模型参与搜索则 33/33 个候选池均发生变化，两者均未消除方向竞争。只有 19/40 个统计单元达到两批趋稳容差，结果仍受每单元仅 4 次重复的限制；不能宣称绝对精度提升。详见 [执行结论](docs/coarse_stability.md#本次执行结果2026-09-24)。

## 阶段 4：候选精配准

阶段三的固定联合搜索候选池模型重评分见 [实施与对照](docs/stage3_model_prior.md)。模型只拟合扫描仪侧已观测正面，保留四方向且首位未变；独立结果位于 `outputs/stage_03_model_prior_rescore`，不作为阶段 4 输入。

阶段 4 核验阶段 2、阶段 3 报告以及规范候选 JSON/矩阵的一致性。检查 `candidate_comparison.png` 后，将数据清单中的 `approvals.stage3_reviewed` 设为 `true` 才能运行。

当前阶段 4 的输入校验硬性要求 16 个规范候选和 16 份矩阵，不能直接读取本次顺序基线的 7 个候选或联合搜索的 4 个候选。其默认输入路径仍指向 `outputs/stage_03_candidates`；使用独立实验输出目录不会自动切换阶段 4 输入。当前数据清单已有的 `stage3_reviewed=true` 和选择 ID 只对应旧正式产物。使用新候选前须调整数量校验、显式衔接新的阶段 3 产物，并重新人工复核；详见 [联合搜索说明](docs/stage3_joint_search.md#验证与限制)。

```powershell
.\.venv\Scripts\python.exe .\scripts\04_refine_candidates.py
```

每个父候选依次尝试多层 GICP 和 Huber robust point-to-plane，并始终使用 board→board、object→object 对应。输出位于 `outputs/stage_04_refinement`；精配准候选 ID 使用 `s4_ref_*`。

## 阶段 5：输出与验证

人工复核阶段 4 后，在数据清单的 `selection.parent_candidate_id` 与 `selection.refined_candidate_id` 中填写新生成的 ID。未填写时程序会失败，不会自动选择候选。

```powershell
.\.venv\Scripts\python.exe .\scripts\05_export_results.py
```

输出位于 `outputs/stage_05_export`，包含米制刚体矩阵、原始毫米坐标映射矩阵、完整注册云、合并云、验证报告和诊断图。自动评分、排名与方向唯一性裁决仍未执行，导出成功不代表变换方向唯一。

默认只允许导出状态为 `refined_valid` 的候选。如果人工复核后仍决定采用其他状态的候选，可显式覆盖安全门：
现在配置文件修改要用的参数再运行下面的命令
```powershell
.\.venv\Scripts\python.exe .\scripts\05_export_results.py `
  --allow-nonvalid-selection
```

如果命令行父候选与数据清单不同，还必须添加 `--confirm-nondefault-parent`。覆盖操作不会把候选改写为 `refined_valid`；最终报告会保留阶段 4 原状态，并记录 `selection_method=explicit_user_override` 与 `safety_gate_overridden=true`。对于 `no_safe_refinement`，阶段 4 的 `matrix_m` 通常是安全回退的父矩阵，而不是被接受的精配准更新。
