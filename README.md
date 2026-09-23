# PointCloudLab

模块化的 INSPIRE 2 → FAST-LIVO2 点云配准项目。当前有效流程连续编号为阶段 1～5：检查、分割、几何粗配准、精配准、导出。FPFH 参数搜索和 FPFH + RANSAC 全局粗配准路线已移除。

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

之后每个阶段只需传同一份清单；阶段产物和前置报告路径会从 `output_root` 自动衔接：

```powershell
.\.venv\Scripts\python.exe .\scripts\01_inspect_data.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\02_segment_regions.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\04_refine_candidates.py --dataset .\configs\dataset_new.json
.\.venv\Scripts\python.exe .\scripts\05_export_results.py --dataset .\configs\dataset_new.json
```

阶段 2 的预览确认、阶段 3/4 的人工验收和阶段 5 的显式候选选择仍按下文执行。清单提供默认值，原有 `--source-full`、`--target-full`、`--source-board` 等参数仍可临时覆盖。阶段 5 核验当前清单、前置报告及 `selection` 的一致性。

## 坐标与单位约定

- source：INSPIRE 2 `.ply`，原始单位 mm；读取后以 `(0,0,0)` 为缩放中心执行 `XYZ_m = 0.001 * XYZ_mm`。
- target：FAST-LIVO2 `.pcd`，单位 m。
- 固定方向：`INSPIRE_TO_FAST`，未来矩阵定义为 `p_FAST,m = T_FAST<-INSPIRE(m) p_INSPIRE,m`。
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

阶段 3 直接核验阶段 2 报告和四份规范点云，然后运行白板/对象投影几何路线。不会搜索法向/FPFH 参数，也不会计算 FPFH 或执行特征全局配准。

```powershell
.\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py
```

几何路线保留 PCA/OBB、轮廓角度、0°/90°/180°/270° 竞争方向和多个平面内平移假设。输出位于 `outputs/stage_03_candidates`；候选 ID 使用 `s3_geo_*` 和 `s3_can_*`。

## 阶段 4：候选精配准

阶段 4 核验阶段 2、阶段 3 报告以及规范候选 JSON/矩阵的一致性。检查 `candidate_comparison.png` 后，将数据清单中的 `approvals.stage3_reviewed` 设为 `true` 才能运行。

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
