# 实验版本与记录

## 已保存的代码基线

2026-09-25 的现有方法、配置、测试和文档已先保存为 Git 提交 `8a0ae34`。提交前运行 `python -m unittest discover -s tests -q`，100 项测试通过。这是快照提交；下面的实验记录工具在后续提交中加入。

## 新实验

先提交代码及配置改动，再用 `scripts/run_experiment.py` 启动实验。它拒绝覆盖已有实验 ID、输出目录或归档文件，也拒绝在代码工作区有未提交改动时运行。`experiments/` 中尚未提交的实验记录不影响下一次运行。

每次运行生成：

- `experiments/<ID>/manifest.json`：代码提交、完整命令、时间、退出状态、配置和输入的路径、大小及 SHA-256、环境变量、结果文件索引和归档哈希；
- `experiments/<ID>/configs/`：配置文件的逐字节快照，包括依赖锁定文件；
- `experiments/<ID>/results/`：指定的结果报告快照（单文件不超过 1 MiB）；
- `outputs/<ID>/`：完整运行产物和合并日志；
- `archives/experiments/<ID>.zip`：该输出目录的本地归档。

`outputs/`、`data/` 和 `archives/` 不进入 Git。实验结束后提交 `experiments/<ID>/`，并把归档和必要的原始输入备份到持久存储；清单里的哈希用于核对备份。仅有 Git 清单无法恢复被删除的点云或完整输出。

例如运行一组新的阶段 3 联合搜索（PowerShell）：

```powershell
$env:OMP_NUM_THREADS = '1'
.\.venv\Scripts\python.exe .\scripts\run_experiment.py `
  --id 20260925-grid-joint-01 `
  --output-dir outputs/20260925-grid-joint-01 `
  --dataset configs/dataset.json `
  --config registration=configs/registration.yaml `
  --input source_board=outputs/stage_02_segmentation/source_board_points.ply `
  --input source_object=outputs/stage_02_segmentation/source_object_points.ply `
  --input target_board=outputs/stage_02_segmentation/target_board_points.pcd `
  --input target_object=outputs/stage_02_segmentation/target_object_points.pcd `
  --input stage2_report=outputs/stage_02_segmentation/segmentation_report.json `
  --result-file candidate_report.json `
  --result-file candidate_summary.csv `
  -- .\.venv\Scripts\python.exe .\scripts\03_generate_candidates.py `
     --dataset configs/dataset.json --output-dir '{output_dir}'
```

若实验使用临时 CLI 覆盖参数，写在命令末尾。阶段 3 的 `candidate_report.json` 保存生效配置和覆盖项；运行器同时保存原始配置快照与实际命令。扰动稳定性实验可将 `coarse_stability.json` 指定为 `--config stability=...`，运行器会从其 `input_paths` 自动登记输入；把 `paired_manifest.json` 与 `summary.json` 都列为 `--result-file`，即可保存配对扰动清单和结果。

实验结束后核对 `manifest.json` 的 `status`、输入哈希、主报告及结果摘要，再提交该实验目录。失败的实验也会有日志、状态和已生成产物的索引，应保留记录，不复用 ID。需要重试时使用新 ID；支持接续的脚本应在自己的运行报告里记录接续关系。

## 已有实验的补录

`scripts/backfill_experiment_records.py` 根据现存报告和产物，为阶段 3 对比、模型重评分及稳定性试验建立 `historical_*` 记录，并分别归档。补录记录保留报告中的配置、输入哈希和结果，同时核对当前输入是否仍与当时报告一致。历史实验发生在代码快照提交之前，因此其 `code_commit` 和精确命令标为未知；不能用新提交 SHA 冒充原运行版本。原始输出不会被修改。

## 2026-09-25 当前阶段结果

实验目标：对贴在白板上的正方体进行点云配准。阶段 3 较大步长联合搜索的四候选运行记录见 `experiments/20260925-grid-joint-fast-fixed-threads/`。阶段 4 已尝试全部四个候选，均为 `no_safe_refinement`，安全门拒绝了 24 次更新，序列化核验通过；因此当前没有可供阶段 5 导出的有效精配准候选。阶段 5 的一次调用因数据清单仍指向旧候选而被拒绝。完整运行产物留在本地 `outputs/`（不纳入 Git）；摘要也见 README 的阶段 4 说明。
