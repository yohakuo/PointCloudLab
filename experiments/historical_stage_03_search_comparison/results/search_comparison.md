# 阶段二：联合搜索对比

各次运行的四份阶段 2 输入哈希、栅格、高度与可信度、平面坐标系和评分配置一致。

| 运行 | 原始/规范候选 | 保留法向×方向分支 | 首位角度 | 首位栅格误差 | 首位点级 Chamfer | 搜索/整次耗时 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| stage_03_grid_sequential | 32/7 | 4 | 270.00° | 3.214 mm | 4.837 mm | 1.29/8.11 s |
| stage_03_grid_joint | 8/4 | 4 | 270.00° | 3.214 mm | 4.837 mm | 16.54/22.22 s |
| stage_03_grid_joint_fast | 8/4 | 4 | 270.00° | 3.214 mm | 4.837 mm | 9.41/15.08 s |

## 方向与平移覆盖

角度在 PCA 平面坐标系内；分支为相距 90° 的四个方向区间，区间内独立优化后角度无需严格相差 90°。平移单位为 mm。

| 运行 | 法向 | 方向分支 | 原始/规范 | 规范角度 | 平移 (u, v) | 最佳栅格误差 |
| --- | --- | ---: | ---: | --- | --- | ---: |
| stage_03_grid_sequential | observed_object_side | 0 | 8/2 | 0.00, 0.00 | (-3.00, 9.00); (-9.00, 9.00) | 3.231 mm |
| stage_03_grid_sequential | observed_object_side | 90 | 8/2 | 90.00, 90.00 | (-9.00, 3.00); (-12.00, -3.00) | 3.215 mm |
| stage_03_grid_sequential | observed_object_side | 180 | 8/2 | 180.00, 180.00 | (-3.00, -3.00); (-3.00, -9.00) | 3.226 mm |
| stage_03_grid_sequential | observed_object_side | 270 | 8/1 | 270.00 | (-0.00, 3.00) | 3.214 mm |
| stage_03_grid_joint | observed_object_side | 0 | 2/1 | 0.00 | (-3.00, 9.00) | 3.230 mm |
| stage_03_grid_joint | observed_object_side | 90 | 2/1 | 90.00 | (-9.00, 3.00) | 3.215 mm |
| stage_03_grid_joint | observed_object_side | 180 | 2/1 | 180.00 | (-3.00, -3.00) | 3.228 mm |
| stage_03_grid_joint | observed_object_side | 270 | 2/1 | 270.00 | (0.00, 3.00) | 3.214 mm |
| stage_03_grid_joint_fast | observed_object_side | 0 | 2/1 | 0.00 | (-3.00, 9.00) | 3.230 mm |
| stage_03_grid_joint_fast | observed_object_side | 90 | 2/1 | 90.00 | (-9.00, 3.00) | 3.215 mm |
| stage_03_grid_joint_fast | observed_object_side | 180 | 2/1 | 180.00 | (-3.00, -3.00) | 3.228 mm |
| stage_03_grid_joint_fast | observed_object_side | 270 | 2/1 | 270.00 | (0.00, 3.00) | 3.214 mm |

耗时是同机依次执行的单次观测，不是统计基准；搜索耗时不含拟合、候选门限和绘图，整次耗时包含这些步骤但不含 Python 导入。
没有真实位姿标签，误差下降仅表示该匹配度量改善，不证明绝对位姿更准或方向唯一。
原始候选为细化后的种子输出，完整粗搜索空间另见 candidate_report.json 的 angle_profile 与 total_evaluations；近似相同的种子可在 SE(3) 去重时合并。
固定数量的阶段 4 接口及人工复核未在本项中执行；旧正式输出和旧选择 ID 未替换。

![搜索覆盖与匹配误差](search_comparison.png)
