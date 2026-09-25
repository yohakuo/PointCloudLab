# 完整粗配准扰动稳定性实验

总尝试 45；有效 45；失败 0（分母均为方法×扰动输入运行）。
无真实位姿标签；以下都是给定扰动方案下的经验稳定性，获胜频率不是方向正确概率。
候选只供人工复核；没有自动选择方向，也没有触及精配准、正式候选与最终输出。

## 扰动与执行

种子 20260924；每组每档 4 次；试运行 1 次；配对扰动在 paired_inputs/ 保存。
空间采样为带随机相位的均匀体素均值；边界仅从原始 ROI 的既有点增减；白板观测重采样后重新拟合并检查内点率、p90 残差、法向和覆盖。
低/高档扫描仪体素与边界为 0.3/0.6 mm，雷达为 1.5/3 mm，白板体素为 1.5/3 mm。
尺度依据：扫描仪白板残差 p90 约 0.175 mm，雷达白板约 2.322 mm，栅格为 3 mm；这些是观测扰动，不是物体真实尺寸误差。
搜索粗角步长 10.0°、粗平移步长 6 mm；四方向全周搜索及局部细化仍启用。
近似并列阈值 0.25 mm；方向聚类 35.0°，跨运行分配上限 40.0°。

## 汇总（分母见 attempts、valid；方向频率分母为 valid 中有赢家者）

| 方法 | 组 | 档 | 有效/尝试 | 近并列/有效 | 赢家次数 | 分支缺失运行 | 模型回退运行 | 边界命中运行 | 耗时 p50 s |
| --- | --- | --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| sampled_random | board | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.03 |
| sampled_random | board | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.02 |
| sampled_random | boundary | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.04 |
| sampled_random | boundary | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.06 |
| sampled_random | combined | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 0.97 |
| sampled_random | combined | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 0.96 |
| sampled_random | spatial | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 0.91 |
| sampled_random | spatial | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 0.96 |
| grid_sequential | board | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.82 |
| grid_sequential | board | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.83 |
| grid_sequential | boundary | high | 1/1 | 1/1 | {2: 1} | 0 | 0 | 0 | 1.67 |
| grid_sequential | boundary | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.81 |
| grid_sequential | combined | high | 1/1 | 0/1 | {3: 1} | 0 | 0 | 0 | 1.54 |
| grid_sequential | combined | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 1.70 |
| grid_sequential | spatial | high | 1/1 | 0/1 | {1: 1} | 0 | 0 | 0 | 1.70 |
| grid_sequential | spatial | low | 1/1 | 1/1 | {2: 1} | 0 | 0 | 0 | 1.71 |
| grid_joint | board | high | 1/1 | 1/1 | {2: 1} | 0 | 0 | 0 | 8.69 |
| grid_joint | board | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 9.26 |
| grid_joint | boundary | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 8.97 |
| grid_joint | boundary | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 9.18 |
| grid_joint | combined | high | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 8.47 |
| grid_joint | combined | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 8.67 |
| grid_joint | spatial | high | 1/1 | 1/1 | {1: 1} | 1 | 0 | 0 | 8.90 |
| grid_joint | spatial | low | 1/1 | 1/1 | {2: 1} | 0 | 0 | 0 | 9.02 |
| joint_rescore | board | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 9.16 |
| joint_rescore | board | low | 1/1 | 1/1 | {2: 1} | 0 | 0 | 0 | 9.68 |
| joint_rescore | boundary | high | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 9.28 |
| joint_rescore | boundary | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 9.57 |
| joint_rescore | combined | high | 1/1 | 1/1 | {3: 1} | 0 | 0 | 0 | 8.64 |
| joint_rescore | combined | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 8.94 |
| joint_rescore | spatial | high | 1/1 | 1/1 | {2: 1} | 1 | 0 | 0 | 9.09 |
| joint_rescore | spatial | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 9.29 |
| model_in_search | board | high | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 15.60 |
| model_in_search | board | low | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 15.33 |
| model_in_search | boundary | high | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 15.07 |
| model_in_search | boundary | low | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 15.45 |
| model_in_search | combined | high | 1/1 | 1/1 | {3: 1} | 0 | 0 | 0 | 14.66 |
| model_in_search | combined | low | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 15.30 |
| model_in_search | spatial | high | 1/1 | 1/1 | {0: 1} | 0 | 0 | 0 | 15.22 |
| model_in_search | spatial | low | 1/1 | 1/1 | {1: 1} | 0 | 0 | 0 | 15.34 |

## 分支位置与方向

以下位置是在固定雷达白板平面上，把同一个无扰动扫描仪参考点通过实际三维矩阵投影得到。分支通过旋转距离一对一分配；法向符号单独处理，重复旋转先合并。
| 方法 | 组 | 档 | 分支 | 存活/有效 | 固定点偏移 p90 mm | 角度波动 p90 ° | 同一评分内赢家差 p50 mm |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| sampled_random | board | high | 0 | 1/1 | 0.094 | 0.30 | 0.000 |
| sampled_random | board | high | 1 | 1/1 | 0.094 | 0.30 | 0.154 |
| sampled_random | board | high | 2 | 1/1 | 0.096 | 0.31 | 0.188 |
| sampled_random | board | high | 3 | 1/1 | 0.097 | 0.31 | 0.415 |
| sampled_random | board | low | 0 | 1/1 | 0.132 | 0.41 | 0.000 |
| sampled_random | board | low | 1 | 1/1 | 0.128 | 0.41 | 0.140 |
| sampled_random | board | low | 2 | 1/1 | 0.134 | 0.40 | 0.182 |
| sampled_random | board | low | 3 | 1/1 | 0.130 | 0.41 | 0.411 |
| sampled_random | boundary | high | 0 | 1/1 | 2.230 | 18.24 | 0.000 |
| sampled_random | boundary | high | 1 | 1/1 | 1.830 | 18.24 | 0.050 |
| sampled_random | boundary | high | 2 | 1/1 | 1.789 | 9.24 | 0.292 |
| sampled_random | boundary | high | 3 | 1/1 | 1.849 | 18.24 | 0.238 |
| sampled_random | boundary | low | 0 | 1/1 | 0.640 | 4.29 | 0.000 |
| sampled_random | boundary | low | 1 | 1/1 | 0.665 | 4.29 | 0.088 |
| sampled_random | boundary | low | 2 | 1/1 | 0.422 | 4.29 | 0.276 |
| sampled_random | boundary | low | 3 | 1/1 | 0.369 | 4.71 | 0.384 |
| sampled_random | combined | high | 0 | 1/1 | 3.121 | 4.90 | 0.000 |
| sampled_random | combined | high | 1 | 1/1 | 0.238 | 4.90 | 0.031 |
| sampled_random | combined | high | 2 | 1/1 | 1.073 | 13.90 | 0.171 |
| sampled_random | combined | high | 3 | 1/1 | 0.168 | 4.90 | 0.249 |
| sampled_random | combined | low | 0 | 1/1 | 0.526 | 4.49 | 0.000 |
| sampled_random | combined | low | 1 | 1/1 | 0.447 | 4.49 | 0.169 |
| sampled_random | combined | low | 2 | 1/1 | 0.392 | 4.52 | 0.264 |
| sampled_random | combined | low | 3 | 1/1 | 0.384 | 4.49 | 0.440 |
| sampled_random | spatial | high | 0 | 1/1 | 3.359 | 3.10 | 0.000 |
| sampled_random | spatial | high | 1 | 1/1 | 4.508 | 3.10 | 0.007 |
| sampled_random | spatial | high | 2 | 1/1 | 0.419 | 1.10 | 0.195 |
| sampled_random | spatial | high | 3 | 1/1 | 0.299 | 3.10 | 0.195 |
| sampled_random | spatial | low | 0 | 1/1 | 3.980 | 0.71 | 0.000 |
| sampled_random | spatial | low | 1 | 1/1 | 4.792 | 0.71 | 0.020 |
| sampled_random | spatial | low | 2 | 1/1 | 3.035 | 0.29 | 0.088 |
| sampled_random | spatial | low | 3 | 1/1 | 3.347 | 0.71 | 0.219 |
| grid_sequential | board | high | 0 | 1/1 | 12.016 | 0.30 | 0.000 |
| grid_sequential | board | high | 1 | 1/1 | 9.016 | 0.31 | 0.002 |
| grid_sequential | board | high | 2 | 1/1 | 9.014 | 0.30 | 0.063 |
| grid_sequential | board | high | 3 | 1/1 | 9.018 | 0.31 | 0.047 |
| grid_sequential | board | low | 0 | 1/1 | 0.027 | 0.41 | 0.000 |
| grid_sequential | board | low | 1 | 1/1 | 6.026 | 0.41 | 0.033 |
| grid_sequential | board | low | 2 | 1/1 | 6.023 | 0.41 | 0.020 |
| grid_sequential | board | low | 3 | 1/1 | 6.030 | 0.40 | 0.039 |
| grid_sequential | boundary | high | 0 | 1/1 | 1.843 | 1.24 | 0.036 |
| grid_sequential | boundary | high | 1 | 1/1 | 4.402 | 1.24 | 0.026 |
| grid_sequential | boundary | high | 2 | 1/1 | 4.400 | 1.24 | 0.000 |
| grid_sequential | boundary | high | 3 | 1/1 | 4.579 | 1.24 | 0.031 |
| grid_sequential | boundary | low | 0 | 1/1 | 8.525 | 4.29 | 0.000 |
| grid_sequential | boundary | low | 1 | 1/1 | 7.012 | 4.29 | 0.088 |
| grid_sequential | boundary | low | 2 | 1/1 | 1.641 | 4.29 | 0.112 |
| grid_sequential | boundary | low | 3 | 1/1 | 7.510 | 4.29 | 0.288 |
| grid_sequential | combined | high | 0 | 1/1 | 1.226 | 5.90 | 0.289 |
| grid_sequential | combined | high | 1 | 1/1 | 5.860 | 5.90 | 0.499 |
| grid_sequential | combined | high | 2 | 1/1 | 4.383 | 5.90 | 0.771 |
| grid_sequential | combined | high | 3 | 1/1 | 3.396 | 5.90 | 0.000 |
| grid_sequential | combined | low | 0 | 1/1 | 8.371 | 4.52 | 0.000 |
| grid_sequential | combined | low | 1 | 1/1 | 6.750 | 4.52 | 0.058 |
| grid_sequential | combined | low | 2 | 1/1 | 4.345 | 4.52 | 0.127 |
| grid_sequential | combined | low | 3 | 1/1 | 4.660 | 4.52 | 0.088 |
| grid_sequential | spatial | high | 0 | 1/1 | 2.432 | 1.10 | 0.502 |
| grid_sequential | spatial | high | 1 | 1/1 | 5.973 | 6.10 | 0.000 |
| grid_sequential | spatial | high | 2 | 1/1 | 0.773 | 1.10 | 0.446 |
| grid_sequential | spatial | high | 3 | 1/1 | 0.308 | 1.10 | 0.346 |
| grid_sequential | spatial | low | 0 | 1/1 | 8.850 | 0.29 | 0.039 |
| grid_sequential | spatial | low | 1 | 1/1 | 5.897 | 0.29 | 0.017 |
| grid_sequential | spatial | low | 2 | 1/1 | 5.621 | 0.29 | 0.000 |
| grid_sequential | spatial | low | 3 | 1/1 | 6.116 | 0.29 | 0.071 |
| grid_joint | board | high | 0 | 1/1 | 8.336 | 21.73 | 0.038 |
| grid_joint | board | high | 1 | 1/1 | 6.630 | 19.98 | 0.023 |
| grid_joint | board | high | 2 | 1/1 | 7.622 | 19.98 | 0.000 |
| grid_joint | board | high | 3 | 1/1 | 3.970 | 19.73 | 0.053 |
| grid_joint | board | low | 0 | 1/1 | 0.027 | 0.41 | 0.000 |
| grid_joint | board | low | 1 | 1/1 | 3.026 | 0.41 | 0.034 |
| grid_joint | board | low | 2 | 1/1 | 3.023 | 0.41 | 0.025 |
| grid_joint | board | low | 3 | 1/1 | 6.030 | 0.40 | 0.041 |
| grid_joint | boundary | high | 0 | 1/1 | 1.457 | 5.74 | 0.000 |
| grid_joint | boundary | high | 1 | 1/1 | 2.871 | 4.24 | 0.022 |
| grid_joint | boundary | high | 2 | 1/1 | 3.281 | 1.51 | 0.041 |
| grid_joint | boundary | high | 3 | 1/1 | 4.489 | 2.24 | 0.061 |
| grid_joint | boundary | low | 0 | 1/1 | 8.525 | 4.29 | 0.000 |
| grid_joint | boundary | low | 1 | 1/1 | 7.012 | 4.29 | 0.088 |
| grid_joint | boundary | low | 2 | 1/1 | 1.641 | 4.29 | 0.116 |
| grid_joint | boundary | low | 3 | 1/1 | 7.510 | 4.29 | 0.289 |
| grid_joint | combined | high | 0 | 1/1 | 2.312 | 1.12 | 0.506 |
| grid_joint | combined | high | 1 | 1/1 | 7.931 | 31.10 | 0.000 |
| grid_joint | combined | high | 2 | 1/1 | 4.626 | 31.10 | 0.789 |
| grid_joint | combined | high | 3 | 1/1 | 5.100 | 2.91 | 0.126 |
| grid_joint | combined | low | 0 | 1/1 | 8.371 | 4.52 | 0.000 |
| grid_joint | combined | low | 1 | 1/1 | 6.750 | 4.52 | 0.062 |
| grid_joint | combined | low | 2 | 1/1 | 4.345 | 4.52 | 0.127 |
| grid_joint | combined | low | 3 | 1/1 | 4.660 | 4.52 | 0.088 |
| grid_joint | spatial | high | 0 | 0/1 | nan | nan | nan |
| grid_joint | spatial | high | 1 | 1/1 | 8.613 | 3.40 | 0.000 |
| grid_joint | spatial | high | 2 | 1/1 | 1.369 | 16.85 | 0.311 |
| grid_joint | spatial | high | 3 | 1/1 | 3.061 | 27.35 | 0.203 |
| grid_joint | spatial | low | 0 | 1/1 | 8.850 | 0.29 | 0.040 |
| grid_joint | spatial | low | 1 | 1/1 | 5.897 | 0.29 | 0.017 |
| grid_joint | spatial | low | 2 | 1/1 | 5.621 | 0.29 | 0.000 |
| grid_joint | spatial | low | 3 | 1/1 | 6.116 | 0.29 | 0.073 |
| joint_rescore | board | high | 0 | 1/1 | 8.336 | 21.73 | 0.000 |
| joint_rescore | board | high | 1 | 1/1 | 3.970 | 19.73 | 0.027 |
| joint_rescore | board | high | 2 | 1/1 | 7.622 | 19.98 | 0.035 |
| joint_rescore | board | high | 3 | 1/1 | 6.630 | 19.98 | 0.022 |
| joint_rescore | board | low | 0 | 1/1 | 0.027 | 0.41 | 0.009 |
| joint_rescore | board | low | 1 | 1/1 | 6.030 | 0.40 | 0.058 |
| joint_rescore | board | low | 2 | 1/1 | 3.023 | 0.41 | 0.000 |
| joint_rescore | board | low | 3 | 1/1 | 3.026 | 0.41 | 0.010 |
| joint_rescore | boundary | high | 0 | 1/1 | 1.457 | 5.74 | 0.022 |
| joint_rescore | boundary | high | 1 | 1/1 | 4.489 | 2.24 | 0.000 |
| joint_rescore | boundary | high | 2 | 1/1 | 3.281 | 1.51 | 0.054 |
| joint_rescore | boundary | high | 3 | 1/1 | 2.871 | 4.24 | 0.016 |
| joint_rescore | boundary | low | 0 | 1/1 | 8.525 | 4.29 | 0.000 |
| joint_rescore | boundary | low | 1 | 1/1 | 1.606 | 4.29 | 0.060 |
| joint_rescore | boundary | low | 2 | 1/1 | 1.641 | 4.29 | 0.030 |
| joint_rescore | boundary | low | 3 | 1/1 | 2.579 | 4.29 | 0.023 |
| joint_rescore | combined | high | 0 | 1/1 | 5.912 | 31.10 | 0.073 |
| joint_rescore | combined | high | 1 | 1/1 | 6.466 | 1.61 | 0.038 |
| joint_rescore | combined | high | 2 | 1/1 | 4.626 | 31.10 | 0.121 |
| joint_rescore | combined | high | 3 | 1/1 | 7.931 | 31.10 | 0.000 |
| joint_rescore | combined | low | 0 | 1/1 | 8.371 | 4.52 | 0.000 |
| joint_rescore | combined | low | 1 | 1/1 | 4.660 | 4.52 | 0.045 |
| joint_rescore | combined | low | 2 | 1/1 | 4.345 | 4.52 | 0.010 |
| joint_rescore | combined | low | 3 | 1/1 | 6.750 | 4.52 | 0.057 |
| joint_rescore | spatial | high | 0 | 0/1 | nan | nan | nan |
| joint_rescore | spatial | high | 1 | 1/1 | 3.061 | 27.35 | 0.061 |
| joint_rescore | spatial | high | 2 | 1/1 | 1.369 | 16.85 | 0.000 |
| joint_rescore | spatial | high | 3 | 1/1 | 6.822 | 5.10 | 0.040 |
| joint_rescore | spatial | low | 0 | 1/1 | 8.850 | 0.29 | 0.000 |
| joint_rescore | spatial | low | 1 | 1/1 | 6.116 | 0.29 | 0.033 |
| joint_rescore | spatial | low | 2 | 1/1 | 5.621 | 0.29 | 0.014 |
| joint_rescore | spatial | low | 3 | 1/1 | 5.897 | 0.29 | 0.019 |
| model_in_search | board | high | 0 | 1/1 | 0.022 | 0.30 | 0.034 |
| model_in_search | board | high | 1 | 1/1 | 0.022 | 0.30 | 0.000 |
| model_in_search | board | high | 2 | 1/1 | 2.982 | 0.31 | 0.010 |
| model_in_search | board | high | 3 | 1/1 | 3.016 | 0.31 | 0.034 |
| model_in_search | board | low | 0 | 1/1 | 0.024 | 0.41 | 0.000 |
| model_in_search | board | low | 1 | 1/1 | 0.027 | 0.41 | 0.009 |
| model_in_search | board | low | 2 | 1/1 | 3.030 | 0.40 | 0.009 |
| model_in_search | board | low | 3 | 1/1 | 2.974 | 0.41 | 0.010 |
| model_in_search | boundary | high | 0 | 1/1 | 3.777 | 20.76 | 0.034 |
| model_in_search | boundary | high | 1 | 1/1 | 1.367 | 1.24 | 0.000 |
| model_in_search | boundary | high | 2 | 1/1 | 4.222 | 0.99 | 0.001 |
| model_in_search | boundary | high | 3 | 1/1 | 3.530 | 1.24 | 0.012 |
| model_in_search | boundary | low | 0 | 1/1 | 2.614 | 4.29 | 0.011 |
| model_in_search | boundary | low | 1 | 1/1 | 8.525 | 4.29 | 0.000 |
| model_in_search | boundary | low | 2 | 1/1 | 4.543 | 4.29 | 0.035 |
| model_in_search | boundary | low | 3 | 1/1 | 4.045 | 4.29 | 0.023 |
| model_in_search | combined | high | 0 | 1/1 | 4.795 | 5.90 | 0.141 |
| model_in_search | combined | high | 1 | 1/1 | 9.867 | 31.10 | 0.091 |
| model_in_search | combined | high | 2 | 1/1 | 0.026 | 31.10 | 0.043 |
| model_in_search | combined | high | 3 | 1/1 | 10.254 | 31.10 | 0.000 |
| model_in_search | combined | low | 0 | 1/1 | 7.313 | 4.52 | 0.010 |
| model_in_search | combined | low | 1 | 1/1 | 8.371 | 4.52 | 0.000 |
| model_in_search | combined | low | 2 | 1/1 | 4.660 | 4.52 | 0.045 |
| model_in_search | combined | low | 3 | 1/1 | 3.814 | 4.52 | 0.042 |
| model_in_search | spatial | high | 0 | 1/1 | 0.457 | 13.60 | 0.000 |
| model_in_search | spatial | high | 1 | 1/1 | 1.245 | 6.10 | 0.066 |
| model_in_search | spatial | high | 2 | 1/1 | 3.349 | 13.60 | 0.003 |
| model_in_search | spatial | high | 3 | 1/1 | 9.551 | 6.10 | 0.008 |
| model_in_search | spatial | low | 0 | 1/1 | 8.621 | 0.29 | 0.014 |
| model_in_search | spatial | low | 1 | 1/1 | 8.850 | 0.29 | 0.000 |
| model_in_search | spatial | low | 2 | 1/1 | 6.116 | 0.29 | 0.033 |
| model_in_search | spatial | low | 3 | 1/1 | 11.893 | 0.29 | 0.019 |

## 经验判读

位置稳定以各存活分支固定点偏移 p90 均不超过 3 mm 为操作阈值；方向竞争以多分支获胜或近似并列为证据。此分类只针对配置中的扰动。
| 方法 | 组 | 档 | 位置 | 方向 | 跨方向固定点最大距离 p90 mm |
| --- | --- | --- | --- | --- | ---: |
| sampled_random | board | high | 稳定 | 有歧义 | 10.808 |
| sampled_random | board | low | 稳定 | 有歧义 | 10.808 |
| sampled_random | boundary | high | 稳定 | 有歧义 | 8.275 |
| sampled_random | boundary | low | 稳定 | 有歧义 | 10.267 |
| sampled_random | combined | high | 不稳定或证据不足 | 有歧义 | 12.774 |
| sampled_random | combined | low | 稳定 | 有歧义 | 10.712 |
| sampled_random | spatial | high | 不稳定或证据不足 | 有歧义 | 14.464 |
| sampled_random | spatial | low | 不稳定或证据不足 | 有歧义 | 12.236 |
| grid_sequential | board | high | 不稳定或证据不足 | 有歧义 | 12.000 |
| grid_sequential | board | low | 不稳定或证据不足 | 有歧义 | 15.000 |
| grid_sequential | boundary | high | 不稳定或证据不足 | 有歧义 | 14.788 |
| grid_sequential | boundary | low | 不稳定或证据不足 | 有歧义 | 14.368 |
| grid_sequential | combined | high | 不稳定或证据不足 | 未见切换，仍不能判唯一 | 14.452 |
| grid_sequential | combined | low | 不稳定或证据不足 | 有歧义 | 10.933 |
| grid_sequential | spatial | high | 不稳定或证据不足 | 未见切换，仍不能判唯一 | 11.756 |
| grid_sequential | spatial | low | 不稳定或证据不足 | 有歧义 | 11.970 |
| grid_joint | board | high | 不稳定或证据不足 | 有歧义 | 11.848 |
| grid_joint | board | low | 不稳定或证据不足 | 有歧义 | 12.369 |
| grid_joint | boundary | high | 不稳定或证据不足 | 有歧义 | 11.992 |
| grid_joint | boundary | low | 不稳定或证据不足 | 有歧义 | 14.368 |
| grid_joint | combined | high | 不稳定或证据不足 | 有歧义 | 11.020 |
| grid_joint | combined | low | 不稳定或证据不足 | 有歧义 | 10.933 |
| grid_joint | spatial | high | 不稳定或证据不足 | 有歧义 | 11.857 |
| grid_joint | spatial | low | 不稳定或证据不足 | 有歧义 | 11.970 |
| joint_rescore | board | high | 不稳定或证据不足 | 有歧义 | 11.848 |
| joint_rescore | board | low | 不稳定或证据不足 | 有歧义 | 12.369 |
| joint_rescore | boundary | high | 不稳定或证据不足 | 有歧义 | 11.992 |
| joint_rescore | boundary | low | 不稳定或证据不足 | 有歧义 | 19.781 |
| joint_rescore | combined | high | 不稳定或证据不足 | 有歧义 | 14.734 |
| joint_rescore | combined | low | 不稳定或证据不足 | 有歧义 | 10.933 |
| joint_rescore | spatial | high | 不稳定或证据不足 | 有歧义 | 11.226 |
| joint_rescore | spatial | low | 不稳定或证据不足 | 有歧义 | 11.970 |
| model_in_search | board | high | 不稳定或证据不足 | 有歧义 | 12.000 |
| model_in_search | board | low | 不稳定或证据不足 | 有歧义 | 12.000 |
| model_in_search | boundary | high | 不稳定或证据不足 | 有歧义 | 13.829 |
| model_in_search | boundary | low | 不稳定或证据不足 | 有歧义 | 19.781 |
| model_in_search | combined | high | 不稳定或证据不足 | 有歧义 | 17.138 |
| model_in_search | combined | low | 不稳定或证据不足 | 有歧义 | 19.776 |
| model_in_search | spatial | high | 不稳定或证据不足 | 有歧义 | 10.696 |
| model_in_search | spatial | low | 不稳定或证据不足 | 有歧义 | 11.970 |

## 批次趋稳检查

按有效运行数分批，比较末两批累计位置 p90、赢家频率和近并列比例。少量重复下频率分辨率很粗，满足阈值也不构成置信保证。
| 方法 | 组 | 档 | 状态 | 最后有效数 | p90 变化 mm | 赢家频率最大变化 | 近并列比例变化 |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| grid_joint | board | high | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | board | low | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | boundary | high | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | boundary | low | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | combined | high | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | combined | low | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | spatial | high | insufficient_batches | 0 | nan | nan | nan |
| grid_joint | spatial | low | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | board | high | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | board | low | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | boundary | high | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | boundary | low | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | combined | high | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | combined | low | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | spatial | high | insufficient_batches | 0 | nan | nan | nan |
| grid_sequential | spatial | low | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | board | high | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | board | low | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | boundary | high | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | boundary | low | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | combined | high | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | combined | low | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | spatial | high | insufficient_batches | 0 | nan | nan | nan |
| joint_rescore | spatial | low | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | board | high | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | board | low | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | boundary | high | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | boundary | low | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | combined | high | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | combined | low | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | spatial | high | insufficient_batches | 0 | nan | nan | nan |
| model_in_search | spatial | low | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | board | high | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | board | low | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | boundary | high | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | boundary | low | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | combined | high | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | combined | low | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | spatial | high | insufficient_batches | 0 | nan | nan | nan |
| sampled_random | spatial | low | insufficient_batches | 0 | nan | nan | nan |

未达到配置趋稳条件的单元：40/40。因此其频率和高分位数仍有统计限制。

不同评分体系原始分数没有直接相减；`joint_rescore` 与 `grid_joint` 使用同一完整搜索候选池，`model_in_search` 独立改变搜索目标与候选池。
候选池逐输入的精确矩阵交集见 pool_changes.csv；模型参与搜索的池变化归属搜索目标，冻结池重评分只改变评价次序。
若某分支获胜较多，也不能据此认定其为真实方向；近似对称性与无真实标签仍要求人工复核。
精配准接口要求 16 个规范候选而本实验的联合方法通常不足 16 个；这是独立衔接事项，不复制候选凑数。

## 产物

逐次记录 runs/；配对清单 paired_inputs/；汇总 summary.json、summary.csv、branch_summary.csv；图 fixed_plane_positions.png、direction_frequency.png、score_gaps.png。
