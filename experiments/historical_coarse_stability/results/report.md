# 完整粗配准扰动稳定性实验

总尝试 165；有效 165；失败 0（分母均为方法×扰动输入运行）。
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
| sampled_random | board | high | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 1.04 |
| sampled_random | board | low | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 1.04 |
| sampled_random | boundary | high | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 1.05 |
| sampled_random | boundary | low | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 1.05 |
| sampled_random | combined | high | 4/4 | 4/4 | {0: 3, 1: 1} | 0 | 0 | 0 | 0.94 |
| sampled_random | combined | low | 4/4 | 4/4 | {0: 3, 1: 1} | 0 | 0 | 0 | 0.97 |
| sampled_random | spatial | high | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 0.93 |
| sampled_random | spatial | low | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 0.98 |
| grid_sequential | board | high | 4/4 | 4/4 | {0: 2, 1: 2} | 0 | 0 | 0 | 1.85 |
| grid_sequential | board | low | 4/4 | 4/4 | {0: 1, 1: 2, 2: 1} | 0 | 0 | 0 | 1.83 |
| grid_sequential | boundary | high | 4/4 | 4/4 | {2: 4} | 0 | 0 | 0 | 1.83 |
| grid_sequential | boundary | low | 4/4 | 4/4 | {0: 2, 1: 2} | 0 | 0 | 0 | 1.85 |
| grid_sequential | combined | high | 4/4 | 2/4 | {3: 2, 1: 2} | 0 | 0 | 0 | 1.59 |
| grid_sequential | combined | low | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 1.72 |
| grid_sequential | spatial | high | 4/4 | 1/4 | {1: 3, 3: 1} | 0 | 0 | 0 | 1.63 |
| grid_sequential | spatial | low | 4/4 | 4/4 | {2: 3, 1: 1} | 0 | 0 | 0 | 1.74 |
| grid_joint | board | high | 4/4 | 4/4 | {2: 1, 1: 2, 0: 1} | 0 | 0 | 0 | 9.27 |
| grid_joint | board | low | 4/4 | 4/4 | {0: 2, 1: 1, 2: 1} | 0 | 0 | 0 | 9.33 |
| grid_joint | boundary | high | 4/4 | 4/4 | {0: 4} | 0 | 0 | 0 | 9.43 |
| grid_joint | boundary | low | 4/4 | 4/4 | {0: 2, 1: 2} | 0 | 0 | 0 | 9.43 |
| grid_joint | combined | high | 4/4 | 3/4 | {1: 2, 3: 2} | 0 | 0 | 0 | 8.70 |
| grid_joint | combined | low | 4/4 | 4/4 | {0: 3, 1: 1} | 0 | 0 | 0 | 9.26 |
| grid_joint | spatial | high | 4/4 | 2/4 | {1: 3, 3: 1} | 1 | 0 | 0 | 8.81 |
| grid_joint | spatial | low | 4/4 | 4/4 | {2: 4} | 0 | 0 | 0 | 9.20 |
| joint_rescore | board | high | 4/4 | 4/4 | {0: 3, 2: 1} | 0 | 0 | 0 | 9.69 |
| joint_rescore | board | low | 4/4 | 4/4 | {2: 1, 3: 2, 0: 1} | 0 | 0 | 0 | 9.75 |
| joint_rescore | boundary | high | 4/4 | 4/4 | {1: 2, 2: 2} | 0 | 0 | 0 | 9.79 |
| joint_rescore | boundary | low | 4/4 | 4/4 | {0: 2, 3: 2} | 0 | 0 | 0 | 9.83 |
| joint_rescore | combined | high | 4/4 | 4/4 | {3: 1, 1: 3} | 0 | 0 | 0 | 8.90 |
| joint_rescore | combined | low | 4/4 | 4/4 | {0: 1, 1: 2, 3: 1} | 0 | 0 | 0 | 9.53 |
| joint_rescore | spatial | high | 4/4 | 4/4 | {2: 3, 0: 1} | 1 | 0 | 0 | 9.00 |
| joint_rescore | spatial | low | 4/4 | 4/4 | {0: 2, 2: 1, 3: 1} | 0 | 0 | 0 | 9.48 |
| model_in_search | board | high | 4/4 | 4/4 | {1: 4} | 0 | 0 | 0 | 15.42 |
| model_in_search | board | low | 4/4 | 4/4 | {0: 1, 3: 1, 1: 2} | 0 | 0 | 0 | 15.42 |
| model_in_search | boundary | high | 4/4 | 4/4 | {1: 2, 2: 2} | 0 | 0 | 0 | 15.55 |
| model_in_search | boundary | low | 4/4 | 4/4 | {1: 4} | 0 | 0 | 0 | 15.62 |
| model_in_search | combined | high | 4/4 | 4/4 | {3: 2, 2: 2} | 0 | 0 | 0 | 15.08 |
| model_in_search | combined | low | 4/4 | 4/4 | {1: 1, 3: 2, 2: 1} | 0 | 0 | 0 | 15.50 |
| model_in_search | spatial | high | 4/4 | 4/4 | {0: 3, 1: 1} | 0 | 0 | 0 | 15.16 |
| model_in_search | spatial | low | 4/4 | 4/4 | {1: 2, 0: 1, 2: 1} | 0 | 0 | 0 | 15.30 |

## 分支位置与方向

以下位置是在固定雷达白板平面上，把同一个无扰动扫描仪参考点通过实际三维矩阵投影得到。分支通过旋转距离一对一分配；法向符号单独处理，重复旋转先合并。
| 方法 | 组 | 档 | 分支 | 存活/有效 | 固定点偏移 p90 mm | 角度波动 p90 ° | 同一评分内赢家差 p50 mm |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| sampled_random | board | high | 0 | 4/4 | 0.084 | 0.27 | 0.000 |
| sampled_random | board | high | 1 | 4/4 | 0.084 | 0.27 | 0.151 |
| sampled_random | board | high | 2 | 4/4 | 0.086 | 0.28 | 0.186 |
| sampled_random | board | high | 3 | 4/4 | 0.087 | 0.28 | 0.413 |
| sampled_random | board | low | 0 | 4/4 | 0.116 | 0.35 | 0.000 |
| sampled_random | board | low | 1 | 4/4 | 0.114 | 0.36 | 0.154 |
| sampled_random | board | low | 2 | 4/4 | 0.118 | 0.35 | 0.189 |
| sampled_random | board | low | 3 | 4/4 | 0.115 | 0.36 | 0.421 |
| sampled_random | boundary | high | 0 | 4/4 | 2.230 | 18.24 | 0.000 |
| sampled_random | boundary | high | 1 | 4/4 | 1.830 | 18.24 | 0.057 |
| sampled_random | boundary | high | 2 | 4/4 | 2.727 | 9.24 | 0.152 |
| sampled_random | boundary | high | 3 | 4/4 | 1.849 | 18.24 | 0.226 |
| sampled_random | boundary | low | 0 | 4/4 | 1.124 | 9.13 | 0.000 |
| sampled_random | boundary | low | 1 | 4/4 | 4.122 | 7.13 | 0.115 |
| sampled_random | boundary | low | 2 | 4/4 | 5.741 | 4.29 | 0.177 |
| sampled_random | boundary | low | 3 | 4/4 | 0.629 | 9.13 | 0.314 |
| sampled_random | combined | high | 0 | 4/4 | 2.840 | 5.22 | 0.000 |
| sampled_random | combined | high | 1 | 4/4 | 4.808 | 6.07 | 0.034 |
| sampled_random | combined | high | 2 | 4/4 | 1.009 | 13.86 | 0.131 |
| sampled_random | combined | high | 3 | 4/4 | 3.730 | 4.86 | 0.195 |
| sampled_random | combined | low | 0 | 4/4 | 3.018 | 8.30 | 0.000 |
| sampled_random | combined | low | 1 | 4/4 | 3.094 | 8.30 | 0.057 |
| sampled_random | combined | low | 2 | 4/4 | 0.792 | 7.62 | 0.116 |
| sampled_random | combined | low | 3 | 4/4 | 0.881 | 8.30 | 0.286 |
| sampled_random | spatial | high | 0 | 4/4 | 5.498 | 7.39 | 0.000 |
| sampled_random | spatial | high | 1 | 4/4 | 3.727 | 16.61 | 0.034 |
| sampled_random | spatial | high | 2 | 4/4 | 3.912 | 1.77 | 0.194 |
| sampled_random | spatial | high | 3 | 4/4 | 4.571 | 16.61 | 0.247 |
| sampled_random | spatial | low | 0 | 4/4 | 4.183 | 9.18 | 0.000 |
| sampled_random | spatial | low | 1 | 4/4 | 4.868 | 14.98 | 0.028 |
| sampled_random | spatial | low | 2 | 4/4 | 2.182 | 0.52 | 0.070 |
| sampled_random | spatial | low | 3 | 4/4 | 2.860 | 14.87 | 0.237 |
| grid_sequential | board | high | 0 | 4/4 | 9.313 | 0.27 | 0.001 |
| grid_sequential | board | high | 1 | 4/4 | 7.214 | 0.28 | 0.001 |
| grid_sequential | board | high | 2 | 4/4 | 7.206 | 0.27 | 0.050 |
| grid_sequential | board | high | 3 | 4/4 | 8.108 | 0.28 | 0.063 |
| grid_sequential | board | low | 0 | 4/4 | 8.104 | 0.35 | 0.030 |
| grid_sequential | board | low | 1 | 4/4 | 5.121 | 0.36 | 0.006 |
| grid_sequential | board | low | 2 | 4/4 | 6.019 | 0.36 | 0.047 |
| grid_sequential | board | low | 3 | 4/4 | 5.124 | 0.35 | 0.057 |
| grid_sequential | boundary | high | 0 | 4/4 | 3.076 | 1.24 | 0.044 |
| grid_sequential | boundary | high | 1 | 4/4 | 4.402 | 1.24 | 0.058 |
| grid_sequential | boundary | high | 2 | 4/4 | 5.225 | 1.24 | 0.000 |
| grid_sequential | boundary | high | 3 | 4/4 | 4.579 | 1.24 | 0.035 |
| grid_sequential | boundary | low | 0 | 4/4 | 8.525 | 4.29 | 0.023 |
| grid_sequential | boundary | low | 1 | 4/4 | 7.012 | 4.29 | 0.044 |
| grid_sequential | boundary | low | 2 | 4/4 | 3.008 | 4.29 | 0.127 |
| grid_sequential | boundary | low | 3 | 4/4 | 7.510 | 4.29 | 0.217 |
| grid_sequential | combined | high | 0 | 4/4 | 2.938 | 5.56 | 0.435 |
| grid_sequential | combined | high | 1 | 4/4 | 7.998 | 5.56 | 0.170 |
| grid_sequential | combined | high | 2 | 4/4 | 7.540 | 5.56 | 0.587 |
| grid_sequential | combined | high | 3 | 4/4 | 7.711 | 5.56 | 0.020 |
| grid_sequential | combined | low | 0 | 4/4 | 8.361 | 8.10 | 0.000 |
| grid_sequential | combined | low | 1 | 4/4 | 5.901 | 8.10 | 0.055 |
| grid_sequential | combined | low | 2 | 4/4 | 6.195 | 8.10 | 0.045 |
| grid_sequential | combined | low | 3 | 4/4 | 5.557 | 8.10 | 0.082 |
| grid_sequential | spatial | high | 0 | 4/4 | 5.364 | 1.79 | 0.456 |
| grid_sequential | spatial | high | 1 | 4/4 | 6.392 | 4.83 | 0.000 |
| grid_sequential | spatial | high | 2 | 4/4 | 7.835 | 1.79 | 0.497 |
| grid_sequential | spatial | high | 3 | 4/4 | 8.175 | 1.79 | 0.246 |
| grid_sequential | spatial | low | 0 | 4/4 | 9.046 | 0.52 | 0.037 |
| grid_sequential | spatial | low | 1 | 4/4 | 5.933 | 0.52 | 0.020 |
| grid_sequential | spatial | low | 2 | 4/4 | 5.661 | 0.52 | 0.000 |
| grid_sequential | spatial | low | 3 | 4/4 | 6.216 | 0.52 | 0.070 |
| grid_joint | board | high | 0 | 4/4 | 8.803 | 15.28 | 0.021 |
| grid_joint | board | high | 1 | 4/4 | 5.544 | 14.05 | 0.012 |
| grid_joint | board | high | 2 | 4/4 | 6.231 | 14.05 | 0.027 |
| grid_joint | board | high | 3 | 4/4 | 5.379 | 13.88 | 0.068 |
| grid_joint | board | low | 0 | 4/4 | 7.981 | 15.87 | 0.003 |
| grid_joint | board | low | 1 | 4/4 | 4.493 | 16.22 | 0.023 |
| grid_joint | board | low | 2 | 4/4 | 6.346 | 15.34 | 0.032 |
| grid_joint | board | low | 3 | 4/4 | 5.214 | 15.34 | 0.031 |
| grid_joint | boundary | high | 0 | 4/4 | 2.350 | 5.74 | 0.000 |
| grid_joint | boundary | high | 1 | 4/4 | 3.194 | 6.51 | 0.061 |
| grid_joint | boundary | high | 2 | 4/4 | 4.237 | 4.76 | 0.021 |
| grid_joint | boundary | high | 3 | 4/4 | 4.489 | 7.51 | 0.073 |
| grid_joint | boundary | low | 0 | 4/4 | 8.525 | 4.29 | 0.023 |
| grid_joint | boundary | low | 1 | 4/4 | 7.012 | 4.29 | 0.044 |
| grid_joint | boundary | low | 2 | 4/4 | 3.008 | 4.29 | 0.128 |
| grid_joint | boundary | low | 3 | 4/4 | 7.510 | 4.29 | 0.217 |
| grid_joint | combined | high | 0 | 4/4 | 2.108 | 10.21 | 0.463 |
| grid_joint | combined | high | 1 | 4/4 | 8.390 | 28.27 | 0.010 |
| grid_joint | combined | high | 2 | 4/4 | 7.540 | 22.98 | 0.500 |
| grid_joint | combined | high | 3 | 4/4 | 7.711 | 3.43 | 0.063 |
| grid_joint | combined | low | 0 | 4/4 | 8.361 | 8.98 | 0.000 |
| grid_joint | combined | low | 1 | 4/4 | 5.891 | 11.08 | 0.057 |
| grid_joint | combined | low | 2 | 4/4 | 6.495 | 10.73 | 0.065 |
| grid_joint | combined | low | 3 | 4/4 | 5.557 | 16.68 | 0.063 |
| grid_joint | spatial | high | 0 | 3/4 | 5.533 | 1.81 | 0.408 |
| grid_joint | spatial | high | 1 | 4/4 | 7.969 | 2.93 | 0.000 |
| grid_joint | spatial | high | 2 | 4/4 | 4.559 | 32.64 | 0.359 |
| grid_joint | spatial | high | 3 | 4/4 | 8.175 | 33.12 | 0.176 |
| grid_joint | spatial | low | 0 | 4/4 | 9.046 | 0.52 | 0.043 |
| grid_joint | spatial | low | 1 | 4/4 | 6.161 | 0.52 | 0.025 |
| grid_joint | spatial | low | 2 | 4/4 | 6.093 | 0.52 | 0.000 |
| grid_joint | spatial | low | 3 | 4/4 | 7.566 | 10.55 | 0.082 |
| joint_rescore | board | high | 0 | 4/4 | 8.803 | 15.28 | 0.000 |
| joint_rescore | board | high | 1 | 4/4 | 5.379 | 13.88 | 0.025 |
| joint_rescore | board | high | 2 | 4/4 | 6.231 | 14.05 | 0.026 |
| joint_rescore | board | high | 3 | 4/4 | 5.544 | 14.05 | 0.032 |
| joint_rescore | board | low | 0 | 4/4 | 7.981 | 15.87 | 0.023 |
| joint_rescore | board | low | 1 | 4/4 | 5.214 | 15.34 | 0.030 |
| joint_rescore | board | low | 2 | 4/4 | 5.113 | 0.36 | 0.020 |
| joint_rescore | board | low | 3 | 4/4 | 3.021 | 0.36 | 0.005 |
| joint_rescore | boundary | high | 0 | 4/4 | 2.350 | 5.74 | 0.026 |
| joint_rescore | boundary | high | 1 | 4/4 | 4.489 | 7.51 | 0.006 |
| joint_rescore | boundary | high | 2 | 4/4 | 4.237 | 4.76 | 0.027 |
| joint_rescore | boundary | high | 3 | 4/4 | 3.194 | 6.51 | 0.016 |
| joint_rescore | boundary | low | 0 | 4/4 | 8.525 | 4.29 | 0.012 |
| joint_rescore | boundary | low | 1 | 4/4 | 3.043 | 4.29 | 0.039 |
| joint_rescore | boundary | low | 2 | 4/4 | 3.008 | 4.29 | 0.021 |
| joint_rescore | boundary | low | 3 | 4/4 | 3.010 | 4.29 | 0.011 |
| joint_rescore | combined | high | 0 | 4/4 | 4.628 | 25.68 | 0.053 |
| joint_rescore | combined | high | 1 | 4/4 | 8.092 | 3.35 | 0.000 |
| joint_rescore | combined | high | 2 | 4/4 | 7.540 | 22.98 | 0.085 |
| joint_rescore | combined | high | 3 | 4/4 | 8.390 | 25.90 | 0.040 |
| joint_rescore | combined | low | 0 | 4/4 | 8.361 | 8.98 | 0.003 |
| joint_rescore | combined | low | 1 | 4/4 | 5.557 | 11.60 | 0.013 |
| joint_rescore | combined | low | 2 | 4/4 | 6.495 | 10.73 | 0.029 |
| joint_rescore | combined | low | 3 | 4/4 | 5.891 | 11.08 | 0.021 |
| joint_rescore | spatial | high | 0 | 3/4 | 5.533 | 1.81 | 0.114 |
| joint_rescore | spatial | high | 1 | 4/4 | 8.175 | 19.70 | 0.079 |
| joint_rescore | spatial | high | 2 | 4/4 | 4.559 | 32.64 | 0.000 |
| joint_rescore | spatial | high | 3 | 4/4 | 6.716 | 4.13 | 0.057 |
| joint_rescore | spatial | low | 0 | 4/4 | 9.006 | 0.52 | 0.003 |
| joint_rescore | spatial | low | 1 | 4/4 | 7.566 | 10.55 | 0.036 |
| joint_rescore | spatial | low | 2 | 4/4 | 6.093 | 0.52 | 0.018 |
| joint_rescore | spatial | low | 3 | 4/4 | 6.161 | 0.52 | 0.017 |
| model_in_search | board | high | 0 | 4/4 | 0.021 | 0.27 | 0.018 |
| model_in_search | board | high | 1 | 4/4 | 0.021 | 0.27 | 0.000 |
| model_in_search | board | high | 2 | 4/4 | 2.982 | 0.28 | 0.017 |
| model_in_search | board | high | 3 | 4/4 | 3.013 | 0.28 | 0.028 |
| model_in_search | board | low | 0 | 4/4 | 4.202 | 0.36 | 0.012 |
| model_in_search | board | low | 1 | 4/4 | 2.103 | 0.35 | 0.004 |
| model_in_search | board | low | 2 | 4/4 | 3.018 | 0.35 | 0.008 |
| model_in_search | board | low | 3 | 4/4 | 9.004 | 0.36 | 0.013 |
| model_in_search | boundary | high | 0 | 4/4 | 3.777 | 20.76 | 0.024 |
| model_in_search | boundary | high | 1 | 4/4 | 2.065 | 4.26 | 0.004 |
| model_in_search | boundary | high | 2 | 4/4 | 4.222 | 7.01 | 0.000 |
| model_in_search | boundary | high | 3 | 4/4 | 3.530 | 4.51 | 0.007 |
| model_in_search | boundary | low | 0 | 4/4 | 2.614 | 4.29 | 0.020 |
| model_in_search | boundary | low | 1 | 4/4 | 8.525 | 4.29 | 0.000 |
| model_in_search | boundary | low | 2 | 4/4 | 4.543 | 4.29 | 0.028 |
| model_in_search | boundary | low | 3 | 4/4 | 6.009 | 4.29 | 0.022 |
| model_in_search | combined | high | 0 | 4/4 | 4.544 | 19.14 | 0.080 |
| model_in_search | combined | high | 1 | 4/4 | 7.587 | 25.60 | 0.095 |
| model_in_search | combined | high | 2 | 4/4 | 4.511 | 24.37 | 0.022 |
| model_in_search | combined | high | 3 | 4/4 | 9.724 | 31.19 | 0.010 |
| model_in_search | combined | low | 0 | 4/4 | 7.215 | 12.65 | 0.008 |
| model_in_search | combined | low | 1 | 4/4 | 6.438 | 8.45 | 0.004 |
| model_in_search | combined | low | 2 | 4/4 | 4.594 | 13.70 | 0.024 |
| model_in_search | combined | low | 3 | 4/4 | 6.885 | 13.53 | 0.011 |
| model_in_search | spatial | high | 0 | 4/4 | 2.956 | 10.08 | 0.000 |
| model_in_search | spatial | high | 1 | 4/4 | 4.090 | 4.83 | 0.076 |
| model_in_search | spatial | high | 2 | 4/4 | 5.497 | 14.47 | 0.032 |
| model_in_search | spatial | high | 3 | 4/4 | 11.560 | 9.07 | 0.033 |
| model_in_search | spatial | low | 0 | 4/4 | 8.619 | 4.02 | 0.015 |
| model_in_search | spatial | low | 1 | 4/4 | 9.006 | 4.37 | 0.003 |
| model_in_search | spatial | low | 2 | 4/4 | 6.136 | 0.52 | 0.021 |
| model_in_search | spatial | low | 3 | 4/4 | 11.832 | 3.84 | 0.017 |

## 经验判读

位置稳定以各存活分支固定点偏移 p90 均不超过 3 mm 为操作阈值；方向竞争以多分支获胜或近似并列为证据。此分类只针对配置中的扰动。
| 方法 | 组 | 档 | 位置 | 方向 | 跨方向固定点最大距离 p90 mm |
| --- | --- | --- | --- | --- | ---: |
| sampled_random | board | high | 稳定 | 有歧义 | 10.808 |
| sampled_random | board | low | 稳定 | 有歧义 | 10.808 |
| sampled_random | boundary | high | 稳定 | 有歧义 | 11.947 |
| sampled_random | boundary | low | 不稳定或证据不足 | 有歧义 | 11.617 |
| sampled_random | combined | high | 不稳定或证据不足 | 有歧义 | 12.321 |
| sampled_random | combined | low | 不稳定或证据不足 | 有歧义 | 13.596 |
| sampled_random | spatial | high | 不稳定或证据不足 | 有歧义 | 16.593 |
| sampled_random | spatial | low | 不稳定或证据不足 | 有歧义 | 14.255 |
| grid_sequential | board | high | 不稳定或证据不足 | 有歧义 | 12.369 |
| grid_sequential | board | low | 不稳定或证据不足 | 有歧义 | 15.000 |
| grid_sequential | boundary | high | 不稳定或证据不足 | 有歧义 | 14.788 |
| grid_sequential | boundary | low | 不稳定或证据不足 | 有歧义 | 14.368 |
| grid_sequential | combined | high | 不稳定或证据不足 | 有歧义 | 20.431 |
| grid_sequential | combined | low | 不稳定或证据不足 | 有歧义 | 17.243 |
| grid_sequential | spatial | high | 不稳定或证据不足 | 有歧义 | 19.012 |
| grid_sequential | spatial | low | 不稳定或证据不足 | 有歧义 | 12.352 |
| grid_joint | board | high | 不稳定或证据不足 | 有歧义 | 14.211 |
| grid_joint | board | low | 不稳定或证据不足 | 有歧义 | 12.316 |
| grid_joint | boundary | high | 不稳定或证据不足 | 有歧义 | 11.992 |
| grid_joint | boundary | low | 不稳定或证据不足 | 有歧义 | 14.368 |
| grid_joint | combined | high | 不稳定或证据不足 | 有歧义 | 20.431 |
| grid_joint | combined | low | 不稳定或证据不足 | 有歧义 | 17.165 |
| grid_joint | spatial | high | 不稳定或证据不足 | 有歧义 | 16.569 |
| grid_joint | spatial | low | 不稳定或证据不足 | 有歧义 | 18.241 |
| joint_rescore | board | high | 不稳定或证据不足 | 有歧义 | 14.211 |
| joint_rescore | board | low | 不稳定或证据不足 | 有歧义 | 13.707 |
| joint_rescore | boundary | high | 不稳定或证据不足 | 有歧义 | 11.992 |
| joint_rescore | boundary | low | 不稳定或证据不足 | 有歧义 | 19.781 |
| joint_rescore | combined | high | 不稳定或证据不足 | 有歧义 | 20.431 |
| joint_rescore | combined | low | 不稳定或证据不足 | 有歧义 | 17.010 |
| joint_rescore | spatial | high | 不稳定或证据不足 | 有歧义 | 16.100 |
| joint_rescore | spatial | low | 不稳定或证据不足 | 有歧义 | 18.241 |
| model_in_search | board | high | 不稳定或证据不足 | 有歧义 | 15.000 |
| model_in_search | board | low | 不稳定或证据不足 | 有歧义 | 14.100 |
| model_in_search | boundary | high | 不稳定或证据不足 | 有歧义 | 14.555 |
| model_in_search | boundary | low | 不稳定或证据不足 | 有歧义 | 19.781 |
| model_in_search | combined | high | 不稳定或证据不足 | 有歧义 | 16.862 |
| model_in_search | combined | low | 不稳定或证据不足 | 有歧义 | 18.377 |
| model_in_search | spatial | high | 不稳定或证据不足 | 有歧义 | 16.688 |
| model_in_search | spatial | low | 不稳定或证据不足 | 有歧义 | 14.820 |

## 批次趋稳检查

按有效运行数分批，比较末两批累计位置 p90、赢家频率和近并列比例。少量重复下频率分辨率很粗，满足阈值也不构成置信保证。
| 方法 | 组 | 档 | 状态 | 最后有效数 | p90 变化 mm | 赢家频率最大变化 | 近并列比例变化 |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| grid_joint | board | high | not_converged | 4 | 0.143 | 0.250 | 0.000 |
| grid_joint | board | low | not_converged | 4 | 0.897 | 0.250 | 0.000 |
| grid_joint | boundary | high | within_configured_tolerances | 4 | 0.050 | 0.000 | 0.000 |
| grid_joint | boundary | low | within_configured_tolerances | 4 | 0.203 | 0.000 | 0.000 |
| grid_joint | combined | high | not_converged | 4 | 0.266 | 0.000 | 0.250 |
| grid_joint | combined | low | not_converged | 4 | 0.426 | 0.250 | 0.000 |
| grid_joint | spatial | high | not_converged | 4 | 0.876 | 0.250 | 0.500 |
| grid_joint | spatial | low | within_configured_tolerances | 4 | 0.001 | 0.000 | 0.000 |
| grid_sequential | board | high | within_configured_tolerances | 4 | 0.900 | 0.000 | 0.000 |
| grid_sequential | board | low | not_converged | 4 | 0.001 | 0.250 | 0.000 |
| grid_sequential | boundary | high | within_configured_tolerances | 4 | 0.129 | 0.000 | 0.000 |
| grid_sequential | boundary | low | within_configured_tolerances | 4 | 0.203 | 0.000 | 0.000 |
| grid_sequential | combined | high | not_converged | 4 | 0.478 | 0.500 | 0.500 |
| grid_sequential | combined | low | within_configured_tolerances | 4 | 0.426 | 0.000 | 0.000 |
| grid_sequential | spatial | high | not_converged | 4 | 0.906 | 0.250 | 0.250 |
| grid_sequential | spatial | low | not_converged | 4 | 0.001 | 0.250 | 0.000 |
| joint_rescore | board | high | not_converged | 4 | 0.143 | 0.250 | 0.000 |
| joint_rescore | board | low | not_converged | 4 | 0.015 | 0.250 | 0.000 |
| joint_rescore | boundary | high | within_configured_tolerances | 4 | 0.050 | 0.000 | 0.000 |
| joint_rescore | boundary | low | within_configured_tolerances | 4 | 0.345 | 0.000 | 0.000 |
| joint_rescore | combined | high | not_converged | 4 | 0.266 | 0.250 | 0.000 |
| joint_rescore | combined | low | not_converged | 4 | 0.426 | 0.250 | 0.000 |
| joint_rescore | spatial | high | not_converged | 4 | 0.876 | 0.250 | 0.000 |
| joint_rescore | spatial | low | not_converged | 4 | 0.426 | 0.250 | 0.000 |
| model_in_search | board | high | within_configured_tolerances | 4 | 0.001 | 0.000 | 0.000 |
| model_in_search | board | low | not_converged | 4 | 0.600 | 0.500 | 0.000 |
| model_in_search | boundary | high | within_configured_tolerances | 4 | 0.017 | 0.000 | 0.000 |
| model_in_search | boundary | low | within_configured_tolerances | 4 | 0.503 | 0.000 | 0.000 |
| model_in_search | combined | high | within_configured_tolerances | 4 | 0.806 | 0.000 | 0.000 |
| model_in_search | combined | low | not_converged | 4 | 0.495 | 0.250 | 0.000 |
| model_in_search | spatial | high | not_converged | 4 | 1.136 | 0.250 | 0.000 |
| model_in_search | spatial | low | not_converged | 4 | 1.370 | 0.250 | 0.000 |
| sampled_random | board | high | within_configured_tolerances | 4 | 0.001 | 0.000 | 0.000 |
| sampled_random | board | low | within_configured_tolerances | 4 | 0.002 | 0.000 | 0.000 |
| sampled_random | boundary | high | within_configured_tolerances | 4 | 0.100 | 0.000 | 0.000 |
| sampled_random | boundary | low | within_configured_tolerances | 4 | 0.324 | 0.000 | 0.000 |
| sampled_random | combined | high | not_converged | 4 | 0.481 | 0.250 | 0.000 |
| sampled_random | combined | low | not_converged | 4 | 1.487 | 0.250 | 0.000 |
| sampled_random | spatial | high | within_configured_tolerances | 4 | 0.483 | 0.000 | 0.000 |
| sampled_random | spatial | low | within_configured_tolerances | 4 | 0.307 | 0.000 | 0.000 |

未达到配置趋稳条件的单元：21/40。因此其频率和高分位数仍有统计限制。

不同评分体系原始分数没有直接相减；`joint_rescore` 与 `grid_joint` 使用同一完整搜索候选池，`model_in_search` 独立改变搜索目标与候选池。
候选池逐输入的精确矩阵交集见 pool_changes.csv；模型参与搜索的池变化归属搜索目标，冻结池重评分只改变评价次序。
若某分支获胜较多，也不能据此认定其为真实方向；近似对称性与无真实标签仍要求人工复核。
精配准接口要求 16 个规范候选而本实验的联合方法通常不足 16 个；这是独立衔接事项，不复制候选凑数。

## 产物

逐次记录 runs/；配对清单 paired_inputs/；汇总 summary.json、summary.csv、branch_summary.csv；图 fixed_plane_positions.png、direction_frequency.png、score_gaps.png。
