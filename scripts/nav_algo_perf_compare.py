import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

# Configure matplotlib for non-interactive backend (server-friendly)
plt.switch_backend('Agg')  # Use non-interactive backend for headless environments
plt.rcParams['font.family'] = 'DejaVu Sans'  # Set Ubuntu-compatible font

# ========= 1) 原始+新增算法数据 =========
# 定义导航算法性能指标（RMS）
performance_data = {
    'KF': {  # Kalman Filter
        'Latitude': 0.046104,   # Meters error
        'Longitude': 0.245085,  # Meters error
        'Altitude': 0.067023,   # Meters error
        'Yaw': 0.002833,        # Degrees error
        'Pitch': 0.000083,      # Degrees error
        'Roll': 0.000519        # Degrees error
    },
    'EKF': {  # Extended Kalman Filter
        'Latitude': 0.045025,
        'Longitude': 0.048222,
        'Altitude': 0.014661,
        'Yaw': 0.002178,
        'Pitch': 0.000080,
        'Roll': 0.000523
    },
    'UKF': {  # Unscented Kalman Filter (新加：平滑前)
        'Latitude': 0.042458,
        'Longitude': 0.047164,
        'Altitude': 0.014742,
        'Yaw': 0.002175,
        'Pitch': 0.000085,
        'Roll': 0.000523
    },
    'EKF+RTS': {  # EKF with Rauch-Tung-Striebel smoother
        'Latitude': 0.012959,
        'Longitude': 0.019882,
        'Altitude': 0.006577,
        'Yaw': 0.001972,
        'Pitch': 0.000263,
        'Roll': 0.000568
    },
    'UKF+RTS': {  # 新加：UKF 经 RTS 平滑
        'Latitude': 0.012936,
        'Longitude': 0.019751,
        'Altitude': 0.006214,
        'Yaw': 0.001985,
        'Pitch': 0.000257,
        'Roll': 0.000567
    }
}

# ========= 2) DataFrame 结构 =========
df = pd.DataFrame(performance_data).T  # 算法为行
position_metrics = ['Latitude', 'Longitude', 'Altitude']
attitude_metrics = ['Yaw', 'Pitch', 'Roll']
all_metrics = position_metrics + attitude_metrics

# 统一的算法顺序（绘图与标注使用）
algorithms = ['KF', 'EKF', 'UKF', 'EKF+RTS', 'UKF+RTS']

# 颜色映射（保持你原来风格，新增 UKF / UKF+RTS）
colors = {
    'KF': '#E74C3C',       # red
    'EKF': '#F39C12',      # orange
    'UKF': '#3498DB',      # blue
    'EKF+RTS': '#27AE60',  # green
    'UKF+RTS': '#8E44AD'   # purple
}

def calculate_improvement(original, improved):
    """
    计算从 original 到 improved 的百分比改善（正值=误差降低）
    """
    return ((original - improved) / original) * 100

# ========= 3) 改进率计算（含 UKF 路径）=========
improvement_records = []
for metric in all_metrics:
    rec = {'Metric': metric}

    # KF 基线 -> 其他
    rec['KF→EKF'] = calculate_improvement(df.loc['KF', metric], df.loc['EKF', metric])
    rec['KF→UKF'] = calculate_improvement(df.loc['KF', metric], df.loc['UKF', metric])
    rec['KF→EKF+RTS'] = calculate_improvement(df.loc['KF', metric], df.loc['EKF+RTS', metric])
    rec['KF→UKF+RTS'] = calculate_improvement(df.loc['KF', metric], df.loc['UKF+RTS', metric])

    # 各自平滑前后
    rec['EKF→EKF+RTS'] = calculate_improvement(df.loc['EKF', metric], df.loc['EKF+RTS', metric])
    rec['UKF→UKF+RTS'] = calculate_improvement(df.loc['UKF', metric], df.loc['UKF+RTS', metric])

    improvement_records.append(rec)

improvement_df = pd.DataFrame(improvement_records).set_index('Metric')

# ========= 4) 输出目录 =========
output_dir = '../output/nav_eval'
os.makedirs(output_dir, exist_ok=True)

# ========= 5) 位置误差对比图（支持 5 类算法）=========
plt.figure(figsize=(12, 7))
x_indices = np.arange(len(position_metrics))
n_alg = len(algorithms)
bar_width = 0.15  # 根据算法数量压缩宽度
offsets = (np.arange(n_alg) - (n_alg - 1) / 2.0) * bar_width

for j, algo in enumerate(algorithms):
    plt.bar(
        x_indices + offsets[j],
        df.loc[algo, position_metrics],
        bar_width,
        label=algo,
        color=colors.get(algo, None),
        edgecolor='black'
    )

plt.xlabel('Position Metrics', fontsize=12, fontweight='bold')
plt.ylabel('RMS Error (meters)', fontsize=12, fontweight='bold')
plt.title('Position Error Comparison', fontsize=14, fontweight='bold')
plt.xticks(x_indices, position_metrics, fontsize=11)
plt.legend(fontsize=10, ncol=3)
plt.grid(axis='y', linestyle='--', alpha=0.7)

# 数值标签
for i, metric in enumerate(position_metrics):
    for j, algo in enumerate(algorithms):
        height = df.loc[algo, metric]
        plt.text(
            x_indices[i] + offsets[j],
            height + 0.002,
            f'{height:.6f}',
            ha='center',
            va='bottom',
            fontsize=7
        )

plt.tight_layout()
plt.savefig(f'{output_dir}/position_comparison.png', dpi=300)
plt.close()

# ========= 6) 姿态误差对比图（对数轴，支持 5 类算法）=========
plt.figure(figsize=(12, 8))
x_indices = np.arange(len(attitude_metrics))
n_alg = len(algorithms)
bar_width = 0.15
offsets = (np.arange(n_alg) - (n_alg - 1) / 2.0) * bar_width

plt.yscale('log')

min_val = min(df[attitude_metrics].min().min(), 1e-6)  # 避免 0
max_val = max(df[attitude_metrics].max().max(), 1e-3)
plt.ylim(min_val * 0.8, max_val * 1.2)

bar_handles = []
for j, algo in enumerate(algorithms):
    bars = plt.bar(
        x_indices + offsets[j],
        df.loc[algo, attitude_metrics],
        bar_width,
        label=algo,
        color=colors.get(algo, None),
        edgecolor='black'
    )
    bar_handles.append(bars)

plt.xlabel('Attitude Metrics', fontsize=12, fontweight='bold')
plt.ylabel('RMS Error (degrees, log scale)', fontsize=12, fontweight='bold')
plt.title('Attitude Error Comparison (Log Scale)', fontsize=14, fontweight='bold')
plt.xticks(x_indices, attitude_metrics, fontsize=11)
plt.legend(fontsize=10, ncol=3)
plt.grid(axis='y', linestyle='--', alpha=0.7)

# 数值标签
for bars in bar_handles:
    for bar in bars:
        height = bar.get_height()
        plt.text(
            bar.get_x() + bar.get_width()/2.0,
            height * 1.2,
            f'{height:.6f}',
            ha='center',
            va='bottom',
            fontsize=7
        )

plt.tight_layout()
plt.savefig(f'{output_dir}/attitude_comparison.png', dpi=300)
plt.close()

# ========= 7) 改进率热力图（加入 UKF 列）=========
plt.figure(figsize=(12, 7))
ax = plt.gca()
heatmap = ax.matshow(improvement_df, cmap='RdYlGn', vmin=-100, vmax=100)

# 标注每个单元
for i in range(len(improvement_df)):
    for j in range(len(improvement_df.columns)):
        value = improvement_df.iloc[i, j]
        text_color = 'white' if abs(value) > 50 else 'black'
        plt.text(j, i, f'{value:.1f}%', ha='center', va='center',
                 color=text_color, fontsize=10, fontweight='bold')
 
 
# Configure axes with rotated labels

# Configure axes with rotated labels
ax.set_xticks(range(len(improvement_df.columns)))
ax.set_xticklabels(improvement_df.columns, fontsize=9, ha='center')
ax.set_yticks(range(len(improvement_df.index)))
ax.set_yticklabels(improvement_df.index, fontsize=10)

plt.title('Performance Improvement Percentage', fontsize=14, fontweight='bold', pad=20)
plt.colorbar(heatmap, label='Improvement Percentage (%)')
plt.tight_layout()
plt.savefig(f'{output_dir}/improvement_heatmap.png', dpi=300)
plt.close()

# ========= 8) 文本报告（增加 UKF & UKF+RTS）=========
report = """
===== Navigation Performance Analysis Report =====

[Position Errors]
Algorithm    Latitude (m) Longitude (m) Altitude (m)
----------------------------------------------------
KF           {kf_lat:.6f}   {kf_lon:.6f}    {kf_alt:.6f}
EKF          {ekf_lat:.6f}   {ekf_lon:.6f}    {ekf_alt:.6f}
UKF          {ukf_lat:.6f}   {ukf_lon:.6f}    {ukf_alt:.6f}
EKF+RTS      {ekfrts_lat:.6f}   {ekfrts_lon:.6f}    {ekfrts_alt:.6f}
UKF+RTS      {ukfrts_lat:.6f}   {ukfrts_lon:.6f}    {ukfrts_alt:.6f}

[Attitude Errors (degrees)]
Algorithm    Yaw         Pitch       Roll
----------------------------------------------------
KF           {kf_yaw:.6f}   {kf_pitch:.6f}  {kf_roll:.6f}
EKF          {ekf_yaw:.6f}   {ekf_pitch:.6f}  {ekf_roll:.6f}
UKF          {ukf_yaw:.6f}   {ukf_pitch:.6f}  {ukf_roll:.6f}
EKF+RTS      {ekfrts_yaw:.6f}   {ekfrts_pitch:.6f}  {ekfrts_roll:.6f}
UKF+RTS      {ukfrts_yaw:.6f}   {ukfrts_pitch:.6f}  {ukfrts_roll:.6f}

===== Performance Improvement Metrics =====
Metric          KF→EKF   KF→UKF  KF→EKF+RTS  KF→UKF+RTS  EKF→EKF+RTS  UKF→UKF+RTS
-----------------------------------------------------------------------------------
Latitude        {imp_lat_kf_ekf:>7.1f}  {imp_lat_kf_ukf:>7.1f}    {imp_lat_kf_ekfrts:>7.1f}     {imp_lat_kf_ukfrts:>7.1f}      {imp_lat_ekf_ekfrts:>7.1f}       {imp_lat_ukf_ukfrts:>7.1f}
Longitude       {imp_lon_kf_ekf:>7.1f}  {imp_lon_kf_ukf:>7.1f}    {imp_lon_kf_ekfrts:>7.1f}     {imp_lon_kf_ukfrts:>7.1f}      {imp_lon_ekf_ekfrts:>7.1f}       {imp_lon_ukf_ukfrts:>7.1f}
Altitude        {imp_alt_kf_ekf:>7.1f}  {imp_alt_kf_ukf:>7.1f}    {imp_alt_kf_ekfrts:>7.1f}     {imp_alt_kf_ukfrts:>7.1f}      {imp_alt_ekf_ekfrts:>7.1f}       {imp_alt_ukf_ukfrts:>7.1f}
Yaw             {imp_yaw_kf_ekf:>7.1f}  {imp_yaw_kf_ukf:>7.1f}    {imp_yaw_kf_ekfrts:>7.1f}     {imp_yaw_kf_ukfrts:>7.1f}      {imp_yaw_ekf_ekfrts:>7.1f}       {imp_yaw_ukf_ukfrts:>7.1f}
Pitch           {imp_pitch_kf_ekf:>7.1f}  {imp_pitch_kf_ukf:>7.1f}  {imp_pitch_kf_ekfrts:>7.1f}   {imp_pitch_kf_ukfrts:>7.1f}    {imp_pitch_ekf_ekfrts:>7.1f}     {imp_pitch_ukf_ukfrts:>7.1f}
Roll            {imp_roll_kf_ekf:>7.1f}  {imp_roll_kf_ukf:>7.1f}  {imp_roll_kf_ekfrts:>7.1f}    {imp_roll_kf_ukfrts:>7.1f}     {imp_roll_ekf_ekfrts:>7.1f}      {imp_roll_ukf_ukfrts:>7.1f}

===== Key Observations =====
1. EKF vs KF:
   - Longitude error reduced by {imp_lon_kf_ekf:.1f}% (from {kf_lon:.3f}m to {ekf_lon:.3f}m)
   - Altitude error reduced by {imp_alt_kf_ekf:.1f}% (from {kf_alt:.3f}m to {ekf_alt:.3f}m)

2. UKF vs KF:
   - Longitude error reduced by {imp_lon_kf_ukf:.1f}% (from {kf_lon:.3f}m to {ukf_lon:.3f}m)
   - Altitude error reduced by {imp_alt_kf_ukf:.1f}% (from {kf_alt:.3f}m to {ukf_alt:.3f}m)

3. EKF+RTS vs KF:
   - Latitude error reduced by {imp_lat_kf_ekfrts:.1f}% (from {kf_lat:.3f}m to {ekfrts_lat:.3f}m)
   - Longitude error reduced by {imp_lon_kf_ekfrts:.1f}% (from {kf_lon:.3f}m to {ekfrts_lon:.3f}m)
   - Altitude error reduced by {imp_alt_kf_ekfrts:.1f}% (from {kf_alt:.3f}m to {ekfrts_alt:.3f}m)

4. UKF+RTS vs KF:
   - Latitude error reduced by {imp_lat_kf_ukfrts:.1f}% (from {kf_lat:.3f}m to {ukfrts_lat:.3f}m)
   - Longitude error reduced by {imp_lon_kf_ukfrts:.1f}% (from {kf_lon:.3f}m to {ukfrts_lon:.3f}m)
   - Altitude error reduced by {imp_alt_kf_ukfrts:.1f}% (from {kf_alt:.3f}m to {ukfrts_alt:.3f}m)

5. EKF+RTS vs EKF:
   - Latitude error reduced by {imp_lat_ekf_ekfrts:.1f}% (from {ekf_lat:.3f}m to {ekfrts_lat:.3f}m)
   - Altitude error reduced by {imp_alt_ekf_ekfrts:.1f}% (from {ekf_alt:.3f}m to {ekfrts_alt:.3f}m)
   - Pitch error increased by {abs_imp_pitch_ekf_ekfrts:.1f}% (from {ekf_pitch:.6f}° to {ekfrts_pitch:.6f}°)

6. UKF+RTS vs UKF:
   - Latitude error reduced by {imp_lat_ukf_ukfrts:.1f}% (from {ukf_lat:.3f}m to {ukfrts_lat:.3f}m)
   - Altitude error reduced by {imp_alt_ukf_ukfrts:.1f}% (from {ukf_alt:.3f}m to {ukfrts_alt:.3f}m)
   - Pitch error increased by {abs_imp_pitch_ukf_ukfrts:.1f}% (from {ukf_pitch:.6f}° to {ukfrts_pitch:.6f}°)

7. Overall Performance Assessment:
   - Both EKF+RTS and UKF+RTS achieve centimeter-level accuracy in all position metrics.
   - The largest relative gains remain on longitude error (e.g., KF: {kf_lon:.3f}m → EKF+RTS: {ekfrts_lon:.3f}m / UKF+RTS: {ukfrts_lon:.3f}m).
   - Yaw estimation improves after smoothing for both EKF and UKF.
   - Pitch shows a minor regression after smoothing for both EKF and UKF, a known trade-off of RTS smoothing.
"""

formatted_report = report.format(
    # KF
    kf_lat=performance_data['KF']['Latitude'],
    kf_lon=performance_data['KF']['Longitude'],
    kf_alt=performance_data['KF']['Altitude'],
    kf_yaw=performance_data['KF']['Yaw'],
    kf_pitch=performance_data['KF']['Pitch'],
    kf_roll=performance_data['KF']['Roll'],
    # EKF
    ekf_lat=performance_data['EKF']['Latitude'],
    ekf_lon=performance_data['EKF']['Longitude'],
    ekf_alt=performance_data['EKF']['Altitude'],
    ekf_yaw=performance_data['EKF']['Yaw'],
    ekf_pitch=performance_data['EKF']['Pitch'],
    ekf_roll=performance_data['EKF']['Roll'],
    # UKF
    ukf_lat=performance_data['UKF']['Latitude'],
    ukf_lon=performance_data['UKF']['Longitude'],
    ukf_alt=performance_data['UKF']['Altitude'],
    ukf_yaw=performance_data['UKF']['Yaw'],
    ukf_pitch=performance_data['UKF']['Pitch'],
    ukf_roll=performance_data['UKF']['Roll'],
    # EKF+RTS
    ekfrts_lat=performance_data['EKF+RTS']['Latitude'],
    ekfrts_lon=performance_data['EKF+RTS']['Longitude'],
    ekfrts_alt=performance_data['EKF+RTS']['Altitude'],
    ekfrts_yaw=performance_data['EKF+RTS']['Yaw'],
    ekfrts_pitch=performance_data['EKF+RTS']['Pitch'],
    ekfrts_roll=performance_data['EKF+RTS']['Roll'],
    # UKF+RTS
    ukfrts_lat=performance_data['UKF+RTS']['Latitude'],
    ukfrts_lon=performance_data['UKF+RTS']['Longitude'],
    ukfrts_alt=performance_data['UKF+RTS']['Altitude'],
    ukfrts_yaw=performance_data['UKF+RTS']['Yaw'],
    ukfrts_pitch=performance_data['UKF+RTS']['Pitch'],
    ukfrts_roll=performance_data['UKF+RTS']['Roll'],

    # 改进率
    imp_lat_kf_ekf=improvement_df.loc['Latitude', 'KF→EKF'],
    imp_lon_kf_ekf=improvement_df.loc['Longitude', 'KF→EKF'],
    imp_alt_kf_ekf=improvement_df.loc['Altitude', 'KF→EKF'],
    imp_yaw_kf_ekf=improvement_df.loc['Yaw', 'KF→EKF'],
    imp_pitch_kf_ekf=improvement_df.loc['Pitch', 'KF→EKF'],
    imp_roll_kf_ekf=improvement_df.loc['Roll', 'KF→EKF'],

    imp_lat_kf_ukf=improvement_df.loc['Latitude', 'KF→UKF'],
    imp_lon_kf_ukf=improvement_df.loc['Longitude', 'KF→UKF'],
    imp_alt_kf_ukf=improvement_df.loc['Altitude', 'KF→UKF'],
    imp_yaw_kf_ukf=improvement_df.loc['Yaw', 'KF→UKF'],
    imp_pitch_kf_ukf=improvement_df.loc['Pitch', 'KF→UKF'],
    imp_roll_kf_ukf=improvement_df.loc['Roll', 'KF→UKF'],

    imp_lat_kf_ekfrts=improvement_df.loc['Latitude', 'KF→EKF+RTS'],
    imp_lon_kf_ekfrts=improvement_df.loc['Longitude', 'KF→EKF+RTS'],
    imp_alt_kf_ekfrts=improvement_df.loc['Altitude', 'KF→EKF+RTS'],
    imp_yaw_kf_ekfrts=improvement_df.loc['Yaw', 'KF→EKF+RTS'],
    imp_pitch_kf_ekfrts=improvement_df.loc['Pitch', 'KF→EKF+RTS'],
    imp_roll_kf_ekfrts=improvement_df.loc['Roll', 'KF→EKF+RTS'],

    imp_lat_kf_ukfrts=improvement_df.loc['Latitude', 'KF→UKF+RTS'],
    imp_lon_kf_ukfrts=improvement_df.loc['Longitude', 'KF→UKF+RTS'],
    imp_alt_kf_ukfrts=improvement_df.loc['Altitude', 'KF→UKF+RTS'],
    imp_yaw_kf_ukfrts=improvement_df.loc['Yaw', 'KF→UKF+RTS'],
    imp_pitch_kf_ukfrts=improvement_df.loc['Pitch', 'KF→UKF+RTS'],
    imp_roll_kf_ukfrts=improvement_df.loc['Roll', 'KF→UKF+RTS'],

    imp_lat_ekf_ekfrts=improvement_df.loc['Latitude', 'EKF→EKF+RTS'],
    imp_lon_ekf_ekfrts=improvement_df.loc['Longitude', 'EKF→EKF+RTS'],
    imp_alt_ekf_ekfrts=improvement_df.loc['Altitude', 'EKF→EKF+RTS'],
    imp_yaw_ekf_ekfrts=improvement_df.loc['Yaw', 'EKF→EKF+RTS'],
    imp_pitch_ekf_ekfrts=improvement_df.loc['Pitch', 'EKF→EKF+RTS'],
    imp_roll_ekf_ekfrts=improvement_df.loc['Roll', 'EKF→EKF+RTS'],

    imp_lat_ukf_ukfrts=improvement_df.loc['Latitude', 'UKF→UKF+RTS'],
    imp_lon_ukf_ukfrts=improvement_df.loc['Longitude', 'UKF→UKF+RTS'],
    imp_alt_ukf_ukfrts=improvement_df.loc['Altitude', 'UKF→UKF+RTS'],
    imp_yaw_ukf_ukfrts=improvement_df.loc['Yaw', 'UKF→UKF+RTS'],
    imp_pitch_ukf_ukfrts=improvement_df.loc['Pitch', 'UKF→UKF+RTS'],
    imp_roll_ukf_ukfrts=improvement_df.loc['Roll', 'UKF→UKF+RTS'],

    # 特别处理 Pitch 的“增大”为正数表述
    abs_imp_pitch_ekf_ekfrts=abs(improvement_df.loc['Pitch', 'EKF→EKF+RTS']),
    abs_imp_pitch_ukf_ukfrts=abs(improvement_df.loc['Pitch', 'UKF→UKF+RTS'])
)

with open(f'{output_dir}/performance_report.txt', 'w') as f:
    f.write(formatted_report)

# ========= 9) 控制台摘要（同时展示 EKF 与 UKF 链路）=========
print("="*70)
print("Navigation Performance Analysis Completed! Results saved in: ")
print(f"    {output_dir}")
print("1. position_comparison.png - Positional error comparison")
print("2. attitude_comparison.png - Attitude error comparison (log scale)")
print("3. improvement_heatmap.png - Performance improvement heatmap")
print("4. performance_report.txt - Comprehensive performance analysis")
print("="*70)
print("\nKey Findings Summary (Longitude):")
print(f"KF({performance_data['KF']['Longitude']:.3f}m) → "
      f"EKF({performance_data['EKF']['Longitude']:.3f}m) → "
      f"EKF+RTS({performance_data['EKF+RTS']['Longitude']:.3f}m)")
print(f"  → KF→EKF: {improvement_df.loc['Longitude', 'KF→EKF']:.1f}%")
print(f"  → EKF→EKF+RTS: {improvement_df.loc['Longitude', 'EKF→EKF+RTS']:.1f}%")
print(f"  → KF→EKF+RTS: {improvement_df.loc['Longitude', 'KF→EKF+RTS']:.1f}%")

print(f"\nKF({performance_data['KF']['Longitude']:.3f}m) → "
      f"UKF({performance_data['UKF']['Longitude']:.3f}m) → "
      f"UKF+RTS({performance_data['UKF+RTS']['Longitude']:.3f}m)")
print(f"  → KF→UKF: {improvement_df.loc['Longitude', 'KF→UKF']:.1f}%")
print(f"  → UKF→UKF+RTS: {improvement_df.loc['Longitude', 'UKF→UKF+RTS']:.1f}%")
print(f"  → KF→UKF+RTS: {improvement_df.loc['Longitude', 'KF→UKF+RTS']:.1f}%")

print("\nUsage Instructions:")
print(f"eog {output_dir}/*.png  # View generated images")
print(f"cat {output_dir}/performance_report.txt  # View full analysis report")
