import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
 
# Configure matplotlib for non-interactive backend (server-friendly)
plt.switch_backend('Agg')  # Use non-interactive backend for headless environments
plt.rcParams['font.family'] = 'DejaVu Sans'  # Set Ubuntu-compatible font
 
# Define navigation algorithm performance metrics (RMS error values)
performance_data = {
    'KF': {  # Kalman Filter
        'Latitude': 0.046104,   # Degrees error (converted to meters at equator)
        'Longitude': 0.245085,  # Degrees error (converted to meters at equator)
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
    'EKF+RTS': {  # EKF with Rauch-Tung-Striebel smoother
        'Latitude': 0.012959,
        'Longitude': 0.019882,
        'Altitude': 0.006577,
        'Yaw': 0.001972,
        'Pitch': 0.000263,
        'Roll': 0.000568
    }
}
 
# Create DataFrame for structured data manipulation
df = pd.DataFrame(performance_data).T  # Transpose for algorithm-based indexing
position_metrics = ['Latitude', 'Longitude', 'Altitude']  # Navigation position parameters
attitude_metrics = ['Yaw', 'Pitch', 'Roll']  # Vehicle orientation parameters
all_metrics = position_metrics + attitude_metrics  # Combined metrics
 
def calculate_improvement(original, improved):
    """
    Compute percentage improvement between two values
    
    Args:
        original: Baseline measurement value
        improved: Enhanced measurement value
        
    Returns:
        float: Percentage improvement (positive indicates reduction in error)
    """
    return ((original - improved) / original) * 100
 
# Calculate performance improvement metrics
improvement_records = []
for metric in all_metrics:
    # Compute KF to EKF improvement
    kf_to_ekf_improv = calculate_improvement(df.loc['KF', metric], df.loc['EKF', metric])
    
    # Compute KF to EKF+RTS improvement
    kf_to_ekfrts_improv = calculate_improvement(df.loc['KF', metric], df.loc['EKF+RTS', metric])
    
    # Compute EKF to EKF+RTS improvement
    ekf_to_ekfrts_improv = calculate_improvement(df.loc['EKF', metric], df.loc['EKF+RTS', metric])
    
    improvement_records.append({
        'Metric': metric,
        'KF→EKF': kf_to_ekf_improv,
        'KF→EKF+RTS': kf_to_ekfrts_improv,
        'EKF→EKF+RTS': ekf_to_ekfrts_improv
    })
 
# Create improvement DataFrame
improvement_df = pd.DataFrame(improvement_records)
improvement_df.set_index('Metric', inplace=True)  # Set metrics as index
 
# Output directory
output_dir = '../output/nav_eval'
os.makedirs(output_dir, exist_ok=True)
 
# 1. Position Error Comparison Visualization
plt.figure(figsize=(12, 7))
x_indices = np.arange(len(position_metrics))  # X-axis positions
bar_width = 0.25  # Width for clustered bars
 
# Plot KF performance
plt.bar(x_indices - bar_width, df.loc['KF', position_metrics], bar_width,
        label='KF', color='#E74C3C', edgecolor='black')  # Red
# Plot EKF performance
plt.bar(x_indices, df.loc['EKF', position_metrics], bar_width,
        label='EKF', color='#F39C12', edgecolor='black')  # Orange
# Plot EKF+RTS performance
plt.bar(x_indices + bar_width, df.loc['EKF+RTS', position_metrics], bar_width,
        label='EKF+RTS', color='#27AE60', edgecolor='black')  # Green
 
# Configure plot aesthetics
plt.xlabel('Position Metrics', fontsize=12, fontweight='bold')
plt.ylabel('RMS Error (meters)', fontsize=12, fontweight='bold')
plt.title('Position Error Comparison', fontsize=14, fontweight='bold')
plt.xticks(x_indices, position_metrics, fontsize=11)
plt.legend(fontsize=10)
plt.grid(axis='y', linestyle='--', alpha=0.7)  # Horizontal gridlines
 
# Add data labels
for i, metric in enumerate(position_metrics):
    for j, algo in enumerate(['KF', 'EKF', 'EKF+RTS']):
        height = df.loc[algo, metric]
        plt.text(i + (j-1)*bar_width, height + 0.002, f'{height:.6f}', 
                 ha='center', va='bottom', fontsize=8, rotation=0)
 
plt.tight_layout()
plt.savefig(f'{output_dir}/position_comparison.png', dpi=300)  # Export high-res image
plt.close()  # Release memory
 
# 2. Attitude Error Comparison (Logarithmic Scale) - IMPROVED
plt.figure(figsize=(12, 8))  # Slightly taller figure
x_indices = np.arange(len(attitude_metrics))
bar_width = 0.25
 
# Use logarithmic scale for better visualization of small values
plt.yscale('log')
 
# Calculate appropriate y-axis limits
min_val = min(df[attitude_metrics].min().min(), 1e-6)  # Avoid 0 for log scale
max_val = max(df[attitude_metrics].max().max(), 1e-3)
plt.ylim(min_val * 0.8, max_val * 1.2) # Add padding
 
# Plot attitude errors
bar_kf = plt.bar(x_indices - bar_width, df.loc['KF', attitude_metrics], bar_width,
                 label='KF', color='#E74C3C', edgecolor='black')
bar_ekf = plt.bar(x_indices, df.loc['EKF', attitude_metrics], bar_width,
                  label='EKF', color='#F39C12', edgecolor='black')
bar_ekfrts = plt.bar(x_indices + bar_width, df.loc['EKF+RTS', attitude_metrics], bar_width,
                     label='EKF+RTS', color='#27AE60', edgecolor='black')
 
# Configure plot
plt.xlabel('Attitude Metrics', fontsize=12, fontweight='bold')
plt.ylabel('RMS Error (degrees, log scale)', fontsize=12, fontweight='bold')
plt.title('Attitude Error Comparison (Log Scale)', fontsize=14, fontweight='bold')
plt.xticks(x_indices, attitude_metrics, fontsize=11)
plt.legend(fontsize=10)
plt.grid(axis='y', linestyle='--', alpha=0.7)
 
# Add data labels for all bars
for bars in [bar_kf, bar_ekf, bar_ekfrts]:
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2.0, height * 1.2, f'{height:.6f}', 
                 ha='center', va='bottom', fontsize=8, rotation=0)
 
plt.tight_layout()
plt.savefig(f'{output_dir}/attitude_comparison.png', dpi=300)
plt.close()
 
# 3. Performance Improvement Heatmap - IMPROVED
plt.figure(figsize=(11, 7))  # Wider figure
ax = plt.gca()  # Get current axes
 
# Create color-mapped matrix of improvement percentages
heatmap = ax.matshow(improvement_df, cmap='RdYlGn', vmin=-100, vmax=100)
 
# Annotate each cell with percentage value
for i in range(len(improvement_df)):
    for j in range(len(improvement_df.columns)):
        value = improvement_df.iloc[i, j]
        # Use high-contrast text color based on cell brightness
        text_color = 'white' if abs(value) > 50 else 'black'
        plt.text(j, i, f'{value:.1f}%', 
                 ha='center', va='center', 
                 color=text_color, fontsize=10, fontweight='bold')
 
# Configure axes with rotated labels
ax.set_xticks(range(len(improvement_df.columns)))
ax.set_xticklabels(improvement_df.columns, fontsize=9, ha='center') 
ax.set_yticks(range(len(improvement_df.index)))
ax.set_yticklabels(improvement_df.index, fontsize=10)
 
plt.title('Performance Improvement Percentage', fontsize=14, fontweight='bold', pad=20)
plt.colorbar(heatmap, label='Improvement Percentage (%)')  # Add color scale
plt.tight_layout()
plt.savefig(f'{output_dir}/improvement_heatmap.png', dpi=300)
plt.close()

# 4. Generate Performance Analysis Report
report = """
===== Navigation Performance Analysis Report =====

[Position Errors]
Algorithm    Latitude (m) Longitude (m) Altitude (m)
----------------------------------------------------
KF           {kf_lat:.6f}   {kf_lon:.6f}    {kf_alt:.6f}
EKF          {ekf_lat:.6f}   {ekf_lon:.6f}    {ekf_alt:.6f}
EKF+RTS      {ekfrts_lat:.6f}   {ekfrts_lon:.6f}    {ekfrts_alt:.6f}

[Attitude Errors (degrees)]
Algorithm    Yaw         Pitch       Roll
----------------------------------------------------
KF           {kf_yaw:.6f}   {kf_pitch:.6f}  {kf_roll:.6f}
EKF          {ekf_yaw:.6f}   {ekf_pitch:.6f}  {ekf_roll:.6f}
EKF+RTS      {ekfrts_yaw:.6f}   {ekfrts_pitch:.6f}  {ekfrts_roll:.6f}

===== Performance Improvement Metrics =====
Metric       KF→EKF (%)   KF→EKF+RTS (%)   EKF→EKF+RTS (%)
----------------------------------------------------------------
Latitude     {imp_lat_kf_ekf:>7.1f}      {imp_lat_kf_ekfrts:>7.1f}         {imp_lat_ekf_ekfrts:>7.1f}
Longitude    {imp_lon_kf_ekf:>7.1f}      {imp_lon_kf_ekfrts:>7.1f}         {imp_lon_ekf_ekfrts:>7.1f}
Altitude     {imp_alt_kf_ekf:>7.1f}      {imp_alt_kf_ekfrts:>7.1f}         {imp_alt_ekf_ekfrts:>7.1f}
Yaw          {imp_yaw_kf_ekf:>7.1f}      {imp_yaw_kf_ekfrts:>7.1f}         {imp_yaw_ekf_ekfrts:>7.1f}
Pitch        {imp_pitch_kf_ekf:>7.1f}      {imp_pitch_kf_ekfrts:>7.1f}         {imp_pitch_ekf_ekfrts:>7.1f}
Roll         {imp_roll_kf_ekf:>7.1f}      {imp_roll_kf_ekfrts:>7.1f}         {imp_roll_ekf_ekfrts:>7.1f}

===== Key Observations =====
1. EKF vs KF:
   - Longitude error reduced by {imp_lon_kf_ekf:.1f}% (from {kf_lon:.3f}m to {ekf_lon:.3f}m)
   - Altitude error reduced by {imp_alt_kf_ekf:.1f}% (from {kf_alt:.3f}m to {ekf_alt:.3f}m)

2. EKF+RTS vs KF:
   - Latitude error reduced by {imp_lat_kf_ekfrts:.1f}% (from {kf_lat:.3f}m to {ekfrts_lat:.3f}m)
   - Longitude error reduced by {imp_lon_kf_ekfrts:.1f}% (from {kf_lon:.3f}m to {ekfrts_lon:.3f}m)
   - Altitude error reduced by {imp_alt_kf_ekfrts:.1f}% (from {kf_alt:.3f}m to {ekfrts_alt:.3f}m)

3. EKF+RTS vs EKF:
   - Latitude error reduced by {imp_lat_ekf_ekfrts:.1f}% (from {ekf_lat:.3f}m to {ekfrts_lat:.3f}m)
   - Altitude error reduced by {imp_alt_ekf_ekfrts:.1f}% (from {ekf_alt:.3f}m to {ekfrts_alt:.3f}m)
   - Pitch error increased by {abs_imp_pitch_ekf_ekfrts:.1f}% (from {ekf_pitch:.6f}° to {ekfrts_pitch:.6f}°)

4. Overall Performance Assessment:
   - EKF+RTS achieves centimeter-level accuracy in all position metrics
   - Most significant improvement observed in longitude error (KF: {kf_lon:.3f}m → EKF+RTS: {ekfrts_lon:.3f}m)
   - Yaw estimation shows consistent improvement through algorithmic enhancements
   - Pitch metric shows minor regression in EKF+RTS due to smoothing trade-offs
"""

# Populate report template with data
formatted_report = report.format(
    # Position error metrics
    kf_lat=performance_data['KF']['Latitude'],
    kf_lon=performance_data['KF']['Longitude'],
    kf_alt=performance_data['KF']['Altitude'],
    ekf_lat=performance_data['EKF']['Latitude'],
    ekf_lon=performance_data['EKF']['Longitude'],
    ekf_alt=performance_data['EKF']['Altitude'],
    ekfrts_lat=performance_data['EKF+RTS']['Latitude'],
    ekfrts_lon=performance_data['EKF+RTS']['Longitude'],
    ekfrts_alt=performance_data['EKF+RTS']['Altitude'],
    
    # Attitude error metrics
    kf_yaw=performance_data['KF']['Yaw'],
    kf_pitch=performance_data['KF']['Pitch'],
    kf_roll=performance_data['KF']['Roll'],
    ekf_yaw=performance_data['EKF']['Yaw'],
    ekf_pitch=performance_data['EKF']['Pitch'],
    ekf_roll=performance_data['EKF']['Roll'],
    ekfrts_yaw=performance_data['EKF+RTS']['Yaw'],
    ekfrts_pitch=performance_data['EKF+RTS']['Pitch'],
    ekfrts_roll=performance_data['EKF+RTS']['Roll'],
    
    # Improvement percentages
    imp_lat_kf_ekf=improvement_df.loc['Latitude', 'KF→EKF'],
    imp_lat_kf_ekfrts=improvement_df.loc['Latitude', 'KF→EKF+RTS'],
    imp_lat_ekf_ekfrts=improvement_df.loc['Latitude', 'EKF→EKF+RTS'],
    imp_lon_kf_ekf=improvement_df.loc['Longitude', 'KF→EKF'],
    imp_lon_kf_ekfrts=improvement_df.loc['Longitude', 'KF→EKF+RTS'],
    imp_lon_ekf_ekfrts=improvement_df.loc['Longitude', 'EKF→EKF+RTS'],
    imp_alt_kf_ekf=improvement_df.loc['Altitude', 'KF→EKF'],
    imp_alt_kf_ekfrts=improvement_df.loc['Altitude', 'KF→EKF+RTS'],
    imp_alt_ekf_ekfrts=improvement_df.loc['Altitude', 'EKF→EKF+RTS'],
    imp_yaw_kf_ekf=improvement_df.loc['Yaw', 'KF→EKF'],
    imp_yaw_kf_ekfrts=improvement_df.loc['Yaw', 'KF→EKF+RTS'],
    imp_yaw_ekf_ekfrts=improvement_df.loc['Yaw', 'EKF→EKF+RTS'],
    imp_pitch_kf_ekf=improvement_df.loc['Pitch', 'KF→EKF'],
    imp_pitch_kf_ekfrts=improvement_df.loc['Pitch', 'KF→EKF+RTS'],
    imp_pitch_ekf_ekfrts=improvement_df.loc['Pitch', 'EKF→EKF+RTS'],
    imp_roll_kf_ekf=improvement_df.loc['Roll', 'KF→EKF'],
    imp_roll_kf_ekfrts=improvement_df.loc['Roll', 'KF→EKF+RTS'],
    imp_roll_ekf_ekfrts=improvement_df.loc['Roll', 'EKF→EKF+RTS'],
    
    # Special handling for pitch metric increase
    abs_imp_pitch_ekf_ekfrts=abs(improvement_df.loc['Pitch', 'EKF→EKF+RTS'])
)

# Save report to text file
with open(f'{output_dir}/performance_report.txt', 'w') as f:
    f.write(formatted_report)

# Print execution summary
print("="*70)
print("Navigation Performance Analysis Completed! Results saved in: ")
print(f"    {output_dir}")
print("1. position_comparison.png - Positional error comparison")
print("2. attitude_comparison.png - Attitude error comparison (log scale)")
print("3. improvement_heatmap.png - Performance improvement heatmap")
print("4. performance_report.txt - Comprehensive performance analysis")
print("="*70)
print("\nKey Findings Summary:")
print(f"Longitude Error Improvement: KF({performance_data['KF']['Longitude']:.3f}m) → "
      f"EKF({performance_data['EKF']['Longitude']:.3f}m) → "
      f"EKF+RTS({performance_data['EKF+RTS']['Longitude']:.3f}m)")
print(f"  → KF→EKF: {improvement_df.loc['Longitude', 'KF→EKF']:.1f}% improvement")
print(f"  → EKF→EKF+RTS: {improvement_df.loc['Longitude', 'EKF→EKF+RTS']:.1f}% improvement")
print(f"  → Overall Improvement: {improvement_df.loc['Longitude', 'KF→EKF+RTS']:.1f}%")
print("\nUsage Instructions:")
print(f"eog {output_dir}/*.png  # View generated images")
print(f"cat {output_dir}/performance_report.txt  # View full analysis report")
