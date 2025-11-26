import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

# Read the overtakes summary CSV
overtakes_df = pd.read_csv('overtakes.csv')

# Get all detail CSV files
detail_files = sorted(glob.glob('overtake_*_detail.csv'))

print(f"Generating plots for {len(detail_files)} detail files...")
print(f"Summary CSV contains {len(overtakes_df)} overtakes (indices: {overtakes_df['overtake_index'].tolist()})")

# ===== FIGURE 1: Box and Whisker Plot =====
fig1, ax1 = plt.subplots(figsize=(10, 6))

bp = ax1.boxplot([overtakes_df['min_dv2o'].values], 
                   widths=0.5,
                   patch_artist=True,
                   boxprops=dict(facecolor='lightblue', edgecolor='black'),
                   medianprops=dict(color='black', linewidth=2),
                   whiskerprops=dict(color='black'),
                   capprops=dict(color='black'))

# Add horizontal red line at 0.1m
ax1.axhline(y=0.1, color='red', linestyle='-', linewidth=2, label='0.1 m threshold')

ax1.set_ylabel('Minimum Distance (m)', fontsize=12)
ax1.set_xlabel('All Overtakes', fontsize=12)
ax1.set_xticks([1])
ax1.set_xticklabels(['Overtakes'])
ax1.legend(loc='upper right', fontsize=10)
ax1.grid(axis='y', alpha=0.3)

plt.tight_layout()
plt.savefig('overtake_boxplot.pdf', format='pdf', bbox_inches='tight')
print("Saved: overtake_boxplot.pdf")
plt.close()

# ===== FIGURES 2+: Individual Distance vs Progress Plots =====
plots_saved = 0
plots_skipped = 0

for idx, detail_file in enumerate(detail_files):
    # Extract overtake number from filename
    try:
        overtake_num = int(detail_file.split('_')[1])
    except:
        print(f"Warning: Could not parse overtake number from {detail_file}, skipping...")
        plots_skipped += 1
        continue
    
    # Check if this overtake exists in summary
    matching_rows = overtakes_df.loc[overtakes_df['overtake_index'] == overtake_num]
    
    if matching_rows.empty:
        print(f"Warning: Overtake {overtake_num} not found in overtakes.csv, skipping...")
        plots_skipped += 1
        continue
    
    # Read detail data
    detail_df = pd.read_csv(detail_file)
    
    # Create new figure
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot distance vs s (convert to numpy arrays)
    ax.plot(detail_df['s'].values, detail_df['dv2o'].values, 
            linewidth=2, color='blue', label=f'Overtake {overtake_num}')
    
    # Add horizontal red line at 0.1m
    ax.axhline(y=0.1, color='red', linestyle='--', linewidth=1.5, alpha=0.7)
    
    # Get min_dv2o from summary for this overtake
    min_dv2o = matching_rows['min_dv2o'].values[0]
    s_at_min = matching_rows['s_at_min'].values[0]
    
    # Mark minimum point
    ax.plot(s_at_min, min_dv2o, 'ro', markersize=8, 
            label=f'Min: {min_dv2o:.3f} m at s={s_at_min:.2f}')
    
    ax.set_xlabel('Track Progress s (m)', fontsize=12)
    ax.set_ylabel('Distance DV2O (m)', fontsize=12)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(alpha=0.3)
    
    plt.tight_layout()
    
    # Save individual plot
    filename = f'overtake_{overtake_num}_plot.pdf'
    plt.savefig(filename, format='pdf', bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()
    plots_saved += 1

print(f"\n{'='*50}")
print(f"Summary:")
print(f"  Box plot: overtake_boxplot.pdf")
print(f"  Detail plots saved: {plots_saved}")
print(f"  Detail plots skipped: {plots_skipped}")
print(f"{'='*50}")
