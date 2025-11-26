#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

# Set matplotlib for professional output
plt.rcParams.update({
    'font.size': 14,
    'font.family': 'serif', 
    'text.usetex': False,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.2
})

def create_horizon_specific_analysis():
    """Create separate analysis for each horizon configuration"""

    # Real-time constraint parameters for 5Hz control
    TARGET_FREQUENCY = 5.0  # Hz
    RT_CONSTRAINT_MS = 1000.0 / TARGET_FREQUENCY  # 100ms for 10Hz
    
    # Define the configurations
    configurations = [
        {'file': 'solver_benchmark_resultsph3ch3.csv', 'ph': 3, 'ch': 3, 'name': 'PH3-CH3'},
        {'file': 'solver_benchmark_resultsph8ch3.csv', 'ph': 8, 'ch': 3, 'name': 'PH8-CH3'},
        {'file': 'solver_benchmark_resultsph8ch8.csv', 'ph': 8, 'ch': 8, 'name': 'PH8-CH8'},
        {'file': 'solver_benchmark_resultsph15ch15.csv', 'ph': 15, 'ch': 15, 'name': 'PH15-CH15'}
    ]
    
    all_stats = []
    raw_stats = []  # Store raw numerical values for plotting
    
    for config in configurations:
        print(f"\n{'='*60}")
        print(f"Processing {config['name']} Configuration")
        print(f"Prediction Horizon: {config['ph']}, Control Horizon: {config['ch']}")
        print(f"{'='*60}")
        
        # Try to load the file
        csv_path = Path(config['file']).expanduser()
        
        try:
            # Load data
            df = pd.read_csv(csv_path)
            solve_times = df['solve_time_ms'].values
            
            print(f"Loaded {len(df)} benchmark results")
            
            # Calculate statistics
            mean_val = np.mean(solve_times)
            median_val = np.median(solve_times)
            std_val = np.std(solve_times)
            min_val = np.min(solve_times)
            max_val = np.max(solve_times)
            q1 = np.percentile(solve_times, 25)
            q3 = np.percentile(solve_times, 75)
            iqr = q3 - q1
            p95 = np.percentile(solve_times, 95)
            p99 = np.percentile(solve_times, 99)
            
            # Handle convergence data
            converged_data = df['converged'].values if 'converged' in df.columns else np.ones(len(df))
            convergence_rate = np.mean(converged_data) * 100
            
            # Real-time feasibility for 10Hz control
            rt_feasible_10hz = np.sum(solve_times <= RT_CONSTRAINT_MS) / len(solve_times) * 100
            rt_feasible_50ms = np.sum(solve_times <= 50) / len(solve_times) * 100  # Even more aggressive
            rt_count_10hz = np.sum(solve_times <= RT_CONSTRAINT_MS)
            rt_count_50ms = np.sum(solve_times <= 50)
            
            print(f"Mean: {mean_val:.3f} ms")
            print(f"Median: {median_val:.3f} ms") 
            print(f"Std Dev: {std_val:.3f} ms")
            print(f"Convergence: {convergence_rate:.1f}%")
            print(f"5Hz Feasible (≤{RT_CONSTRAINT_MS:.0f}ms): {rt_feasible_10hz:.1f}% ({rt_count_10hz}/{len(solve_times)})")
            
            # Store raw numerical values for plotting
            raw_stats.append({
                'name': config['name'],
                'mean': mean_val,
                'median': median_val,
                'std': std_val,
                'convergence': convergence_rate,
                'rt_10hz': rt_feasible_10hz,
                'rt_50ms': rt_feasible_50ms
            })
            
            # Create individual box plot
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            
            bp = ax.boxplot([solve_times], 
                           labels=[f'{config["name"]}'], 
                           patch_artist=True, 
                           notch=True, 
                           showmeans=True,
                           meanline=False,
                           showfliers=True,
                           whis=1.5)
            
            # Customize appearance based on configuration
            colors = {'PH3-CH3': 'lightblue', 'PH8-CH3': 'lightgreen', 
                     'PH8-CH8': 'lightcoral', 'PH15-CH15': 'lightyellow'}
            edge_colors = {'PH3-CH3': 'navy', 'PH8-CH3': 'darkgreen', 
                          'PH8-CH8': 'darkred', 'PH15-CH15': 'darkorange'}
            
            bp['boxes'][0].set_facecolor(colors[config['name']])
            bp['boxes'][0].set_alpha(0.8)
            bp['boxes'][0].set_edgecolor(edge_colors[config['name']])
            bp['boxes'][0].set_linewidth(2)
            
            # Median line
            bp['medians'][0].set_color('darkred')
            bp['medians'][0].set_linewidth(3)
            
            # Mean marker
            bp['means'][0].set_marker('D')
            bp['means'][0].set_markerfacecolor('darkgreen')
            bp['means'][0].set_markeredgecolor('darkgreen')
            bp['means'][0].set_markersize(8)
            
            # Whiskers and caps
            for whisker in bp['whiskers']:
                whisker.set_color(edge_colors[config['name']])
                whisker.set_linewidth(2)
            for cap in bp['caps']:
                cap.set_color(edge_colors[config['name']])
                cap.set_linewidth(2)
                cap.set_markersize(8)
            
            # Outliers
            for flier in bp['fliers']:
                flier.set_marker('o')
                flier.set_markerfacecolor('red')
                flier.set_markeredgecolor('red')
                flier.set_alpha(0.6)
                flier.set_markersize(3)
            
            # ADD RED LINE AT 100ms FOR 10Hz CONSTRAINT
            ax.axhline(y=RT_CONSTRAINT_MS, color='red', linestyle='-', linewidth=3, 
                      label=f'5Hz Real-time Constraint ({RT_CONSTRAINT_MS:.0f}ms)', alpha=0.8)
            
            # Formatting
            ax.set_ylabel('Solve Time (ms)', fontweight='bold', fontsize=16)
            ax.set_title(f'MPCC Solver Performance - 5Hz Real-time Analysis\nPrediction Horizon = {config["ph"]}, Control Horizon = {config["ch"]}', 
                        fontweight='bold', fontsize=18, pad=20)
            ax.grid(True, alpha=0.3, linestyle='--', axis='y')
            ax.set_facecolor('white')
            
            # REMOVED: Statistics text box - NO LONGER INCLUDED
            
            # Create legend with 10Hz constraint line
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], color='darkred', lw=3, label='Median'),
                Line2D([0], [0], marker='D', color='w', markerfacecolor='darkgreen', 
                       markersize=8, label='Mean'),
                Line2D([0], [0], color='red', lw=3, label=f'5Hz Limit ({RT_CONSTRAINT_MS:.0f}ms)'),
                plt.Rectangle((0, 0), 1, 1, facecolor=colors[config['name']], 
                             edgecolor=edge_colors[config['name']], alpha=0.8, label='IQR (Q1-Q3)'),
                Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                       markersize=4, alpha=0.6, label='Outliers')
            ]
            ax.legend(handles=legend_elements, loc='upper right', fontsize=11)
            
            plt.tight_layout()
            
            # Save individual plots
            box_plot_file = f'mpcc_solver_boxplot_10hz_{config["name"].lower()}.pdf'
            plt.savefig(box_plot_file, format='pdf', dpi=300, bbox_inches='tight')
            print(f"Box plot saved to: {box_plot_file}")
            
            # Also save as PNG
            png_file = f'mpcc_solver_boxplot_10hz_{config["name"].lower()}.png'
            plt.savefig(png_file, format='png', dpi=300, bbox_inches='tight')
            print(f"Box plot PNG saved to: {png_file}")
            
            plt.show()
            plt.close()
            
            # Store statistics for comparison table (formatted strings)
            config_stats = {
                'Configuration': f'{config["name"]}',
                'Prediction Horizon': config['ph'],
                'Control Horizon': config['ch'],
                'Sample Count': len(solve_times),
                'Mean (ms)': f"{mean_val:.3f}",
                'Median (ms)': f"{median_val:.3f}",
                'Std Dev (ms)': f"{std_val:.3f}",
                'Min (ms)': f"{min_val:.3f}",
                'Max (ms)': f"{max_val:.3f}",
                'Q1 (ms)': f"{q1:.3f}",
                'Q3 (ms)': f"{q3:.3f}",
                'IQR (ms)': f"{iqr:.3f}",
                'P95 (ms)': f"{p95:.3f}",
                'P99 (ms)': f"{p99:.3f}",
                'Coefficient of Variation (%)': f"{(std_val/mean_val)*100:.1f}",
                'Convergence Rate (%)': f"{convergence_rate:.1f}",
                '5Hz Feasible Count (≤100ms)': f"{rt_count_10hz}",
                '5Hz Feasible Rate (%)': f"{rt_feasible_10hz:.1f}",
                '50ms Feasible Count': f"{rt_count_50ms}",
                '50ms Feasible Rate (%)': f"{rt_feasible_50ms:.1f}",
                '5Hz Suitability': ("EXCELLENT" if rt_feasible_10hz >= 95 else "GOOD" if rt_feasible_10hz >= 80 else "MARGINAL" if rt_feasible_10hz >= 50 else "POOR")
            }
            all_stats.append(config_stats)
            
            # Create individual statistics file
            individual_stats_data = {
                'Statistic': [
                    'Configuration',
                    'Prediction Horizon', 
                    'Control Horizon',
                    'Sample Count',
                    'Mean (ms)',
                    'Median (ms)',
                    'Standard Deviation (ms)',
                    'Minimum (ms)',
                    'Maximum (ms)',
                    'Q1 (25th Percentile) (ms)',
                    'Q3 (75th Percentile) (ms)',
                    'Interquartile Range (IQR) (ms)',
                    '95th Percentile (ms)',
                    '99th Percentile (ms)',
                    'Range (ms)',
                    'Coefficient of Variation (%)',
                    'Convergence Rate (%)',
                    '5Hz Target Frequency (Hz)',
                    '5Hz Constraint (ms)',
                    '5Hz Feasible Count (≤100ms)',
                    '5Hz Feasible Rate (%)',
                    '50ms Feasible Count',
                    '50ms Feasible Rate (%)',
                    '5Hz Suitability Assessment'
                ],
                'Value': [
                    f'{config["name"]}',
                    f'{config["ph"]}',
                    f'{config["ch"]}',
                    f'{len(solve_times)}',
                    f'{mean_val:.3f}',
                    f'{median_val:.3f}',
                    f'{std_val:.3f}',
                    f'{min_val:.3f}',
                    f'{max_val:.3f}',
                    f'{q1:.3f}',
                    f'{q3:.3f}',
                    f'{iqr:.3f}',
                    f'{p95:.3f}',
                    f'{p99:.3f}',
                    f'{max_val - min_val:.3f}',
                    f'{(std_val/mean_val)*100:.1f}',
                    f'{convergence_rate:.1f}',
                    f'{TARGET_FREQUENCY:.1f}',
                    f'{RT_CONSTRAINT_MS:.1f}',
                    f'{rt_count_10hz}',
                    f'{rt_feasible_10hz:.1f}',
                    f'{rt_count_50ms}',
                    f'{rt_feasible_50ms:.1f}',
                    ("EXCELLENT" if rt_feasible_10hz >= 95 else "GOOD" if rt_feasible_10hz >= 80 else "MARGINAL" if rt_feasible_10hz >= 50 else "POOR")
                ]
            }
            
            # Save individual statistics
            individual_df = pd.DataFrame(individual_stats_data)
            stats_csv = f'mpcc_solver_statistics_10hz_{config["name"].lower()}.csv'
            individual_df.to_csv(stats_csv, index=False)
            print(f"Individual statistics saved to: {stats_csv}")
            
        except FileNotFoundError:
            print(f"Error: Could not find file {csv_path}")
            continue
        except Exception as e:
            print(f"Error processing {config['name']}: {e}")
            import traceback
            traceback.print_exc()
            continue
    
    # Create comparison table
    if all_stats:
        print(f"\n{'='*80}")
        print("CREATING COMPREHENSIVE 5Hz COMPARISON TABLE")
        print(f"{'='*80}")
        
        comparison_df = pd.DataFrame(all_stats)
        comparison_csv = 'mpcc_solver_comparison_10hz_all_horizons.csv'
        comparison_df.to_csv(comparison_csv, index=False)
        print(f"10Hz comparison table saved to: {comparison_csv}")
        
        # Create a summary comparison plot using raw numerical values
        if raw_stats:
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle('MPCC Solver Performance Comparison for 5Hz Real-time Control\nAcross Different Horizon Configurations', 
                        fontsize=18, fontweight='bold', y=0.98)
            
            configs = [stat['name'] for stat in raw_stats]
            means = [stat['mean'] for stat in raw_stats]
            medians = [stat['median'] for stat in raw_stats]
            stds = [stat['std'] for stat in raw_stats]
            rt_10hz_rates = [stat['rt_10hz'] for stat in raw_stats]
            
            colors = ['lightblue', 'lightgreen', 'lightcoral', 'lightyellow']
            
            # Mean comparison with 100ms line
            bars1 = ax1.bar(configs, means, color=colors, alpha=0.8, edgecolor='black')
            ax1.axhline(y=RT_CONSTRAINT_MS, color='red', linestyle='-', linewidth=2, 
                       label=f'5Hz Constraint ({RT_CONSTRAINT_MS:.0f}ms)', alpha=0.8)
            ax1.set_ylabel('Mean Solve Time (ms)', fontweight='bold')
            ax1.set_title('Mean Solve Time vs 5Hz Constraint', fontweight='bold')
            ax1.grid(True, alpha=0.3, axis='y')
            ax1.tick_params(axis='x', rotation=45)
            ax1.legend()
            for bar, mean in zip(bars1, means):
                color = 'green' if mean <= RT_CONSTRAINT_MS else 'red'
                ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(means)*0.01,
                        f'{mean:.0f}', ha='center', va='bottom', fontweight='bold', color=color)
            
            # Median comparison with 100ms line
            bars2 = ax2.bar(configs, medians, color=colors, alpha=0.8, edgecolor='black')
            ax2.axhline(y=RT_CONSTRAINT_MS, color='red', linestyle='-', linewidth=2, 
                       label=f'5Hz Constraint ({RT_CONSTRAINT_MS:.0f}ms)', alpha=0.8)
            ax2.set_ylabel('Median Solve Time (ms)', fontweight='bold')
            ax2.set_title('Median Solve Time vs 5Hz Constraint', fontweight='bold')
            ax2.grid(True, alpha=0.3, axis='y')
            ax2.tick_params(axis='x', rotation=45)
            ax2.legend()
            for bar, median in zip(bars2, medians):
                color = 'green' if median <= RT_CONSTRAINT_MS else 'red'
                ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(medians)*0.01,
                        f'{median:.0f}', ha='center', va='bottom', fontweight='bold', color=color)
            
            # Standard deviation comparison
            bars3 = ax3.bar(configs, stds, color=colors, alpha=0.8, edgecolor='black')
            ax3.set_ylabel('Standard Deviation (ms)', fontweight='bold')
            ax3.set_title('Solve Time Variability', fontweight='bold')
            ax3.grid(True, alpha=0.3, axis='y')
            ax3.tick_params(axis='x', rotation=45)
            for bar, std in zip(bars3, stds):
                ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(stds)*0.01,
                        f'{std:.0f}', ha='center', va='bottom', fontweight='bold')
            
            # 10Hz feasibility comparison - CLEANED UP VERSION
            bars4 = ax4.bar(configs, rt_10hz_rates, color=colors, alpha=0.8, edgecolor='black')
            ax4.set_ylabel('5Hz Feasible Rate (%)', fontweight='bold')
            ax4.set_title('5Hz Real-time Performance (≤100ms)', fontweight='bold')
            ax4.set_ylim(0, 100)
            ax4.grid(True, alpha=0.3, axis='y')
            ax4.tick_params(axis='x', rotation=45)
            # REMOVED: Legend and excellent/good/marginal threshold lines
            for bar, rt_rate in zip(bars4, rt_10hz_rates):
                ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
                        f'{rt_rate:.1f}%', ha='center', va='bottom', fontweight='bold')
            
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            
            # Save comparison plot
            comparison_plot = 'mpcc_solver_10hz_horizon_comparison.pdf'
            plt.savefig(comparison_plot, format='pdf', dpi=300, bbox_inches='tight')
            print(f"10Hz comparison plot saved to: {comparison_plot}")
            
            comparison_png = 'mpcc_solver_10hz_horizon_comparison.png'
            plt.savefig(comparison_png, format='png', dpi=300, bbox_inches='tight')
            print(f"10Hz comparison plot PNG saved to: {comparison_png}")
            
            plt.show()
            plt.close()
        
        # Print summary with 10Hz analysis
        print(f"\n{'='*80}")
        print("10Hz REAL-TIME BENCHMARK ANALYSIS SUMMARY")
        print(f"{'='*80}")
        print(f"Target Control Frequency: {TARGET_FREQUENCY} Hz")
        print(f"Real-time Constraint: ≤ {RT_CONSTRAINT_MS:.0f} ms per solve")
        print(f"{'='*80}")
        
        for stat in all_stats:
            rt_rate = float(stat['5Hz Feasible Rate (%)'])
            status_emoji = "✅" if rt_rate >= 80 else "⚠️" if rt_rate >= 50 else "❌"
            
            print(f"\n{stat['Configuration']} (PH={stat['Prediction Horizon']}, CH={stat['Control Horizon']}) {status_emoji}:")
            print(f"  Mean: {stat['Mean (ms)']} ms")
            print(f"  Median: {stat['Median (ms)']} ms") 
            print(f"  Std Dev: {stat['Std Dev (ms)']} ms")
            print(f"  Convergence: {stat['Convergence Rate (%)']}")
            print(f"  5Hz Feasible: {stat['5Hz Feasible Count (≤100ms)']}/{stat['Sample Count']} ({stat['5Hz Feasible Rate (%)']}%)")
            print(f"  Assessment: {stat['10Hz Suitability']}")
        
        print(f"\n{'='*80}")
        print("RECOMMENDATIONS FOR 10Hz CONTROL:")
        
        best_config = None
        best_rate = 0
        for stat in all_stats:
            rate = float(stat['10Hz Feasible Rate (%)'])
            if rate > best_rate:
                best_rate = rate
                best_config = stat
        
        if best_config:
            print(f"• RECOMMENDED: {best_config['Configuration']} with {best_config['10Hz Feasible Rate (%)']}% feasibility")
        
        print("• PH3-CH3: Best for consistent 10Hz performance")
        print("• PH8-CH3: May require adaptive frequency control")
        print("• PH8-CH8: Not suitable for strict 10Hz requirements") 
        print("• PH15-CH15: Only suitable for slower control frequencies")
        print(f"{'='*80}")
        
    return len(all_stats)

if __name__ == "__main__":
    print("MPCC Solver 10Hz Real-time Analysis")
    print("="*60)
    
    num_processed = create_horizon_specific_analysis()
    
    if num_processed > 0:
        print(f"\n✅ Successfully processed {num_processed} horizon configurations for 10Hz analysis")
        print("\nGenerated Files:")
        print("- Individual box plots with 100ms red line (PDF and PNG)")
        print("- Individual 10Hz statistics tables (CSV)")
        print("- Comprehensive 10Hz comparison table (CSV)")
        print("- 10Hz horizon comparison plot (PDF and PNG)")
    else:
        print("\n❌ No configurations were successfully processed")
        print("Please check that the CSV files exist in the current directory")
