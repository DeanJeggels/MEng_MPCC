import pandas as pd

# List all overtake detail files
overtake_files = [
    "overtake_0_detail.csv",
    "overtake_1_detail.csv",
    "overtake_2_detail.csv",
    "overtake_3_detail.csv",
    "overtake_4_detail.csv",
    "overtake_5_detail.csv",
    "overtake_6_detail.csv",
    "overtake_7_detail.csv",
    "overtake_8_detail.csv",
    "overtake_9_detail.csv",
    "overtake_10_detail.csv",
    "overtake_11_detail.csv",
    "overtake_12_detail.csv",
    "overtake_13_detail.csv",
    "overtake_14_detail.csv",
]

results = []

# Process each file
for idx, file in enumerate(overtake_files):
    df = pd.read_csv(file)

    # Skip if key columns are missing
    if 'dv2o' not in df.columns or 's' not in df.columns:
        continue

    # Find row with minimum dv2o
    min_row = df.loc[df['dv2o'].idxmin()]
    min_dv2o = min_row['dv2o']
    s_at_min = min_row['s']
    num_samples = len(df)

    results.append({
        'overtake_index': idx,
        'min_dv2o': min_dv2o,
        's_at_min': s_at_min,
        'num_samples': num_samples
    })

# Save results to overtakes.csv in standard column format
result_df = pd.DataFrame(results)
result_df.to_csv('overtakes.csv', index=False)

print("Output written to overtakes.csv:")
print(result_df)