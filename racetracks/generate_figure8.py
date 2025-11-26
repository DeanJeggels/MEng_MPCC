import numpy as np
import pandas as pd

def generate_figure8_csv(radius=10.0, length_factor=0.4, num_points=1000, 
                         boundary_width=2.0, output_file='figure8.csv'):
    """
    Generate a Figure 8 (Lemniscate of Gerono) CSV starting at (0,0) 
    and progressing first in positive x and y direction (top left).
    
    Args:
        radius: Controls the size of the loops
        length_factor: Stretches the figure vertically
        num_points: Number of points on the curve
        boundary_width: Track boundary width (left_bound and right_bound)
        output_file: Name of the output CSV file
    """
    # Generate parameter t from 0 to 2*pi
    t = np.linspace(0, 2 * np.pi, num_points)
    
    # Parametric equations for the Lemniscate of Gerono
    x = radius * np.sin(t)
    y = radius * np.sin(t) * np.cos(t) * length_factor
    
    # Find the index where the curve is closest to (0, 0)
    distances = np.sqrt(x**2 + y**2)
    start_idx = np.argmin(distances)
    
    # Roll the arrays so that the closest point to origin is first
    x = np.roll(x, -start_idx)
    y = np.roll(y, -start_idx)
    
    # Ensure the first movement is in positive x and positive y
    if x[1] < x[0] or y[1] < y[0]:
        x = x[::-1]
        y = y[::-1]
    
    # Set the first point exactly to (0, 0)
    x = x - x[0]
    y = y - y[0]
    
    # Create boundary columns
    left_bound = np.ones(num_points) * boundary_width
    right_bound = np.ones(num_points) * boundary_width
    
    # Create DataFrame
    df = pd.DataFrame({
        'x': x,
        'y': y,
        'left_bound': left_bound,
        'right_bound': right_bound
    })
    
    # Save to CSV
    df.to_csv(output_file, index=False)
    
    # Calculate stats
    dx = np.diff(x)
    dy = np.diff(y)
    total_length = np.sum(np.sqrt(dx**2 + dy**2))
    
    print(f"✓ Generated {output_file}")
    print(f"  Radius: {radius}m, Length factor: {length_factor}")
    print(f"  Points: {num_points}, Track length: {total_length:.2f}m")
    print(f"  Starts at (0,0), moves toward (+x, +y)")
    
    return df

# Generate the CSV
generate_figure8_csv(radius=8.0, length_factor=0.5, num_points=1000, 
                     boundary_width=2.0, output_file='figure8.csv')
