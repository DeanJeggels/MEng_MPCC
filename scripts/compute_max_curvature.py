#!/usr/bin/env python3
import numpy as np
import pandas as pd
import math
import sys
from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('mpcc')
csv_path  = os.path.join(pkg_share, 'racetracks','corner_centreline.csv')
print(f"DEBUG: looking for CSV at {csv_path}", file=sys.stderr)
if not os.path.isfile(csv_path):
    print("ERROR: file not found!", file=sys.stderr)
    sys.exit(1)

class LinearSpline:
    def __init__(self):
        self.trajectory_ = None
        self.s_ = None

    def loadFromCSV(self, filename):
        df = pd.read_csv(filename)
        if 'x' not in df.columns or 'y' not in df.columns:
            return False
        self.trajectory_ = df[['x','y']].to_numpy()
        N = len(self.trajectory_)
        self.s_ = np.zeros(N)
        for i in range(1, N):
            dx = self.trajectory_[i,0] - self.trajectory_[i-1,0]
            dy = self.trajectory_[i,1] - self.trajectory_[i-1,1]
            self.s_[i] = self.s_[i-1] + math.hypot(dx, dy)
        return True

    def getDerivatives(self, s):
        if s <= self.s_[0]:
            i = 0
        elif s >= self.s_[-1]:
            i = len(self.s_)-2
        else:
            i = np.searchsorted(self.s_, s) - 1
        i = max(0, min(i, len(self.s_)-2))
        ds = self.s_[i+1] - self.s_[i]
        if ds < 1e-6:
            return {'dx_ds': 1.0, 'dy_ds': 0.0}
        A = self.trajectory_[i]; B = self.trajectory_[i+1]
        return {'dx_ds': (B[0]-A[0])/ds, 'dy_ds': (B[1]-A[1])/ds}

def compute_max_curvature(spline, ds_sample=0.01):
    s_max = spline.s_[-1]
    delta = ds_sample
    max_curv = 0.0
    for s in np.arange(0.0, s_max+ds_sample, ds_sample):
        D   = spline.getDerivatives(s)
        slo = max(0.0, s-delta); shi = min(s_max, s+delta)
        D_lo= spline.getDerivatives(slo); D_hi= spline.getDerivatives(shi)
        ddx = (D_hi['dx_ds']-D_lo['dx_ds'])/(shi-slo)
        ddy = (D_hi['dy_ds']-D_lo['dy_ds'])/(shi-slo)
        num = abs(D['dx_ds']*ddy - D['dy_ds']*ddx)
        den = (D['dx_ds']**2 + D['dy_ds']**2)**1.5
        kappa = num/den if den>1e-9 else 0.0
        max_curv = max(max_curv, kappa)
    return max_curv

def main():
    spline = LinearSpline()
    if not spline.loadFromCSV(csv_path):
        print("Spline load failed", file=sys.stderr)
        sys.exit(1)

    # 1) max curvature
    k_max = compute_max_curvature(spline)
    print(f"κ_max = {k_max:.6f} 1/m")

    # 2) vehicle params
    v = 10.0                       # speed in m/s (set your own)
    turn_rate_deg_s = 90.0         # deg/s
    omega_rad = math.radians(turn_rate_deg_s)
    # lateral accel
    a_lat_max = v * omega_rad      # [m/s^2] [1]
    print(f"a_lat,max = {a_lat_max:.3f} m/s² at ω={turn_rate_deg_s}°/s")

    # 3) lookahead distance
    if k_max>1e-9:
        s_min = v**2/(k_max * a_lat_max)
    else:
        s_min = float('inf')
    print(f"s_min = {s_min:.3f} m")

if __name__=="__main__":
    main()
