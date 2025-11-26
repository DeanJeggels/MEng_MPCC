#!/usr/bin/env python3
import subprocess, time, shutil, pathlib

# 1) parameter ranges
ph_range   = range(1,16)
ch_map     = lambda ph: range(1, ph+1)
iter_list  = [20, 40, 60, 80, 100, 120, 140]

# path to where trajectory_viz writes its CSV (if still using archive step)
out_dir = pathlib.Path.home()/"new_ros2_ws"/"src"/"mpcc"
out_file = out_dir/"mpcc_opt.csv"

for ph in ph_range:
    for ch in ch_map(ph):
        for it in iter_list:
            tag = f"ph{ph}_ch{ch}_it{it}"
            print(f"=== Running {tag} ===")

            # launch karretjie node
            proc_k = subprocess.Popen([
                "ros2", "run", "mpcc", "karretjie_opt"
            ])

            # launch mpcc_controller with parameters
            proc_c = subprocess.Popen([
                "ros2", "run", "mpcc", "mpcc_controller_opt",
                "--ros-args",
                "-p", f"prediction_horizon:={ph}",
                "-p", f"control_horizon:={ch}",
                "-p", f"max_iterations:={it}"
            ])

            # launch trajectory_viz with a unique csv_filename
            proc_v = subprocess.Popen([
                "ros2", "run", "mpcc", "trajectory_viz.py",
                "--ros-args",
                "-p", f"csv_filename:=mpcc_opt_{tag}.csv"
            ])

            # let them run for 60 seconds
            time.sleep(60)

            # tear down all three
            for p in (proc_k, proc_c, proc_v):
                p.terminate()
            for p in (proc_k, proc_c, proc_v):
                p.wait()

            # optional archive if your viz still writes to a fixed name
            if out_file.exists():
                dst = out_dir/f"mpcc_opt_{tag}.csv"
                shutil.move(str(out_file), str(dst))
                print(f"-> saved results to {dst}")
            else:
                print("! Warning: no CSV produced")

print("Sweep complete.")
