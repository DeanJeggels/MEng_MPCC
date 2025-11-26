#!/usr/bin/env python3

import math
import time
import pathlib
import os

import numpy as np
import pandas as pd
import pathlib
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path as NavPath, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import MarkerArray, Marker


Tph = 5
Tch = 5
iterations = 1000
Tnx = 4
Tnu = 3
rate_ = 10.0

class OptimalSequenceVisualizer(Node):
    def __init__(self):
        super().__init__('optimal_sequence_visualizer')

        # parameters & CSV filename
        self.declare_parameter(
            'csv_filename',
            f'ph{Tph}ch{Tch}it{iterations}hz{rate_}TRACKING.csv'
        )
        self.csv_filename = self.get_parameter('csv_filename').value

        # subscriptions
        # self.create_subscription(Odometry,
        #                          '/odom',
        #                          self.odom_callback,
        #                          10)
        self.create_subscription(Float64MultiArray,
                                 'mpcc/optimal_inputs',
                                 self.inputs_callback,
                                 10)
        self.create_subscription(NavPath,
                                 '/mpcc/reference_path',
                                 self.reference_callback,
                                 10)
        self.create_subscription(Float64,
                                 'mpcc/tracking_error',
                                 self.track_callback,
                                 10)
        self.create_subscription(Float64,
                                 'mpcc/opt_time',
                                 self.opt_time_callback,
                                 10)
        self.create_subscription(Odometry,
                                 '/mpcc/result_cmd',
                                 self.cmd_vel_callback,
                                 10)
        self.create_subscription(NavPath,
                                '/mpcc/actual_path',
                                self.odom_callback,
                                10)
        self.create_subscription(PointStamped,
                         '/mpcc/current_reference_point',
                         self.current_ref_callback,
                         10)
        
        self.create_subscription(MarkerArray,
                               '/mpcc/obstacles',
                               self.obstacles_callback,
                               10)


        # timing & buffers
        self.window       = 600.0
        self.start_time   = time.time()

        self.times        = []   # odom timestamp
        self.x_hist       = []   # record x position
        self.y_hist       = []   # record y position
        self.psi_hist     = []   # yaw history
        self.s_times      = []
        self.s_hist       = []   # progress history
        self.input_times  = []
        self.u_hist       = {'v': [], 'vs': [], 'omega': []}
        self.error_times  = []
        self.err_hist     = []
        self.reference_path = None
        self.actual_xy    = []
        self.opt_times    = []
        self.waypoint_idx = []
        self.reference_times = []
        self.reference_x = []
        self.reference_y = []
        self.current_reference_point = None  # Store current reference point
          # Add obstacle data storage
        self.obstacle_times = []
        self.obstacle_data = {}  # Dictionary to store each obstacle's position over time
        self.obstacle_count = 0  # Track number of obstacles
        self.obstacle_ids = set()  # Track unique obstacle IDs



        # set up plotting
        # plt.ion()

        # Figure 1: actual vs reference path
        self.fig_path, self.ax_path = plt.subplots(figsize=(8,6))
        self.ax_path.set_title('2D Path: actual vs reference')
        self.ax_path.set_xlabel('x (m)')
        self.ax_path.set_ylabel('y (m)')
        self.line_pred, = self.ax_path.plot([], [], 'b-', label='actual')
        self.line_ref,  = self.ax_path.plot([], [], 'r--', label='reference')
        self.ax_path.legend(loc='upper right')
        self.fig_path.tight_layout()

        # Figure 2: time series
        (self.fig_data,
         (self.ax_psi,
          self.ax_s,
          self.ax_u,
          self.ax_err)) = plt.subplots(4, 1, figsize=(10,12))

        # ψ vs time
        self.ax_psi.set_title('Heading ψ vs Time')
        self.ax_psi.set_xlabel('Time (s)')
        self.ax_psi.set_ylabel('ψ (rad)')
        self.ax_psi.set_xlim(0, self.window)
        self.ax_psi.set_ylim(-8, 8)
        self.line_psi, = self.ax_psi.plot([], [], 'm-')

        # s vs time
        self.ax_s.set_title('Progress s vs Time')
        self.ax_s.set_xlabel('Time (s)')
        self.ax_s.set_ylabel('s (m)')
        self.ax_s.set_xlim(0, self.window)
        self.ax_s.set_ylim(0, 100)
        self.line_s, = self.ax_s.plot([], [], 'g-')

        # control inputs vs time
        self.ax_u.set_title('Control Inputs vs Time')
        self.ax_u.set_xlabel('Time (s)')
        self.ax_u.set_ylabel('Linear Velocities (m/s)')
        self.ax_u.set_xlim(0, self.window)
        self.ax_u.set_ylim(-0.15, 0.15)
        self.line_v,    = self.ax_u.plot([], [], 'b-',  label='v (m/s)')
        self.line_vs,   = self.ax_u.plot([], [], 'k-.', label='vₛ (m/s)')
        self.ax_omega = self.ax_u.twinx()
        self.ax_omega.set_ylabel('Angular Velocity (rad/s)')
        self.ax_omega.set_ylim(-0.8, 0.8)
        self.line_omega, = self.ax_omega.plot([], [], 'r--', label='ω (°/s)')
        lines1, labs1 = self.ax_u.get_legend_handles_labels()
        lines2, labs2 = self.ax_omega.get_legend_handles_labels()
        self.ax_u.legend(lines1+lines2, labs1+labs2, loc='upper right')

        # tracking error vs time
        self.ax_err.set_title('Tracking Error vs Time')
        self.ax_err.set_xlabel('Time (s)')
        self.ax_err.set_ylabel('Error (m)')
        self.ax_err.set_xlim(0, self.window)
        self.ax_err.set_ylim(-0.5, 0.5)
        self.line_err, = self.ax_err.plot([], [], 'r-')

        self.fig_data.tight_layout()
        # self.create_timer(60.0, self.shutdown_callback)

    def odom_callback(self, msg: Odometry):
        t_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not hasattr(self, 't0'):
            self.t0 = t_ros
        t = t_ros - self.t0

        # Use Gazebo's ground truth pose
        pose = msg.poses[-1].pose
        x = pose.position.x
        y = pose.position.y
        
        # Convert quaternion to yaw for consistency
        qw = pose.orientation.w
        qz = pose.orientation.z
        yaw = 2.0 * math.atan2(qz, qw)
        
        self.times.append(t)
        self.x_hist.append(x)
        self.y_hist.append(y)
        self.psi_hist.append(yaw)
    
    # Rest of function unchanged...

            
        # update path
        self.actual_xy.append((x, y))
        xs, ys = zip(*self.actual_xy)
        self.line_pred.set_data(xs, ys)

        # trim to window
        cutoff = t - self.window
        idx = next((i for i,v in enumerate(self.times) if v>=cutoff), 0)
        self.times    = self.times[idx:]
        self.psi_hist = self.psi_hist[idx:]
        self.line_psi.set_data(self.times, self.psi_hist)
        self.ax_psi.set_xlim(max(0, t-self.window), t)

        # refresh canvases
        self.fig_data.canvas.draw()
        self.fig_data.canvas.flush_events()
        self.fig_path.canvas.draw()
        self.fig_path.canvas.flush_events()

    def current_ref_callback(self, msg: PointStamped):
        """Store current reference point that MPCC is targeting"""
        t = time.time() - self.start_time
        self.reference_times.append(t)
        self.reference_x.append(msg.point.x)
        self.reference_y.append(msg.point.y)
        self.current_reference_point = (msg.point.x, msg.point.y)
        
        # Trim to window
        cutoff = t - self.window
        idx = next((i for i,v in enumerate(self.reference_times) if v>=cutoff), 0)
        self.reference_times = self.reference_times[idx:]
        self.reference_x = self.reference_x[idx:]
        self.reference_y = self.reference_y[idx:]
        if hasattr(self, 'ref_point_plot'):
            self.ref_point_plot.remove()
        self.ref_point_plot = self.ax_path.scatter(msg.point.x, msg.point.y, 
                                                c='orange', s=50, marker='o', 
                                                label='Current Ref', zorder=5)
        self.fig_path.canvas.draw()
        self.fig_path.canvas.flush_events()

    def cmd_vel_callback(self, msg: Odometry):
        t = time.time() - self.start_time
        s = msg.twist.twist.angular.x
        self.s_hist.append(s)
        self.s_times.append(t)
        cutoff = t - self.window
        idx = next((i for i,v in enumerate(self.s_times) if v>=cutoff), 0)
        self.s_hist = self.s_hist[idx:]
        self.s_times = self.s_times[idx:]
        self.line_s.set_data(self.s_times, self.s_hist)
        self.ax_s.set_xlim(max(0, t-self.window), t)
        self.fig_data.canvas.draw()
        self.fig_data.canvas.flush_events()

    def inputs_callback(self, msg: Float64MultiArray):
        data = np.array(msg.data).reshape((Tnu, Tph+1)).T
        t = time.time() - self.start_time

        # Print out whole data array    
        # print("Data array:")
        # print(data)


        self.input_times.append(t)
        self.u_hist['v'].append(data[0,0])
        self.u_hist['omega'].append(data[0,1])
        self.u_hist['vs'].append(data[0,2])

        cutoff = t - self.window
        idx = next((i for i,v in enumerate(self.input_times) if v>=cutoff), 0)
        self.input_times = self.input_times[idx:]
        for k in self.u_hist:
            self.u_hist[k] = self.u_hist[k][idx:]

        self.line_v.set_data(self.input_times, self.u_hist['v'])
        self.line_vs.set_data(self.input_times, self.u_hist['vs'])
        self.ax_u.set_xlim(max(0, t-self.window), t)
        self.line_omega.set_data(self.input_times, self.u_hist['omega'])
        self.ax_omega.set_xlim(max(0, t-self.window), t)

        self.fig_data.canvas.draw()
        self.fig_data.canvas.flush_events()
    def obstacles_callback(self, msg: MarkerArray):
        """Capture obstacle positions from MPCC controller"""
        t = time.time() - self.start_time
        self.obstacle_times.append(t)
        
        # Parse obstacles from marker array
        current_obstacles = {}
        
        for marker in msg.markers:
            # Only process physical obstacle markers (not safety zones)
            if marker.ns == "obstacles_physical":
                obstacle_id = f"obs_{marker.id}"
                self.obstacle_ids.add(obstacle_id)
                
                current_obstacles[obstacle_id] = {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z,
                    'radius': marker.scale.x / 2.0  # Convert diameter to radius
                }
        
        # Store obstacle data for this timestamp
        for obs_id in self.obstacle_ids:
            if obs_id not in self.obstacle_data:
                self.obstacle_data[obs_id] = {
                    'times': [],
                    'x': [],
                    'y': [],
                    'z': [],
                    'radius': []
                }
            
            if obs_id in current_obstacles:
                # Obstacle is present in this frame
                obs = current_obstacles[obs_id]
                self.obstacle_data[obs_id]['times'].append(t)
                self.obstacle_data[obs_id]['x'].append(obs['x'])
                self.obstacle_data[obs_id]['y'].append(obs['y'])
                self.obstacle_data[obs_id]['z'].append(obs['z'])
                self.obstacle_data[obs_id]['radius'].append(obs['radius'])
            else:
                # Obstacle not present, fill with previous value or NaN
                self.obstacle_data[obs_id]['times'].append(t)
                if len(self.obstacle_data[obs_id]['x']) > 0:
                    # Use last known position
                    self.obstacle_data[obs_id]['x'].append(self.obstacle_data[obs_id]['x'][-1])
                    self.obstacle_data[obs_id]['y'].append(self.obstacle_data[obs_id]['y'][-1])
                    self.obstacle_data[obs_id]['z'].append(self.obstacle_data[obs_id]['z'][-1])
                    self.obstacle_data[obs_id]['radius'].append(self.obstacle_data[obs_id]['radius'][-1])
                else:
                    # No previous data, use NaN
                    self.obstacle_data[obs_id]['x'].append(float('nan'))
                    self.obstacle_data[obs_id]['y'].append(float('nan'))
                    self.obstacle_data[obs_id]['z'].append(float('nan'))
                    self.obstacle_data[obs_id]['radius'].append(float('nan'))
        
        # Trim obstacle data to window
        cutoff = t - self.window
        for obs_id in self.obstacle_data:
            # Find cutoff index
            idx = next((i for i, v in enumerate(self.obstacle_data[obs_id]['times']) if v >= cutoff), 0)
            
            # Trim all arrays
            self.obstacle_data[obs_id]['times'] = self.obstacle_data[obs_id]['times'][idx:]
            self.obstacle_data[obs_id]['x'] = self.obstacle_data[obs_id]['x'][idx:]
            self.obstacle_data[obs_id]['y'] = self.obstacle_data[obs_id]['y'][idx:]
            self.obstacle_data[obs_id]['z'] = self.obstacle_data[obs_id]['z'][idx:]
            self.obstacle_data[obs_id]['radius'] = self.obstacle_data[obs_id]['radius'][idx:]


    def reference_callback(self, msg: NavPath):
        pts = np.array([[p.pose.position.x, p.pose.position.y]
                        for p in msg.poses])
        if pts.size:
            self.reference_path = pts
            self.line_ref.set_data(pts[:,0], pts[:,1])
            pad_x = 0.1 * pts[:,0].ptp()
            pad_y = 0.1 * pts[:,1].ptp()
            xmin, xmax = pts[:,0].min()-pad_x, pts[:,0].max()+pad_x
            ymin, ymax = pts[:,1].min()-pad_y, pts[:,1].max()+pad_y
            self.ax_path.set_xlim(xmin, xmax)
            self.ax_path.set_ylim(ymin, ymax)
            self.ax_path.set_aspect('equal','box')
            self.ax_path.set_autoscalex_on(False)
            self.ax_path.set_autoscaley_on(False)
            self.fig_path.canvas.draw()
            self.fig_path.canvas.flush_events()

    def track_callback(self, msg: Float64):
        t = time.time() - self.start_time
        self.error_times.append(t)
        self.err_hist.append(msg.data)
        cutoff = t - self.window
        idx = next((i for i,v in enumerate(self.error_times) if v>=cutoff), 0)
        self.error_times = self.error_times[idx:]
        self.err_hist     = self.err_hist[idx:]

        
        self.line_err.set_data(self.error_times, self.err_hist)
        self.ax_err.set_xlim(max(0, t-self.window), t)
        self.fig_data.canvas.draw()
        self.fig_data.canvas.flush_events()

    def opt_time_callback(self, msg: Float64):
        self.opt_times.append(msg.data)

    def shutdown_callback(self):
        self.get_logger().info("300 seconds elapsed – shutting down")
        self.main_cleanup()
        rclpy.shutdown()

    def main_cleanup(self):
        # 1) Path waypoints DataFrame
        df_path = pd.DataFrame({
            'time':      self.times,
            'x (m)':     self.x_hist,
            'y (m)':     self.y_hist,
            'psi (rad)': self.psi_hist,
        })

        df_ref = pd.DataFrame({
            'time':        self.reference_times,
            'x_ref (m)':   self.reference_x,
            'y_ref (m)':   self.reference_y,
        })
        
        df_s = pd.DataFrame({
            'time': self.s_times,
            's (m)': self.s_hist,
        })

        # 2) Inputs DataFrame
        df_inputs = pd.DataFrame({
            'time':           self.input_times,
            'v (m/s)':        self.u_hist['v'],
            'vs (m/s)':       self.u_hist['vs'],
            'omega (rad/s)':  self.u_hist['omega'],
        })

        # 3) Tracking error DataFrame
        df_err = pd.DataFrame({
            'time':           self.error_times,
            'tracking_error': self.err_hist,
        })

        # 4) Opt-time DataFrame
        df_opt = pd.DataFrame({
            'time':     self.times[:len(self.opt_times)],
            'opt_time': self.opt_times,
        })

        # 5) Obstacles DataFrame - NEW!
        df_obstacles_list = []
        for obs_id in sorted(self.obstacle_ids):
            if obs_id in self.obstacle_data and len(self.obstacle_data[obs_id]['times']) > 0:
                df_obs = pd.DataFrame({
                    'time': self.obstacle_data[obs_id]['times'],
                    f'{obs_id}_x (m)': self.obstacle_data[obs_id]['x'],
                    f'{obs_id}_y (m)': self.obstacle_data[obs_id]['y'],
                    f'{obs_id}_z (m)': self.obstacle_data[obs_id]['z'],
                    f'{obs_id}_radius (m)': self.obstacle_data[obs_id]['radius'],
                })
                df_obstacles_list.append(df_obs)

        # 6) Merge all DataFrames
        df = df_path.copy()
        
        # Merge other dataframes
        for df_to_merge in [df_ref, df_s, df_inputs, df_err, df_opt] + df_obstacles_list:
            if not df_to_merge.empty:
                df = df.merge(df_to_merge, on='time', how='outer')

        # 7) Sort, interpolate, then forward-fill
        df = df.sort_values('time').reset_index(drop=True)
        df = df.interpolate(method='index').ffill()

        # 8) Build output filename
        base = pathlib.Path(self.csv_filename)
        timestamp = int(self.start_time)
        out_name = f"{base.stem}_{timestamp}{base.suffix}"
        out_path = base.with_name(out_name)

        # 9) Save CSV with obstacle data
        df.to_csv(out_path, index=False)
        
        # Log summary including obstacle information
        obstacle_summary = []
        for obs_id in sorted(self.obstacle_ids):
            if obs_id in self.obstacle_data and len(self.obstacle_data[obs_id]['x']) > 0:
                avg_x = pd.Series(self.obstacle_data[obs_id]['x']).mean()
                avg_y = pd.Series(self.obstacle_data[obs_id]['y']).mean()
                avg_r = pd.Series(self.obstacle_data[obs_id]['radius']).mean()
                obstacle_summary.append(f"{obs_id}: pos({avg_x:.3f},{avg_y:.3f}), r={avg_r:.3f}")
        
        self.get_logger().info(
            f"Wrote merged dataset with {len(df)} rows to {out_path}\n"
            f"Included {len(self.obstacle_ids)} obstacles: {', '.join(obstacle_summary)}"
        )




def main(args=None):
    rclpy.init(args=args)
    node = OptimalSequenceVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.main_cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

