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


Tph = 10
Tch = 5
iterations = 1000
Tnx = 4
Tnu = 3
rate_ =2.0
t = 0.0  # Initialize t globally



class OptimalSequenceVisualizerKarretjie(Node):
    def __init__(self):
        super().__init__('optimal_sequence_visualizer_karretjie')

        # parameters & CSV filename
        self.declare_parameter(
            'csv_filename',
            f'ph{Tph}ch{Tch}{rate_}hz_TRACKINGKARRETJIE.csv'
        )
        self.csv_filename = self.get_parameter('csv_filename').value

        # subscriptions
        self.create_subscription(Odometry,
                                 'karretjie/odom',
                                 self.odom_callback,
                                 10)
        self.create_subscription(Float64MultiArray,
                                 'mpcc/optimal_inputs_karretjie',
                                 self.inputs_callback,
                                 10)
        self.create_subscription(NavPath,
                                 '/mpcc/reference_path_karretjie',
                                 self.reference_callback,
                                 10)
        self.create_subscription(Float64,
                                 'mpcc/tracking_error_karretjie',
                                 self.track_callback,
                                 10)
        self.create_subscription(Float64,
                                 'mpcc/opt_time_karretjie',
                                 self.opt_time_callback,
                                 10)
        self.create_subscription(Odometry,
                                 '/mpcc/result_cmd_karretjie',
                                 self.cmd_vel_callback,
                                 10)
        # self.create_subscription(NavPath,
        #                         '/mpcc/actual_path_karretjie',
        #                         self.odom_callback,
        #                         10)

        self.create_subscription(PointStamped,
                 '/mpcc/current_reference_point_karretjie',  # Note _karretjie suffix
                 self.current_ref_callback,
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
        self.create_timer(300.0, self.shutdown_callback)

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
    
    # Since live plotting is disabled, we don't draw the reference point
    # but the data is still collected for CSV export


    def odom_callback(self, msg: Odometry):

            # Initialize time counter on first call
        if not hasattr(self, 't'):
            self.t = 0.0
        else:
            self.t += 0.1  # Increment by 0.1 seconds each callback

        # Fix: Access single pose from Odometry message, not last pose from array
        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y
        yaw = pose.orientation.z
        
        self.times.append(self.t)
        self.x_hist.append(x)
        self.y_hist.append(y)
        self.psi_hist.append(yaw)
            
        # update path
        self.actual_xy.append((x, y))
        xs, ys = zip(*self.actual_xy)
        self.line_pred.set_data(xs, ys)

        # update ψ history
        

        # trim to window
        cutoff = self.t - self.window
        idx = next((i for i,v in enumerate(self.times) if v>=cutoff), 0)
        self.times    = self.times[idx:]
        self.psi_hist = self.psi_hist[idx:]
        self.line_psi.set_data(self.times, self.psi_hist)
        self.ax_psi.set_xlim(max(0, self.t-self.window), self.t)

        # refresh canvases
        # self.fig_data.canvas.draw()
        # self.fig_data.canvas.flush_events()
        # self.fig_path.canvas.draw()
        # self.fig_path.canvas.flush_events()


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
        # self.fig_data.canvas.draw()
        # self.fig_data.canvas.flush_events()

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

        # self.fig_data.canvas.draw()
        # self.fig_data.canvas.flush_events()

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
            # self.fig_path.canvas.draw()
            # self.fig_path.canvas.flush_events()

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
        # self.fig_data.canvas.draw()
        # self.fig_data.canvas.flush_events()

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
        
        # Add reference DataFrame
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

        # 4) Opt-time DataFrame (align length to self.times if needed)
        df_opt = pd.DataFrame({
            'time':     self.times[:len(self.opt_times)],
            'opt_time': self.opt_times,
        })

        # 5) Merge all on 'time' with outer joins - include reference data
        df = (
            df_path
            .merge(df_ref,    on='time', how='outer')  # Add reference merge
            .merge(df_s,      on='time', how='outer')
            .merge(df_inputs, on='time', how='outer')
            .merge(df_err,    on='time', how='outer')
            .merge(df_opt,    on='time', how='outer')
        )

        # 6) Sort, interpolate, then forward-fill
        df = df.sort_values('time').reset_index(drop=True)
        df = df.interpolate(method='index').ffill()

        # 7) Build output filename
        base = pathlib.Path(self.csv_filename)
        timestamp = int(self.start_time)
        out_name = f"{base.stem}_{timestamp}{base.suffix}"
        out_path = base.with_name(out_name)

        # 8) Save CSV
        df.to_csv(out_path, index=False)
        self.get_logger().info(f"Wrote merged dataset with {len(df)} rows to {out_path}")





def main(args=None):
    rclpy.init(args=args)
    node = OptimalSequenceVisualizerKarretjie()
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

# #!/usr/bin/env python3

# import math
# import time
# import pathlib
# import os

# import numpy as np
# import pandas as pd
# import pathlib
# import matplotlib.pyplot as plt

# import rclpy
# from rclpy.node import Node

# from nav_msgs.msg import Path as NavPath, Odometry
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float64MultiArray, Float64

# Tph = 3
# Tch = 3
# iterations = 1000
# Tnx = 4
# Tnu = 3
# rate_ = 10.0

# class OptimalSequenceVisualizerKarretjie(Node):
#     def __init__(self):
#         super().__init__('optimal_sequence_visualizer_karretjie')

#         # parameters & CSV filename
#         self.declare_parameter(
#             'csv_filename',
#             f'ph{Tph}ch{Tch}it{iterations}hz{rate_}TRACKINGKARRETJIE.csv'
#         )
#         self.csv_filename = self.get_parameter('csv_filename').value

#         # subscriptions
#         # self.create_subscription(Odometry,
#         #                          '/odom',
#         #                          self.odom_callback,
#         #                          10)
#         self.create_subscription(Float64MultiArray,
#                                  'mpcc/optimal_inputs_karretjie',
#                                  self.inputs_callback,
#                                  10)
#         self.create_subscription(NavPath,
#                                  '/mpcc/reference_path_karretjie',
#                                  self.reference_callback,
#                                  10)
#         self.create_subscription(Float64,
#                                  'mpcc/tracking_error_karretjie',
#                                  self.track_callback,
#                                  10)
#         self.create_subscription(Float64,
#                                  'mpcc/opt_time_karretjie',
#                                  self.opt_time_callback,
#                                  10)
#         self.create_subscription(Odometry,
#                                  '/mpcc/result_cmd_karretjie',
#                                  self.cmd_vel_callback,
#                                  10)
#         self.create_subscription(NavPath,
#                                 '/mpcc/actual_path_karretjie',
#                                 self.odom_callback,
#                                 10)

#         # timing & buffers
#         self.window       = 600.0
#         self.start_time   = time.time()

#         self.times        = []   # odom timestamp
#         self.x_hist       = []   # record x position
#         self.y_hist       = []   # record y position
#         self.psi_hist     = []   # yaw history
#         self.s_times      = []
#         self.s_hist       = []   # progress history
#         self.input_times  = []
#         self.u_hist       = {'v': [], 'vs': [], 'omega': []}
#         self.error_times  = []
#         self.err_hist     = []
#         self.reference_path = None
#         self.actual_xy    = []
#         self.opt_times    = []
#         self.waypoint_idx = []


#         # set up plotting
#         plt.ion()

#         # Figure 1: actual vs reference path
#         self.fig_path, self.ax_path = plt.subplots(figsize=(8,6))
#         self.ax_path.set_title('2D Path: actual vs reference')
#         self.ax_path.set_xlabel('x (m)')
#         self.ax_path.set_ylabel('y (m)')
#         self.line_pred, = self.ax_path.plot([], [], 'b-', label='actual')
#         self.line_ref,  = self.ax_path.plot([], [], 'r--', label='reference')
#         self.ax_path.legend(loc='upper right')
#         self.fig_path.tight_layout()

#         # Figure 2: time series
#         (self.fig_data,
#          (self.ax_psi,
#           self.ax_s,
#           self.ax_u,
#           self.ax_err)) = plt.subplots(4, 1, figsize=(10,12))

#         # ψ vs time
#         self.ax_psi.set_title('Heading ψ vs Time')
#         self.ax_psi.set_xlabel('Time (s)')
#         self.ax_psi.set_ylabel('ψ (rad)')
#         self.ax_psi.set_xlim(0, self.window)
#         self.ax_psi.set_ylim(-8, 8)
#         self.line_psi, = self.ax_psi.plot([], [], 'm-')

#         # s vs time
#         self.ax_s.set_title('Progress s vs Time')
#         self.ax_s.set_xlabel('Time (s)')
#         self.ax_s.set_ylabel('s (m)')
#         self.ax_s.set_xlim(0, self.window)
#         self.ax_s.set_ylim(0, 100)
#         self.line_s, = self.ax_s.plot([], [], 'g-')

#         # control inputs vs time
#         self.ax_u.set_title('Control Inputs vs Time')
#         self.ax_u.set_xlabel('Time (s)')
#         self.ax_u.set_ylabel('Linear Velocities (m/s)')
#         self.ax_u.set_xlim(0, self.window)
#         self.ax_u.set_ylim(-0.15, 0.15)
#         self.line_v,    = self.ax_u.plot([], [], 'b-',  label='v (m/s)')
#         self.line_vs,   = self.ax_u.plot([], [], 'k-.', label='vₛ (m/s)')
#         self.ax_omega = self.ax_u.twinx()
#         self.ax_omega.set_ylabel('Angular Velocity (rad/s)')
#         self.ax_omega.set_ylim(-0.8, 0.8)
#         self.line_omega, = self.ax_omega.plot([], [], 'r--', label='ω (°/s)')
#         lines1, labs1 = self.ax_u.get_legend_handles_labels()
#         lines2, labs2 = self.ax_omega.get_legend_handles_labels()
#         self.ax_u.legend(lines1+lines2, labs1+labs2, loc='upper right')

#         # tracking error vs time
#         self.ax_err.set_title('Tracking Error vs Time')
#         self.ax_err.set_xlabel('Time (s)')
#         self.ax_err.set_ylabel('Error (m)')
#         self.ax_err.set_xlim(0, self.window)
#         self.ax_err.set_ylim(-0.5, 0.5)
#         self.line_err, = self.ax_err.plot([], [], 'r-')

#         self.fig_data.tight_layout()
#         # self.create_timer(300.0, self.shutdown_callback)

#     def odom_callback(self, msg: NavPath):
#         t_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if not hasattr(self, 't0'):
#             self.t0 = t_ros
#         t = t_ros - self.t0

#         last_pose = msg.poses[-1].pose

#         x = last_pose.position.x
#         y = last_pose.position.y
#         yaw = last_pose.orientation.z
        
#         self.times.append(t)
#         self.x_hist.append(x)
#         self.y_hist.append(y)
#         self.psi_hist.append(yaw)
            
#         # update path
#         self.actual_xy.append((x, y))
#         xs, ys = zip(*self.actual_xy)
#         self.line_pred.set_data(xs, ys)

#         # update ψ history
        

#         # trim to window
#         cutoff = t - self.window
#         idx = next((i for i,v in enumerate(self.times) if v>=cutoff), 0)
#         self.times    = self.times[idx:]
#         self.psi_hist = self.psi_hist[idx:]
#         self.line_psi.set_data(self.times, self.psi_hist)
#         self.ax_psi.set_xlim(max(0, t-self.window), t)

#         # refresh canvases
#         self.fig_data.canvas.draw()
#         self.fig_data.canvas.flush_events()
#         self.fig_path.canvas.draw()
#         self.fig_path.canvas.flush_events()

#     def cmd_vel_callback(self, msg: Odometry):
#         t = time.time() - self.start_time
#         s = msg.twist.twist.angular.x
#         self.s_hist.append(s)
#         self.s_times.append(t)
#         cutoff = t - self.window
#         idx = next((i for i,v in enumerate(self.s_times) if v>=cutoff), 0)
#         self.s_hist = self.s_hist[idx:]
#         self.s_times = self.s_times[idx:]
#         self.line_s.set_data(self.s_times, self.s_hist)
#         self.ax_s.set_xlim(max(0, t-self.window), t)
#         self.fig_data.canvas.draw()
#         self.fig_data.canvas.flush_events()

#     def inputs_callback(self, msg: Float64MultiArray):
#         data = np.array(msg.data).reshape((Tnu, Tph+1)).T
#         t = time.time() - self.start_time

#         # Print out whole data array    
#         # print("Data array:")
#         # print(data)


#         self.input_times.append(t)
#         self.u_hist['v'].append(data[0,0])
#         self.u_hist['omega'].append(data[0,1])
#         self.u_hist['vs'].append(data[0,2])

#         cutoff = t - self.window
#         idx = next((i for i,v in enumerate(self.input_times) if v>=cutoff), 0)
#         self.input_times = self.input_times[idx:]
#         for k in self.u_hist:
#             self.u_hist[k] = self.u_hist[k][idx:]

#         self.line_v.set_data(self.input_times, self.u_hist['v'])
#         self.line_vs.set_data(self.input_times, self.u_hist['vs'])
#         self.ax_u.set_xlim(max(0, t-self.window), t)
#         self.line_omega.set_data(self.input_times, self.u_hist['omega'])
#         self.ax_omega.set_xlim(max(0, t-self.window), t)

#         self.fig_data.canvas.draw()
#         self.fig_data.canvas.flush_events()

#     def reference_callback(self, msg: NavPath):
#         pts = np.array([[p.pose.position.x, p.pose.position.y]
#                         for p in msg.poses])
#         if pts.size:
#             self.reference_path = pts
#             self.line_ref.set_data(pts[:,0], pts[:,1])
#             pad_x = 0.1 * pts[:,0].ptp()
#             pad_y = 0.1 * pts[:,1].ptp()
#             xmin, xmax = pts[:,0].min()-pad_x, pts[:,0].max()+pad_x
#             ymin, ymax = pts[:,1].min()-pad_y, pts[:,1].max()+pad_y
#             self.ax_path.set_xlim(xmin, xmax)
#             self.ax_path.set_ylim(ymin, ymax)
#             self.ax_path.set_aspect('equal','box')
#             self.ax_path.set_autoscalex_on(False)
#             self.ax_path.set_autoscaley_on(False)
#             self.fig_path.canvas.draw()
#             self.fig_path.canvas.flush_events()

#     def track_callback(self, msg: Float64):
#         t = time.time() - self.start_time
#         self.error_times.append(t)
#         self.err_hist.append(msg.data)
#         cutoff = t - self.window
#         idx = next((i for i,v in enumerate(self.error_times) if v>=cutoff), 0)
#         self.error_times = self.error_times[idx:]
#         self.err_hist     = self.err_hist[idx:]

        
#         self.line_err.set_data(self.error_times, self.err_hist)
#         self.ax_err.set_xlim(max(0, t-self.window), t)
#         self.fig_data.canvas.draw()
#         self.fig_data.canvas.flush_events()

#     def opt_time_callback(self, msg: Float64):
#         self.opt_times.append(msg.data)

#     def shutdown_callback(self):
#         self.get_logger().info("300 seconds elapsed – shutting down")
#         self.main_cleanup()
#         rclpy.shutdown()

#     def main_cleanup(self):

#         # 1) Path waypoints DataFrame
#         df_path = pd.DataFrame({
#             'time':      self.times,
#             'x (m)':     self.x_hist,
#             'y (m)':     self.y_hist,
#             'psi (rad)': self.psi_hist,
#         })
#         df_s = pd.DataFrame({

#             'time': self.s_times,
#             's (m)': self.s_hist,
#         })

#         # 2) Inputs DataFrame
#         df_inputs = pd.DataFrame({
#             'time':           self.input_times,
#             'v (m/s)':        self.u_hist['v'],
#             'vs (m/s)':       self.u_hist['vs'],
#             'omega (rad/s)':  self.u_hist['omega'],
#         })

#         # 3) Tracking error DataFrame
#         df_err = pd.DataFrame({
#             'time':           self.error_times,
#             'tracking_error': self.err_hist,
#         })

#         # 4) Opt-time DataFrame (align length to self.times if needed)
#         df_opt = pd.DataFrame({
#             'time':     self.times[:len(self.opt_times)],
#             'opt_time': self.opt_times,
#         })

#         # 5) Merge all on 'time' with outer joins
#         df = (
#             df_path
#             .merge(df_s,      on='time', how='outer')
#             .merge(df_inputs, on='time', how='outer')
#             .merge(df_err,    on='time', how='outer')
#             .merge(df_opt,    on='time', how='outer')
#         )

#         # 6) Sort, interpolate, then forward-fill
#         df = df.sort_values('time').reset_index(drop=True)
#         df = df.interpolate(method='index').ffill()

#         # 7) Build output filename without misusing with_suffix
#         base = pathlib.Path(self.csv_filename)
#         timestamp = int(self.start_time)
#         out_name = f"{base.stem}_{timestamp}{base.suffix}"
#         out_path = base.with_name(out_name)

#         # 8) Save CSV
#         df.to_csv(out_path, index=False)
#         self.get_logger().info(f"Wrote merged dataset with {len(df)} rows to {out_path}")




# def main(args=None):
#     rclpy.init(args=args)
#     node = OptimalSequenceVisualizerKarretjie()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.main_cleanup()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
