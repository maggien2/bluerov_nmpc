#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import OverrideRCIn

import numpy as np
import casadi as ca
import do_mpc
import tf_transformations as tf
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil
import time 

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

class NMPCNode(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.master.wait_heartbeat()
        self.get_logger().info("MAVLink connected")

        self.x0 = None

        self.t0 = time.time()
        self.time_hist = []
        self.eta_hist = []
        self.eta_ref_hist = []
        self.err_hist = []
        self.tau_hist = []

        self.sub = self.create_subscription(
            Odometry,
            'model/orca4/odometry',
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        self.eta_ref = np.zeros((6,1))
        self.ref_sub = self.create_subscription(
            Float64MultiArray,
            '/mpc/eta_ref',
            self.ref_callback,
            10
        )
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode service...')

        self.timer = self.create_timer(0.1, self.control_loop)
        self.model, self.mpc = self.build_mpc()

    def build_mpc(self):

        model = do_mpc.model.Model('continuous')

        eta = model.set_variable('_x', 'eta', (6,1))
        nu  = model.set_variable('_x', 'nu',  (6,1))
        tau = model.set_variable('_u', 'tau', (6,1))

        m = 11.4
        Ix, Iy, Iz = 0.21, 0.245, 0.245
        zg = 0.02
        g = 9.81

        phi, theta, psi = eta[3], eta[4], eta[5]

        cphi, sphi = ca.cos(phi), ca.sin(phi)
        cth, sth   = ca.cos(theta), ca.sin(theta)
        cps, sps   = ca.cos(psi), ca.sin(psi)
        tth        = ca.tan(theta)

        R = ca.vertcat(
            ca.horzcat(cps*cth, -sps*cphi + cps*sth*sphi,  sps*sphi + cps*cphi*sth),
            ca.horzcat(sps*cth,  cps*cphi + sphi*sth*sps, -cps*sphi + sth*sps*cphi),
            ca.horzcat(-sth,     cth*sphi,                cth*cphi)
        )

        T_ang = ca.vertcat(
            ca.horzcat(1, sphi*tth, cphi*tth),
            ca.horzcat(0, cphi,    -sphi),
            ca.horzcat(0, sphi/cth, cphi/cth)
        )

        J = ca.vertcat(
            ca.horzcat(R, ca.DM.zeros(3,3)),
            ca.horzcat(ca.DM.zeros(3,3), T_ang)
        )

        Xdu, Ydv, Zdw = 6.36, 7.12, 18.68
        Kdp, Mdq, Ndr = 0.189, 0.135, 0.222

        M_A = ca.diag(ca.vertcat(Xdu, Ydv, Zdw, Kdp, Mdq, Ndr))

        M_RB = ca.vertcat(
                ca.horzcat(m, 0, 0, 0, m*zg, 0),
                ca.horzcat(0, m, 0, -m*zg, 0, 0),
                ca.horzcat(0, 0, m, 0, 0, 0),
                ca.horzcat(0, -m*zg, 0, Ix, 0, 0),
                ca.horzcat(m*zg, 0, 0, 0, Iy, 0),
                ca.horzcat(0, 0, 0, 0, 0, Iz)
                )

        M = M_RB + M_A

        u1,u2,u3,p,q,r = nu[0],nu[1],nu[2],nu[3],nu[4],nu[5]

        C_RB = ca.vertcat(
            ca.horzcat(0, 0, 0, 0, m*u3, -m*u2),
            ca.horzcat(0, 0, 0, -m*u3, 0, m*u1),
            ca.horzcat(0, 0, 0, m*u2, -m*u1, 0),
            ca.horzcat(0, m*u3, -m*u2, 0, Iz*r, -Iy*q),
            ca.horzcat(-m*u3, 0, m*u1, -Iz*r, 0, Ix*p),
            ca.horzcat(m*u2, -m*u1, 0, Iy*q, -Ix*p, 0)
        )

        C_A = ca.vertcat(
            ca.horzcat(0, 0, 0, 0, -Zdw*u3,  Ydv*u2),
            ca.horzcat(0, 0, 0,  Zdw*u3, 0, -Xdu*u1),
            ca.horzcat(0, 0, 0, -Ydv*u2, Xdu*u1, 0),
            ca.horzcat(0, -Zdw*u3,  Ydv*u2, 0, -Ndr*r,  Mdq*q),
            ca.horzcat(Zdw*u3, 0, -Xdu*u1,  Ndr*r, 0, -Kdp*p),
            ca.horzcat(-Ydv*u2, Xdu*u1, 0, -Mdq*q, Kdp*p, 0)
        )

        C = C_RB + C_A

        Xu, Xuu = 13.7, 141.0
        Yv, Yvv = 0, 217.0
        Zw, Zww = 33.0, 190.0
        Kp, Kpp = 0, 1.19
        Mq, Mqq = 0.8, 0.47
        Nr, Nrr = 0, 1.5


        D = ca.diag(ca.vertcat(
            Xu + Xuu*ca.fabs(u1),
            Yv + Yvv*ca.fabs(u2),
            Zw + Zww*ca.fabs(u3),
            Kp + Kpp*ca.fabs(p),
            Mq + Mqq*ca.fabs(q),
            Nr + Nrr*ca.fabs(r)
        ))

        W = m*g
        g_eta = -ca.vertcat(0,0,0, zg*W*cth*sphi, zg*W*sth, 0)

        eta_dot = J @ nu
        nu_dot  = ca.solve(M , tau - C @ nu - D @ nu - g_eta)

        model.set_rhs('eta', eta_dot)
        model.set_rhs('nu', nu_dot)

        eta_ref = model.set_variable('_tvp', 'eta_ref', (6,1))

        model.setup()

        mpc = do_mpc.controller.MPC(model)

        mpc.set_param(
            n_horizon=20,
            t_step=0.1,
            state_discretization='collocation',
            nlpsol_opts={'ipopt.print_level':0, 'print_time':0}
        )

        tvp_template = mpc.get_tvp_template()

        def tvp_fun(t_now):
            for k in range(mpc.settings.n_horizon + 1):
                tvp_template['_tvp', k, 'eta_ref'] = ca.DM(self.eta_ref)
            return tvp_template

        mpc.set_tvp_fun(tvp_fun)


        Q = ca.diag(ca.DM([2, 2, 5, 1, 1, 2]))
        # R = 0.1 * ca.DM.eye(6)
        R = ca.diag(ca.DM([0.02, 0.02, 0.02, 1, 1, 0.1]))

        x_err = model.x['eta'] - eta_ref
        mterm = ca.mtimes([x_err.T, Q, x_err])
        lterm = mterm + ca.mtimes([model.u['tau'].T, R, model.u['tau']])

        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(tau=0.01)

        tau_lower = np.array([
            -10,   # Fx
            -10,   # Fy
            -10,   # Fz
            -1,    # Mx
            -1,    # My
            -1     # Mz
        ]).reshape(6,1)

        tau_upper = np.array([
            10,
            10,
            10,
            1,
            1,
            1
        ]).reshape(6,1)

        mpc.bounds['lower','_u','tau'] = tau_lower
        mpc.bounds['upper','_u','tau'] = tau_upper

        mpc.setup()

        return model, mpc

    def odom_callback(self, msg):

        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        q = msg.pose.pose.orientation
        euler = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])

        eta = np.array([pos.x, pos.y, pos.z, *euler])
        nu  = np.array([vel.x, vel.y, vel.z, ang.x, ang.y, ang.z])

        self.x0 = np.concatenate([eta, nu]).reshape(12,1)

    def ref_callback(self, msg):
        data = np.array(msg.data)

        if data.size != 6:
            self.get_logger().warn("eta_ref must have 6 elements")
            return

        self.eta_ref = data.reshape(6,1)

    def force_to_pwm(self, F):
        if abs(F) < 0.3:
            return 1500
        F = F / 9.81
        if F > 0:
            pwm = -5.887*F**2 + 92.45*F + 1541.4
        else:
            pwm = 8.397*F**2 + 113.78*F + 1460.4
        return int(np.clip(pwm, 1100, 1900))

    def arm_vehicle(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        self.get_logger().info("Arming sent")


    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        self.get_logger().info(f"Mode set to {mode}")
    
    def send_rc_overrides(self, channel_map):

        rc_values = [65535] * 18
        
        for ch, pwm in channel_map.items():
            if 1 <= ch <= 18:
                rc_values[ch - 1] = int(pwm)
                
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_values
        )
        
    def control_loop(self):

        if self.x0 is None:
            self.get_logger().info("Waiting for odometry...")
            return
        
        if not hasattr(self, "vehicle_initialized"):
            self.set_mode('STABILIZE')   # or 'STABILIZE'
            self.arm_vehicle()
            self.vehicle_initialized = True
            self.get_logger().info("Vehicle initialized (mode + arm)")
            return
        
        if not hasattr(self, "mpc_initialized"):
            self.mpc.x0 = self.x0.flatten()
            self.mpc.set_initial_guess()
            self.mpc_initialized = True
            self.get_logger().info("MPC initialized")
            return

        self.mpc.x0 = self.x0.flatten()
        u0 = self.mpc.make_step(self.x0.flatten())

        tau = np.array(u0).reshape(6,1)

        tau[5] = tau[5] * -1
        tau[1] = tau[1] * -1

        self.log_data(tau)

        rc_commands = {
            1: self.force_to_pwm(tau[4]), # pitch, ang_y
            3: self.force_to_pwm(tau[2]), # throttle, z
            4: self.force_to_pwm(tau[5]), # yaw, ang_z
            5: self.force_to_pwm(tau[0]), # surge, x
            6: self.force_to_pwm(tau[1])  # sway, y
        }

        print(self.force_to_pwm(tau[0]) - 1500,
              self.force_to_pwm(tau[1]) - 1500,
              self.force_to_pwm(tau[2]) - 1500,
            #   self.force_to_pwm(tau[3]) - 1500,
              self.force_to_pwm(tau[4]) - 1500,
              self.force_to_pwm(tau[5]) - 1500,)
        
        self.send_rc_overrides(rc_commands)

    def log_data(self, tau):
        t = time.time() - self.t0

        eta = self.x0[:6].flatten()
        eta_ref = self.eta_ref.flatten()
        err = eta_ref - eta
        tau = np.array(tau).flatten()

        self.time_hist.append(t)
        self.eta_hist.append(eta)
        self.eta_ref_hist.append(eta_ref)
        self.err_hist.append(err)
        self.tau_hist.append(tau)

    def plot_results(self, cutoff=0.0):
        if len(self.time_hist) < 2:
            self.get_logger().warn("Not enough data to plot.")
            return

        t = np.array(self.time_hist)
        eta = np.array(self.eta_hist)
        eta_ref = np.array(self.eta_ref_hist)
        err = np.array(self.err_hist)
        tau = np.array(self.tau_hist)

        mask = t >= cutoff

        if np.sum(mask) < 2:
            self.get_logger().warn("Cutoff too large.")
            return

        t = t[mask] - cutoff
        eta = eta[mask]
        eta_ref = eta_ref[mask]
        err = err[mask]
        tau = tau[mask]

        labels = ['x','y','z','roll','pitch','yaw']
        tau_labels = ['Fx','Fy','Fz','Mx','My','Mz']

        fig1, axs = plt.subplots(1, 2, figsize=(14, 5))

        axs[0].plot(eta[:,0], eta[:,1], linewidth=2, label='Actual')
        axs[0].plot(
            eta_ref[:,0],
            eta_ref[:,1],
            '--',
            linewidth=2,
            label='Reference'
        )
        axs[0].set_title(f'XY Trajectory (t > {cutoff}s)')
        axs[0].set_xlabel('x [m]')
        axs[0].set_ylabel('y [m]')
        axs[0].grid(True)
        axs[0].axis('equal')
        axs[0].legend()

        axs[1].plot(t, eta[:,2], linewidth=2, label='Actual z')
        axs[1].plot(
            t,
            eta_ref[:,2],
            '--',
            linewidth=2,
            label='Reference z'
        )
        axs[1].set_title(f'Depth Tracking (t > {cutoff}s)')
        axs[1].set_xlabel('Time [s]')
        axs[1].set_ylabel('z [m]')
        axs[1].grid(True)
        axs[1].legend()

        fig1.suptitle('Trajectory (Cutoff)', fontsize=16)
        plt.tight_layout()

        fig2, axs = plt.subplots(3, 2, figsize=(14,10), sharex=True)
        axs = axs.flatten()

        for i in range(6):
            axs[i].plot(t, err[:,i], linewidth=2)
            axs[i].set_title(f'{labels[i]} error')
            axs[i].set_ylabel('error')
            axs[i].grid(True)

        axs[4].set_xlabel('Time [s]')
        axs[5].set_xlabel('Time [s]')

        fig2.suptitle(f'Tracking Error (t > {cutoff}s)', fontsize=16)
        plt.tight_layout()

        fig3, axs = plt.subplots(3, 2, figsize=(14,10), sharex=True)
        axs = axs.flatten()

        for i in range(6):
            axs[i].plot(t, tau[:,i], linewidth=2)
            axs[i].set_title(tau_labels[i])
            axs[i].set_ylabel('tau')
            axs[i].grid(True)

        axs[4].set_xlabel('Time [s]')
        axs[5].set_xlabel('Time [s]')

        fig3.suptitle(f'MPC Control Inputs (t > {cutoff}s)', fontsize=16)
        plt.tight_layout()

        plt.show()

    def compute_rms_error(self, cutoff=0.0):
        if len(self.err_hist) == 0:
            self.get_logger().warn("No error data.")
            return

        t = np.array(self.time_hist)
        err = np.array(self.err_hist)

        mask = t >= cutoff

        if np.sum(mask) == 0:
            self.get_logger().warn("Cutoff too large.")
            return

        err_ss = err[mask]

        rms = np.sqrt(np.mean(err_ss**2, axis=0))

        labels = ['x','y','z','roll','pitch','yaw']

        print(f"\n===== RMS Error (t > {cutoff}s) =====")
        for l, r in zip(labels, rms):
            print(f"{l:>5}: {r:.4f}")

        pos_rms = np.sqrt(np.mean(err_ss[:,0:3]**2))
        ang_rms = np.sqrt(np.mean(err_ss[:,3:6]**2))

        print(f"\nPosition RMS: {pos_rms:.4f} m")
        print(f"Attitude RMS: {ang_rms:.4f} rad")   


def main(args=None):
    rclpy.init(args=args)
    node = NMPCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping node and plotting results...")
    finally:
        node.compute_rms_error(cutoff=0.0)
        node.plot_results(cutoff=0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
