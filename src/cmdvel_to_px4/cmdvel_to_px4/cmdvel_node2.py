#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    RoverSteeringSetpoint,
    RoverSpeedSetpoint,
    RoverThrottleSetpoint,
)

class CmdVelToPx4HybridAckermann(Node):
    """
    /cmd_vel → PX4 Rover (Ackermann):
      - linear.x  -> RoverSpeedSetpoint.speed_body_x + RoverThrottleSetpoint.throttle_body_x (TrajSetpoint bleibt als Fallback)
      - angular.z -> RoverSteeringSetpoint.normalized_steering_setpoint in [-1, 1]

    Features:
      - Deadman/Timeout (setzt Sollwerte auf 0, wenn /cmd_vel zu alt)
      - Optionaler Pivot-Bias (kleiner Speed bei reinem Lenkwunsch)
      - Offboard/Arm-Sequenz nach Warmup
      - Parametrisierbar
    """

    def __init__(self):
        super().__init__('cmdvel_to_px4_hybrid_ackermann')

        # ---------- Parameter ----------
        self.declare_parameter('vx_max', 1.5)                # m/s
        self.declare_parameter('tick_rate_hz', 20.0)         # Hz
        self.declare_parameter('cmd_timeout', 0.4)           # s Deadman
        self.declare_parameter('ang_full_scale', 1.0)        # rad/s → |steer|=1.0
        self.declare_parameter('pivot_enable', True)         # nur Steering erlaubt "quasi Pivot"
        self.declare_parameter('v_pivot_thresh', 0.05)       # m/s
        self.declare_parameter('ang_pivot_thresh', 0.2)      # rad/s
        self.declare_parameter('pivot_bias_speed', 0.0)      # m/s (z. B. 0.05 falls nötig)

        self.vx_max = float(self.get_parameter('vx_max').value)
        self.dt = 1.0 / float(self.get_parameter('tick_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.ang_full_scale = float(self.get_parameter('ang_full_scale').value)
        self.pivot_enable = bool(self.get_parameter('pivot_enable').value)
        self.v_pivot_thresh = float(self.get_parameter('v_pivot_thresh').value)
        self.ang_pivot_thresh = float(self.get_parameter('ang_pivot_thresh').value)
        self.pivot_bias_speed = float(self.get_parameter('pivot_bias_speed').value)

        # ---------- ROS I/O ----------
        self.sub_cmd    = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pub_ctrl   = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_cmd    = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pub_traj      = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_speed     = self.create_publisher(RoverSpeedSetpoint, '/fmu/in/rover_speed_setpoint', 10)
        self.pub_steer     = self.create_publisher(RoverSteeringSetpoint, '/fmu/in/rover_steering_setpoint', 10)
        self.pub_throttle  = self.create_publisher(RoverThrottleSetpoint, '/fmu/in/rover_throttle_setpoint', 10)

        # ---------- State ----------
        self.last_cmd = Twist()
        self.cmd_last_ts = self.get_clock().now()
        self.offboard_set = False
        self.warmup = 0
        self._dbg_cnt = 0

        # ---------- Timer ----------
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"Hybrid Ackermann ready: vx_max={self.vx_max:.2f} m/s, rate={1/self.dt:.0f} Hz, "
            f"pivot={self.pivot_enable} (bias={self.pivot_bias_speed:.2f} m/s)"
        )

    # ---------------- Callbacks ----------------
    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.cmd_last_ts = self.get_clock().now()

    # ---------------- PX4 commands ----------------
    def set_mode_offboard(self, now_us: int):
        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE  # 176
        msg.param1 = 1.0  # PX4_CUSTOM
        msg.param2 = 6.0  # MAIN_MODE_OFFBOARD
        msg.param3 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def arm(self, now_us: int):
        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM  # 400
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    # ---------------- Main loop ----------------
    def tick(self):
        now = self.get_clock().now()
        now_us = now.nanoseconds // 1000

        # Deadman
        age = (now - self.cmd_last_ts).nanoseconds * 1e-9
        cmd = self.last_cmd
        if age > self.cmd_timeout:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # 1) OffboardControlMode (velocity only)
        ctrl = OffboardControlMode()
        ctrl.timestamp = now_us
        ctrl.position = False
        ctrl.velocity = True
        ctrl.acceleration = False
        ctrl.attitude = False
        ctrl.body_rate = False
        if hasattr(ctrl, 'direct_actuator'):
            ctrl.direct_actuator = False  # wir liefern keine ActuatorMotors direkt
        self.pub_ctrl.publish(ctrl)

        # 2) Speed via RoverSpeedSetpoint (+ optional TrajectorySetpoint für ältere Stacks)
        vx_cmd = float(np.clip(cmd.linear.x, -self.vx_max, self.vx_max))
        nan = float('nan')
        pivot_mode = (
            self.pivot_enable
            and abs(vx_cmd) < self.v_pivot_thresh
            and abs(cmd.angular.z) > self.ang_pivot_thresh
        )

        vx_bias = math.copysign(self.pivot_bias_speed, 1.0) if (pivot_mode and self.pivot_bias_speed > 0.0) else vx_cmd
        throttle_norm = float(np.clip(vx_cmd / max(self.vx_max, 1e-3), -1.0, 1.0))

        speed = RoverSpeedSetpoint()
        speed.timestamp = now_us
        speed.speed_body_x = vx_bias
        speed.speed_body_y = nan
        self.pub_speed.publish(speed)

        throttle = RoverThrottleSetpoint()
        throttle.timestamp = now_us
        throttle.throttle_body_x = throttle_norm
        throttle.throttle_body_y = nan
        self.pub_throttle.publish(throttle)

        vx = vx_cmd
        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.position = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.velocity = [vx, 0.0, 0.0]
        sp.yaw = nan
        sp.yawspeed = nan
        self.pub_traj.publish(sp)

        # 3) Steering via RoverSteeringSetpoint
        yawrate = float(cmd.angular.z)

        if pivot_mode and self.pivot_bias_speed > 0.0:
            # leichtes „Anschubsen“, falls dein Mixer Pivot nur mit etwas Speed macht
            sp2 = TrajectorySetpoint()
            sp2.timestamp = now_us
            sp2.position = [nan, nan, nan]
            sp2.acceleration = [nan, nan, nan]
            sp2.jerk = [nan, nan, nan]
            sp2.velocity = [math.copysign(self.pivot_bias_speed, 1.0), 0.0, 0.0]
            sp2.yaw = nan
            sp2.yawspeed = nan
            self.pub_traj.publish(sp2)

        # angular.z → norm [-1..1]
        full = max(1e-3, self.ang_full_scale)
        steer_norm = float(np.clip(yawrate / full, -1.0, 1.0))
        if pivot_mode:
            steer_norm = math.copysign(1.0, steer_norm)  # voller Einschlag bei Pivot

        steer = RoverSteeringSetpoint()
        steer.timestamp = now_us
        # deine px4_msgs-Version:
        steer.normalized_steering_setpoint = steer_norm
        self.pub_steer.publish(steer)

        # 4) Warmup → Offboard + Arm
        if self.warmup < int(2.0 / self.dt):
            self.warmup += 1
            return

        if not self.offboard_set:
            self.set_mode_offboard(now_us)
            self.arm(now_us)
            self.offboard_set = True
            self.get_logger().info('OFFBOARD gesetzt, ARM gesendet')

        # 5) kurze Debug-Zeile 1×/s
        self._dbg_cnt = (self._dbg_cnt + 1) % int(max(1, 1.0 / self.dt))
        if self._dbg_cnt == 0:
            self.get_logger().info(
                f"vx_cmd={vx_cmd:.2f} m/s, speed_pub={vx_bias:.2f} m/s, throttle={throttle_norm:.2f}, steer={steer_norm:.2f}, pivot={pivot_mode}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPx4HybridAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
