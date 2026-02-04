#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/cmd_vel → PX4 Direct Actuator Control
Default-Setup für Rover:
  - MAIN1 → Throttle (Motor)
  - AUX1  → Steering (Servo)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from px4_msgs.msg import ActuatorMotors, ActuatorServos, OffboardControlMode, VehicleCommand


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class CmdVelToPx4Rover(Node):
    def __init__(self):
        super().__init__('cmdvel_to_px4_rover')

        # ---- Feste Standardparameter für dein Setup ----
        self.rate_hz = 20.0
        self.dt = 1.0 / self.rate_hz
        self.deadman = 0.05     # Sekunden ohne cmd_vel → Stop
        self.warmup = 0.50      # Sekunden bis Offboard aktiv
        self.throttle_max = 0.9 # max. Motorkommandos (±) 0.6
        self.throttle_bias = 0.0 # Neutralstellung des ESC
        self.tool_bias = 0.0
        self.steer_max = 0.9    # max. Lenkkommandos (±) 0.6
        self.invert_speed = False
        self.invert_steer = False

        # Hardware-Zuordnung (wie bei dir)
        self.steering_on_main = False  # False = AUX-Bank
        self.motor_index = 0           # MAIN1 = Motor
        self.steer_index_aux = 0       # AUX1  = Lenkservo
        self.steer_index_main = 1      # (nur falls steering_on_main=True)

        # Array-Längen automatisch bestimmenself.throttle_bias = -0.25
        self.n_main = len(ActuatorMotors().control)   # typ. 12
        self.n_aux = len(ActuatorServos().control)    # typ. 8

        # ROS2 Setup
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos)
        self.sub_stop = self.create_subscription(Empty, '/stop', self.on_stop, qos)
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_main = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos) # nach vorne/hinten Fahren
        self.pub_aux = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos) # Lenkung

        # State
        self.last_cmd_ts = self.get_clock().now()
        self.vx = 0.0
        self.wz = 0.0
        self.vz = 0.0
        self.force_stop = False
        self.warmup_ticks = int(self.warmup / self.dt)
        self.warm_cnt = 0
        self.offboard = False
        self.armed = False
        self._dbg = 0

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"PX4 Rover Node gestartet (MAIN1→Throttle, AUX1→Steering) "
            f"| MAIN={self.n_main} AUX={self.n_aux}"
        )

    # --- Callbacks ---
    def on_cmd(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.vz = msg.linear.z
        self.last_cmd_ts = self.get_clock().now()
        self.force_stop = False

    def on_stop(self, _):
        self.force_stop = True
        self.get_logger().warn("NOT-STOP: sofort 0")

    # --- PX4 Kommandos ---
    def hb_offboard_mode(self, t_us: int):
        m = OffboardControlMode()
        m.timestamp = t_us
        m.position = False
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.direct_actuator = True
        self.pub_mode.publish(m)

    def set_offboard(self, t_us: int):
        vc = VehicleCommand()
        vc.timestamp = t_us
        vc.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        vc.param1 = 1.0  # PX4_CUSTOM
        vc.param2 = 6.0  # MAIN_MODE_OFFBOARD
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.pub_cmd.publish(vc)

    def arm(self, t_us: int, arm: bool):
        vc = VehicleCommand()
        vc.timestamp = t_us
        vc.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vc.param1 = 1.0 if arm else 0.0
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.pub_cmd.publish(vc)
        
    def set_actuator_set1(self, t_us: int, value_norm: float):
        vc = VehicleCommand()
        vc.timestamp = t_us
        vc.command = 187  # MAV_CMD_DO_SET_ACTUATOR
        vc.param1 = float(clamp(value_norm, -1.0, 1.0))  # Set 1 -> MAIN3
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.pub_cmd.publish(vc)


    # --- Publisher ---
    def pub_main_aux(self, t_us: int, thr: float, steer: float):
        mot = ActuatorMotors()
        mot.timestamp = t_us
        mot.control = [float('nan')] * self.n_main
        #self.get_logger().info(f"Publishing to MAIN motors: thr={thr}, steer={steer}")
        if 0 <= self.motor_index < self.n_main:
            mot.control[self.motor_index] = thr
            # Bitmaske: Bit i gehört zu motor_index i
            mot.reversible_flags = (1 << self.motor_index)
        if self.steering_on_main and 0 <= self.steer_index_main < self.n_main:
            mot.control[self.steer_index_main] = steer
            
        self.pub_main.publish(mot)

        if not self.steering_on_main:
            srv = ActuatorServos()
            srv.timestamp = t_us
            srv.control = [float('nan')] * self.n_aux
            if 0 <= self.steer_index_aux < self.n_aux:
                srv.control[self.steer_index_aux] = steer
                
            self.pub_aux.publish(srv)

    # --- Main Loop ---
    def tick(self):
        now = self.get_clock().now()
        t_us = now.nanoseconds // 1000

        self.hb_offboard_mode(t_us)

        if self.warm_cnt < self.warmup_ticks:
            self.pub_main_aux(t_us, -self.throttle_bias, 0.0)
            self.set_actuator_set1(t_us, -self.tool_bias)
            self.warm_cnt += 1
            return

        if not self.offboard:
            self.set_offboard(t_us)
            self.offboard = True
            self.get_logger().info("OFFBOARD aktiviert")

        if not self.armed:
            self.arm(t_us, True)
            self.armed = True
            self.get_logger().info("ARM gesendet")

        # Deadman-Stop
        age = (now - self.last_cmd_ts).nanoseconds * 1e-9
        timed_out = age > self.deadman
        raw_thr = 0.0
        steer = 0.0
        raw_tool = 0.0
        if not (self.force_stop or timed_out):
            raw_thr = clamp(self.vx, -1.0, 1.0) * self.throttle_max
            steer = clamp(self.wz, -1.0, 1.0) * self.steer_max
            raw_tool = clamp(self.vz, -1.0, 1.0)
        
        
        if self.invert_speed:
            raw_thr = -raw_thr
        if self.invert_steer:
            steer = -steer
                
        thr = clamp(raw_thr - self.throttle_bias, -1.0, 1.0)
        tool = clamp(raw_tool + self.tool_bias,-1.0, 1.0)
        self.pub_main_aux(t_us, thr, steer)
        self.set_actuator_set1(t_us, tool) 

        # Debug-Ausgabe einmal pro Sekunde
        self._dbg = (self._dbg + 1) % int(max(1, 1.0 / self.dt))
        if self._dbg == 0:
            self.get_logger().info(
                f"thr={thr:.2f}, steer={steer:.2f}, tool={tool:.2f}, age={age:.2f}s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPx4Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
