#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleCommand

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class CmdVelToActuatorsSafe(Node):
    def __init__(self):
        super().__init__('cmdvel_to_actuators_safe')

        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        self.deadman_timeout = 0.3
        self.throttle_max = 0.7
        self.steer_max = 0.6
        self.tool_max = 0.7
        self.motor_index = 0          # MAIN 1 → Motor
        self.steer_index = 1          # MAIN 2 → Servo (Lenkung)
        self.tool_index = 2 
        self.num_motors = 12

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.offboard_sent = False
        self.armed = False
        self.start_delay_ticks = int(0.5 * self.rate_hz)
        self.start_counter = 0

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pub_mot = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', 10)
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info("CmdVel→ActuatorMotors (MAIN1=Throttle, MAIN2=Steer) gestartet")

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def loop(self):
        now_us = self.get_clock().now().nanoseconds // 1000

        ctrl = OffboardControlMode()
        ctrl.timestamp = now_us
        ctrl.position = False
        ctrl.velocity = False
        ctrl.acceleration = False
        ctrl.attitude = False
        ctrl.body_rate = False
        ctrl.direct_actuator = True
        self.pub_mode.publish(ctrl)

        if self.start_counter < self.start_delay_ticks:
            self.publish_actuators(now_us, 0.0, 0.0)
            self.start_counter += 1
            return

        if not self.offboard_sent:
            self.set_mode_offboard(now_us)
            self.offboard_sent = True
            self.get_logger().info("Offboard aktiviert")

        if not self.armed:
            self.arm(now_us)
            self.armed = True
            self.get_logger().info("Armed!")

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.deadman_timeout:
            throttle = 0.0
            steer = 0.0
            tool = 0.0
        else:
            throttle = clamp(self.last_cmd.linear.x, -self.throttle_max, self.throttle_max)
            steer = clamp(self.last_cmd.angular.z, -self.steer_max, self.steer_max)
            tool = clamp(self.last_cmd.linear.z, -self.tool_max, self.tool_max)

        self.publish_actuators(now_us, throttle, steer, tool)

    def publish_actuators(self, t, throttle, steer, tool):
        mot = ActuatorMotors()
        mot.timestamp = t
        mot.control = [0.0] * self.num_motors
        mot.control[self.motor_index] = throttle
        mot.control[self.steer_index] = steer
        mot.control[self.tool_index] = tool
        self.pub_mot.publish(mot)

    def arm(self, t):
        msg = VehicleCommand()
        msg.timestamp = t
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def set_mode_offboard(self, t):
        msg = VehicleCommand()
        msg.timestamp = t
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToActuatorsSafe()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
