#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    RoverSteeringSetpoint,
    RoverSpeedSetpoint,
    ActuatorServos,
    ActuatorMotors,
)

class RoverTestCLIv2(Node):
    def __init__(self):
        super().__init__("rover_test_cli_v2")

        # pubs
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.pub_ofb = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.pub_steer = self.create_publisher(RoverSteeringSetpoint, "/fmu/in/rover_steering_setpoint", 10)
        self.pub_speed = self.create_publisher(RoverSpeedSetpoint, "/fmu/in/rover_speed_setpoint", 10)
        self.pub_servos = self.create_publisher(ActuatorServos, "/fmu/in/actuator_servos", 10)
        self.pub_motors = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", 10)

        # 20 Hz OffboardControlMode keepalive
        self.timer = self.create_timer(0.05, self.keepalive_cb)

        self.get_logger().info("Starte in 1.5 s …")
        time.sleep(1.5)

        self.arm_and_offboard()
        self.test_rover_steering()
        self.test_rover_speed()
        self.test_actuator_servos()
        self.test_actuator_motors()

        self.get_logger().info("✅ Fertig.")
        rclpy.shutdown()

    def keepalive_cb(self):
        msg = OffboardControlMode()
        msg.timestamp = int(time.time() * 1e6)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        self.pub_ofb.publish(msg)

    def publish_burst(self, pub, msg, seconds=0.5, rate_hz=20):
        dt = 1.0 / rate_hz
        t_end = time.time() + seconds
        while time.time() < t_end:
            # Update timestamp, falls Feld existiert
            if hasattr(msg, "timestamp"):
                msg.timestamp = int(time.time() * 1e6)
            pub.publish(msg)
            time.sleep(dt)

    def arm_and_offboard(self):
        self.get_logger().info("🟢 OFFBOARD+ARM …")
        m = VehicleCommand()
        m.timestamp = int(time.time() * 1e6)
        m.command = 176; m.param1 = 1.0; m.param2 = 6.0
        m.target_system = 1; m.target_component = 1; m.source_system = 1; m.source_component = 1
        m.from_external = True
        self.publish_burst(self.pub_cmd, m, seconds=0.3)

        m = VehicleCommand()
        m.timestamp = int(time.time() * 1e6)
        m.command = 400; m.param1 = 1.0
        m.target_system = 1; m.target_component = 1; m.source_system = 1; m.source_component = 1
        m.from_external = True
        self.publish_burst(self.pub_cmd, m, seconds=0.3)
        time.sleep(0.5)
        self.get_logger().info("✔️ gesendet.")

    def test_rover_steering(self):
        self.get_logger().info("🔄 RoverSteeringSetpoint: rechts → links → Mitte")
        for val, name in [(1.0, "rechts"), (-1.0, "links"), (0.0, "mitte")]:
            msg = RoverSteeringSetpoint()
            msg.normalized_steering_setpoint = float(val)
            self.publish_burst(self.pub_steer, msg, seconds=5)
            self.get_logger().info(f"  → {name} ({val:+.2f})")
            time.sleep(0.5)

    def test_rover_speed(self):
        self.get_logger().info("🏎️ RoverSpeedSetpoint: vorwärts → rückwärts → stop")
        for val, name in [(0.5, "vorwärts"), (-0.4, "rückwärts"), (0.0, "stop")]:
            msg = RoverSpeedSetpoint()
            msg.speed_body_x = float(val)
            msg.speed_body_y = float("nan")
            self.publish_burst(self.pub_speed, msg, seconds=5)
            self.get_logger().info(f"  → {name} ({val:+.2f} m/s)")
            time.sleep(0.5)

    def test_actuator_servos(self):
        self.get_logger().info("⚙️ ActuatorServos (Index 0 = Kanal 1)")
        for val, name in [(1.0, "rechts"), (-1.0, "links"), (0.0, "mitte")]:
            msg = ActuatorServos()
            # Länge des Arrays aus dem Objekt nehmen (meist 8)
            n = len(msg.control)
            arr = [float("nan")] * n
            arr[0] = float(val)
            msg.control = arr
            self.publish_burst(self.pub_servos, msg, seconds=5)
            self.get_logger().info(f"  → Servo {name} ({val:+.2f})")
            time.sleep(0.5)

    def test_actuator_motors(self):
        self.get_logger().info("⚙️ ActuatorMotors (Index 0 = Kanal 1)")
        for val, name in [(0.5, "vorwärts"), (-0.3, "rückwärts"), (0.0, "stop")]:
            msg = ActuatorMotors()
            # WICHTIG: Hier ist bei dir Länge 12 – wir lesen sie dynamisch:
            n = len(msg.control)
            arr = [float("nan")] * n
            arr[0] = float(val)
            msg.control = arr
            self.publish_burst(self.pub_motors, msg, seconds=5)
            self.get_logger().info(f"  → Motor {name} ({val:+.2f})")
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    RoverTestCLIv2()
    rclpy.spin(rclpy.create_node("dummy_spin"))

if __name__ == "__main__":
    main()
