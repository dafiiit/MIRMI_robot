#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from px4_msgs.msg import VehicleCommand


class Px4CommandServices(Node):
    def __init__(self):
        super().__init__('px4_command_services')

        self.pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.srv_arm = self.create_service(SetBool, 'px4/arm', self.on_arm)
        self.srv_offboard = self.create_service(SetBool, 'px4/offboard', self.on_offboard)

        self.get_logger().info("Services ready: /px4/arm, /px4/offboard (std_srvs/SetBool)")

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True

        self.pub.publish(msg)

    def on_arm(self, request: SetBool.Request, response: SetBool.Response):
        arm = bool(request.data)
        # VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        self._publish_vehicle_command(command=400, param1=(1.0 if arm else 0.0))

        response.success = True
        response.message = "ARM sent" if arm else "DISARM sent"
        self.get_logger().info(response.message)
        return response

    def on_offboard(self, request: SetBool.Request, response: SetBool.Response):
        enable = bool(request.data)

        if enable:
            # VEHICLE_CMD_DO_SET_MODE = 176
            # param1 = 1 (custom mode), param2 = 6 (PX4 OFFBOARD)
            self._publish_vehicle_command(command=176, param1=1.0, param2=6.0)
            response.success = True
            response.message = "OFFBOARD mode request sent"
        else:
            # "Offboard aus" ist setup-abhängig.
            # Sinnvolle Default-Variante: zurück auf POSCTL (Position Control) param2 = 3
            # (Wenn du lieber HOLD/MANUAL willst, sag Bescheid.)
            self._publish_vehicle_command(command=176, param1=1.0, param2=3.0)
            response.success = True
            response.message = "POSCTL mode request sent (offboard disabled)"

        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = Px4CommandServices()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
