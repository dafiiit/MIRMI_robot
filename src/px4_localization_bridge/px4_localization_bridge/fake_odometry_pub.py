#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry

def set_if(msg, name, value):
    if hasattr(msg, name):
        setattr(msg, name, value)

class FakeOdometryPublisher(Node):
    """
    Dummy-Odometrie an PX4 über /fmu/in/vehicle_odometry (20 Hz),
    damit EKF eine lokale Pose bekommt (ohne GPS).
    """
    def __init__(self):
        super().__init__('fake_odometry_pub')
        self.pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.dt = 0.05
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info('FakeOdometryPublisher -> /fmu/in/vehicle_visual_odometry (20 Hz)')

    def timer_cb(self):
        now_us = self.get_clock().now().nanoseconds // 1000

        msg = VehicleOdometry()
        # Zeitstempel
        set_if(msg, 'timestamp', now_us)
        set_if(msg, 'timestamp_sample', now_us)

        # Kreisbewegung (x,y), z=0
        radius = 1.0
        omega  = 0.2
        self.t += self.dt
        x = radius * math.cos(omega * self.t)
        y = radius * math.sin(omega * self.t)
        z = 0.0

        # Position (verschiedene Feldnamen je nach Version)
        if hasattr(msg, 'position'):
            msg.position = [x, y, z]
        else:
            set_if(msg, 'x', x)
            set_if(msg, 'y', y)
            set_if(msg, 'z', z)

        # Orientierung (nur Yaw) -> Quaternion [x,y,z,w]
        yaw = math.atan2(y, x)
        q = [0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)]
        # (optional) normalisieren:
        n = (q[2]**2 + q[3]**2) ** 0.5
        q[2] /= n; q[3] /= n
        msg.q = q
        if hasattr(msg, 'q'):
            msg.q = q
        else:
            # manche Versionen haben einzelne Felder (sehr selten)
            set_if(msg, 'q', q)

        # Lineare Geschwindigkeit im lokalen Frame
        vx = -radius * omega * math.sin(omega * self.t)
        vy =  radius * omega * math.cos(omega * self.t)
        vz = 0.0

        if hasattr(msg, 'velocity'):
            msg.velocity = [vx, vy, vz]
        else:
            set_if(msg, 'vx', vx)
            set_if(msg, 'vy', vy)
            set_if(msg, 'vz', vz)

        # Winkelgeschwindigkeit (roll, pitch, yaw)
        if hasattr(msg, 'angular_velocity'):
            msg.angular_velocity = [0.0, 0.0, omega]
        else:
            set_if(msg, 'rollspeed', 0.0)
            set_if(msg, 'pitchspeed', 0.0)
            set_if(msg, 'yawspeed', omega)

        # Variancen/Kovarianzen (je nach Version unterschiedlich: *_variance oder *_covariance)
        if hasattr(msg, 'position_variance'):
            msg.position_variance = [0.05, 0.05, 0.10]
        if hasattr(msg, 'orientation_variance'):
            msg.orientation_variance = [0.02, 0.02, 0.003]
        if hasattr(msg, 'velocity_variance'):
            msg.velocity_variance = [0.04, 0.04, 0.09]

        # Frames (falls vorhanden): LOCAL_FRAME_NED u. ä.
        set_if(msg, 'pose_frame', 0)      # 1 = LOCAL_FRAME_NED (typisch)
        set_if(msg, 'velocity_frame', 2)  # falls Feld existiert

        # Qualität/Reset-Counter optional
        set_if(msg, 'quality', 100)
        set_if(msg, 'reset_counter', 0)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
