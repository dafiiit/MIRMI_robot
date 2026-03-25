import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JSTester(Node):
    def __init__(self):
        super().__init__('js_tester')
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.get_logger().info("Listening to /joint_states")
        
    def cb(self, msg):
        n = len(msg.name)
        p = len(msg.position)
        v = len(msg.velocity)
        e = len(msg.effort)
        self.get_logger().info(f"Received msg -> name:{n}, pos:{p}, vel:{v}, eff:{e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = JSTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
