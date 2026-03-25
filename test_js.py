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
        
        is_invalid = False
        if p > 0 and n != p:
            is_invalid = True
        if v > 0 and n != v:
            is_invalid = True
        if e > 0 and n != e:
            is_invalid = True
        if n == 0 and (p > 0 or v > 0 or e > 0):
            is_invalid = True
            
        if is_invalid:
            self.get_logger().error(f"Invalid sizes! name:{n}, pos:{p}, vel:{v}, eff:{e}")
            self.get_logger().error(f"names: {msg.name}")
            
def main():
    rclpy.init()
    node = JSTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
