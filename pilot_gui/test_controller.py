import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyViewer(Node):
    def __init__(self):
        super().__init__("joy_viewer")
        self.log = self.get_logger()
        self.log.info("hello")
        self.count = 0
        self.joy_sub = self.create_subscription(Joy,'joy',self.pub_joy,qos_profile=10)
    
    def pub_joy(self,joy):
        print("hello")
        self.count+=1
        print(self.count)
        print(joy.buttons)

def main(args=None):
   rclpy.init(args=args)
   joy_viewer = JoyViewer()
   rclpy.spin(joy_viewer)
   joy_viewer.destroy_node()
   rclpy.shutdown()
 
if __name__ == "__main__":
   main()