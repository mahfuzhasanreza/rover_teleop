import rclpy
from sensor_msgs.msg import Joy

class RoverTeleopNode:
    def __init__(self):
        self.node = rclpy.create_node('rover_teleop_node')
        self.dummyNode = None

        self.joy_subscriber = self.node.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):

        if msg.buttons[2] == 1 and self.dummyNode is None:
            self.create_dummy_node()
            print("X was pressed and executed")
            self.publish_twist_msg()

        elif msg.buttons[1] == 1 and self.dummyNode is not None:  
            self.destroy_dummy_node()
            print("B was pressed and executed")

    def create_dummy_node(self):
        self.dummyNode = rclpy.create_node('teleop_is_on')

    def destroy_dummy_node(self):
        self.dummyNode.destroy_node()
        self.dummyNode = None

    def publish_twist_msg(self):
        # implement variable speed movement here
        print()

def main(args=None):
    rclpy.init(args=args)
    rover_teleop_node = RoverTeleopNode()
    try:
        rclpy.spin(rover_teleop_node.node)
    except KeyboardInterrupt:
        pass
    finally:
        rover_teleop_node.destroy_dummy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

