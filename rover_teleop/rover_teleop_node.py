import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class RoverTeleopNode(Node):

    def __init__(self):

        super().__init__('rover_teleop_node')
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        self.dummyNode = None
        self.send_msg = False


    def joy_callback(self, msg):

        if msg.buttons[2] == 1 and self.dummyNode is None:
            self.send_msg = True
            self.create_dummy_node()
            print("X was pressed and executed")

        elif msg.buttons[1] == 1 and self.dummyNode is not None:  
            self.send_msg = False
            self.destroy_dummy_node()
            print("B was pressed and executed")

        if self.send_msg:
            self.publish_twist_msg(msg)

    def create_dummy_node(self):
        self.dummyNode = rclpy.create_node('teleop_is_on')

    def destroy_dummy_node(self):
        self.dummyNode.destroy_node()
        self.dummyNode = None

    def publish_twist_msg(self, joy):
        # implement variable speed movement here ( from reza )
        twist = Twist()
        twist.linear.x = joy.axes[1] + joy.axes[4]
        twist.angular.z = joy.axes[0] + joy.axes[3]
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rover_teleop_node = RoverTeleopNode()
    try:
        rclpy.spin(rover_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        rover_teleop_node.destroy_dummy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

