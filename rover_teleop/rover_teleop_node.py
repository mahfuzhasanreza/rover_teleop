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

        # xbox-360 and ps4
        # if msg.buttons[2] == 1 and self.dummyNode is None:
        #     self.send_msg = True
        #     self.create_dummy_node()
        #     print("X was pressed and executed")

        # elif msg.buttons[1] == 1 and self.dummyNode is not None:  
        #     self.send_msg = False
        #     self.destroy_dummy_node()
        #     print("B was pressed and executed")


        # logitech X-3D Pro
        if msg.buttons[1] and self.dummyNode is None:
            self.send_msg = True
            self.create_dummy_node()
            print("X was pressed and executed")

        elif msg.buttons[2] and self.dummyNode is not None:  
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
        twist = Twist()
        # implement variable speed movement here ( from reza )
        # twist.linear.x = joy.axes[1] + joy.axes[4]
        # twist.angular.z = joy.axes[0] + joy.axes[3]

        # xbox-360 and ps4
        # twist.linear.x = (joy.axes[1]/2) + (joy.axes[4]/2)
        # twist.angular.z = (joy.axes[0]/2) + (joy.axes[3]/2)

        # logitech X-3D Pro

        if joy.buttons[0]:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        # if 360 rotate is ON and usual traverse is not ON
        elif (joy.axes[2] > 0.17 or joy.axes[2] < -0.17) and not (joy.axes[1] > 0.1 or joy.axes[1] < -0.1) :
            twist.linear.x = 0.0
            twist.angular.z = joy.axes[2]
        else:
            twist.linear.x = joy.axes[1] 
            twist.angular.z = joy.axes[0] 
            # remove angluar velocity bound [ -0.1, 0.1 ]
            if twist.angular.z < 0.1 and twist.angular.z > -0.1 :
                twist.angular.z = 0.0

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

