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
        self.controller_model = None

    def joy_callback(self, msg):

        # if self.controller_model is None:
        #     self.detect_controller_model(msg)

        self.detect_controller_model(msg)

        # xbox-360 
        if self.controller_model ==  "Xbox-360 Controller":
            if msg.buttons[2] == 1 and self.dummyNode is None:
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("Manual Drive")
            elif msg.buttons[1] == 1 and self.dummyNode is not None:  
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("Autonomous Drive")

        # ps4
        elif self.controller_model ==  "PS4 Controller":
            if msg.buttons[3] == 1 and self.dummyNode is None:
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("Manual Drive")
            elif msg.buttons[1] == 1 and self.dummyNode is not None:
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("Autonomous Drive")

        # logitech x-3D pro
        elif self.controller_model ==  "Logitech X-3D Pro":
            if msg.buttons[1] == 1 and self.dummyNode is None:
                self.send_msg = True
                self.create_dummy_node()
                self.get_logger().info("Manual Drive")
            elif msg.buttons[2] == 1 and self.dummyNode is not None:
                self.send_msg = False
                self.destroy_dummy_node()
                self.get_logger().info("Autonomous Drive")
        else:
            self.get_logger().info("Unknown Controller")

        if self.send_msg:
            self.publish_twist_msg(msg)



    def create_dummy_node(self):
        self.dummyNode = rclpy.create_node('teleop_is_on')

    def destroy_dummy_node(self):
        if self.dummyNode is not None:
            self.dummyNode.destroy_node()
            self.dummyNode = None

    def publish_twist_msg(self, joy):
        twist = Twist()
        
        if self.controller_model ==  "Xbox-360 Controller":
            twist.linear.x = (joy.axes[1]/2) + (joy.axes[4]/2)
            twist.angular.z = (joy.axes[0]/2) + (joy.axes[3]/2)

        elif self.controller_model ==  "PS4 Controller":
            twist.linear.x = (joy.axes[1]/2) + (joy.axes[4]/2)
            twist.angular.z = (joy.axes[0]/2) + (joy.axes[3]/2)
            # remove velocity bound [ -0.17, 0.17 ]
            if(joy.axes[0] < 0.17 and joy.axes[0] > -0.17):
                twist.angular.z -= joy.axes[0]/2
            if (joy.axes[3] < 0.17 and joy.axes[3] > -0.17):
                twist.angular.z -= joy.axes[3]/2
            if(joy.axes[1] < 0.17 and joy.axes[1] > -0.17):
                twist.linear.x -= joy.axes[1]/2
            if(joy.axes[4] < 0.17 and joy.axes[4] > -0.17):
                twist.linear.x -= joy.axes[4]/2

        elif self.controller_model ==  "Logitech X-3D Pro":
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


    def detect_controller_model(self, msg):
        # self.get_logger().info(f"Received Joy message with {len(msg.buttons)} buttons and {len(msg.axes)} axes.")  # debug
        if len(msg.buttons) == 11 and len(msg.axes) == 8:
            self.controller_model = "Xbox-360 Controller"
        elif len(msg.buttons) == 13 and len(msg.axes) == 8:
            self.controller_model = "PS4 Controller"
        elif len(msg.buttons) == 12 and len(msg.axes) == 6:
            self.controller_model = "Logitech X-3D Pro"
        else:
            self.controller_model = "Unknown Controller Model"

        # self.get_logger().info(f"Detected Controller Model: {self.controller_model}")

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

