import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class RoverTeleopNode(Node):

    def __init__(self):
        super().__init__('rover_teleop_node')

        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.dummyNode = None
        self.send_msg = False
        

    def joy_callback(self, msg):
        self.detect_controller_model(msg)
        self.controller_config()
    
        if msg.buttons[self.enable_button] == 1 and self.dummyNode is None:
            self.send_msg = True
            self.create_dummy_node()
            self.get_logger().info("Manual Drive")
        elif msg.buttons[self.disable_button] == 1 and self.dummyNode is not None:  
            self.send_msg = False
            self.destroy_dummy_node()
            self.get_logger().info("Autonomous Drive")

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

        if self.controller_model is not "Logitech X-3D Pro":
            twist.linear.x = (joy.axes[self.left_linear]/2) + (joy.axes[self.right_linear]/2)
            twist.angular.z = (joy.axes[self.left_angular]/2) + (joy.axes[self.right_angular]/2)

        if self.need_calibration:
            self.calibrate(joy, twist)
        
        self.cmd_vel_publisher.publish(twist)


    def detect_controller_model(self, msg):
        if len(msg.buttons) == 11 and len(msg.axes) == 8:
            self.controller_model = "Xbox-360 Controller"
        elif len(msg.buttons) == 13 and len(msg.axes) == 8:
            self.controller_model = "PS4 Controller"
        elif len(msg.buttons) == 12 and len(msg.axes) == 6:
            self.controller_model = "Logitech X-3D Pro"
        else:
            self.controller_model = "Unknown Controller Model"


    def calibrate(self, joy, twist):
        if self.controller_model ==  "PS4 Controller":
            if(joy.axes[0] < 0.17 and joy.axes[0] > -0.17):
                twist.angular.z -= joy.axes[0]/2
            if (joy.axes[3] < 0.17 and joy.axes[3] > -0.17):
                twist.angular.z -= joy.axes[3]/2
            if(joy.axes[1] < 0.17 and joy.axes[1] > -0.17):
                twist.linear.x -= joy.axes[1]/2
            if(joy.axes[4] < 0.17 and joy.axes[4] > -0.17):
                twist.linear.x -= joy.axes[4]/2
        elif self.controller_model ==  "Logitech X-3D Pro":
            if joy.buttons[self.emergency_stop]:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            # if 360 rotate is ON and usual traverse is not ON
            elif(joy.axes[2] > 0.17 or joy.axes[2] < -0.17) and not (joy.axes[1] > 0.1 or joy.axes[1] < -0.1) :
                twist.linear.x = 0.0
                twist.angular.z = joy.axes[2]
            else:
                twist.linear.x = joy.axes[self.left_linear] 
                # remove angluar velocity bound [ -0.1, 0.1 ]
                if twist.angular.z < 0.1 and twist.angular.z > -0.1:
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = joy.axes[self.left_angular]


    def controller_config(self):
        if self.controller_model == "Xbox-360 Controller":
            self.enable_button = 2
            self.disable_button = 1
            self.left_linear = 1
            self.left_angular = 0
            self.right_linear = 4
            self.right_angular = 3
            self.need_calibration = False
            self.emergency_stop = None
        elif self.controller_model == "PS4 Controller":
            self.enable_button = 3
            self.disable_button = 1
            self.left_linear = 1
            self.left_angular = 0
            self.right_linear = 4
            self.right_angular = 3
            self.need_calibration = True
            self.emergency_stop = None
        elif self.controller_model == "Logitech X-3D Pro":
            self.enable_button = 1
            self.disable_button = 2
            self.left_linear = 1
            self.left_angular = 0
            self.right_linear = None
            self.right_angular = None
            self.need_calibration = True
            self.emergency_stop = 0
        else:
            self.get_logger().info("Unknown Controller")


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

