## Description:
```javascript
Nodes:
    Publisher   : joy_node   
    Subscriber  : rover_teleop_node

Topic:
    /joy        controller inputs
    /cmd_vel    command velocity

Message:
    [Joy]       (docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)
    [Twist]     (docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

Controller:
    Xbox-360

Button:
    Joy-stick   rover movement
    X           enables Manual Mode
    B           enables Autonomous Mode

```

### Setup
```bash
# install dependency
sudo apt install ros-<distro>-joy

# install package
cd ~/ros2_ws/src/
git clone https://github.com/sheikhshakibhossain/rover_teleop.git
cd ..
colcon build

# run the package
source ~/ros2_ws/install/setup.bash
ros2 launch rover_teleop rover_teleop.launch.py
```
