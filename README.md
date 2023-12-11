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
        Note: Use both sticks for full accelerations 
    
    PS4
    Button:
        Joy-stick   rover movement
        Square           enables Manual Mode
        Circle           enables Autonomous Mode
        Note: Use both sticks for full accelerations

    Logitech Extreme 3D PRO
    Button:
        Joy-stick   rover movement
        Trigger     emergency brake
        2           enables Manual Mode
        3           enables Autonomous Mode
        Note: Rotate stick on it's origin to perform 360 rotations
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
