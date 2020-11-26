sudo chmod 666 /dev/ttyACM{0,1}
source /opt/ros/foxy/setup.bash
source ~/Documents/infant_ws/install/setup.bash
ros2 launch infant infant.launch.py
