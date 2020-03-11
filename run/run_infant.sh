sudo chmod 666 /dev/ttyACM{0,1}
source ~/infant_ws/install/setup.sh
ros2 launch ~/infant_ws/run/infant_launch.py
