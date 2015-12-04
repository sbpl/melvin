To install ps3 drivers:
sudo add-apt-repository ppa:falk-t-j/qtsixa
sudo apt-get update
sudo apt-get install sixad

To pair:
Plug in both joystick and bluetooth dongle
sudo sixpair

To connect to ROS:
sixad --boot-yes #first time only
sixad --start
rosrun joy joy_node
