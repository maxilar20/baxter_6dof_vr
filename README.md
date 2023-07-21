Dependencies

- ZED-ros2-wrapper
- Perception PCL
- Baxter_common_ros2
- ROS-TCP-Endpoint

Docker:

- docker pull maxilar20/ros2_baxter_bridge

To run:
TERMINAL 1:
sudo docker run -it --net=host ros2_baxter_bridge
source opt/ros/noetic/setup.bash
source ros_ws/devel/setup.bash
export ROS_IP="130.209.252.206"
export ROS_MASTER_URI="http://cornwall:11311"
rosrun baxter_tools enable_robot.py -e

source opt/ros/galactic/setup.bash
source ros2_bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge

TERMINAL 2:
ros2 launch baxter_6dof_vr baxter_6dof_vr.launch.py

TERMINAL 3 (After mapping is done):
ros2 service call /zed2/zed_node/enable_mapping std_srvs/srv/SetBool "{data: False}"
