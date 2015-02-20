# chessboard_localization

This is a ROS package that provides localization feedback with respect to a standard chessboard pattern. The package has been tested on Ubuntu 14.04 and ROS Indigo.

Bringup instructions:

1. To install ROS Indigo, please follow the ROS wiki instruction in the following link: http://wiki.ros.org/ROS/Installation

2. Please create a catkin workspace with the following instruction: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

3. Clone the package into the src folder under the workspace (e.g. ~/catkin_ws/src from the example in step 2.) and compile via: 

   git clone https://github.com/y22ma/chessboard_localization.git
   
   cd ~/catkin_ws
   
   catkin_make
   
4. Run the demo via:

   rosrun chessboard_localization run.bash
   
   roscd chessboard_localization/data
   
   rosbag play sample.bag #please ask Jimmy for this dataset

This should bring up a RVIZ GUI that visualize the pose of the camera with respect to the chessboard during the events of the sample.bag dataset.
