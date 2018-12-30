cmake_minimum_required(VERSION 2.8.3)
project(birl_inverse_kinematic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  trac_ik_baxter
)
catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS trajectory_msgs sensor_msgs  std_msgs trac_ik_baxter)

 include_directories(
  ${catkin_INCLUDE_DIRS}
)