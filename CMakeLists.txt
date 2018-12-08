cmake_minimum_required(VERSION 2.8.3)
project(birl_inverse_kinematic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  trac_ik_baxter
)
catkin_python_setup()

