cmake_minimum_required(VERSION 3.0.2)
project(dataset_generation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp rospy std_msgs)

catkin_package(
#  LIBRARIES lib
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/high_res_obstacle_generator.py
  scripts/sentence_generator.py
  scripts/seg_mask.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
