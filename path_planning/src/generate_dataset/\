#!/usr/bin/env python3
import roslaunch # Python API for launching .launch files
from geometry_msgs import PoseWithCovarianceStamped # Publish this as initial locations
from geometry_msgs import PoseStamped # Publish this as goals
from visualization_msgs import Marker # Subscribe this to get planned path
import rospy # Random necessary library
import sentence_generator # Import sentence dataset with corresponding waypoints
import shutil # For copying and pasting data between directories
import cv2 # For some CV stuff

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kanishk/ros_ws/wheelchair/src/dependencies/robot_navigation/dlux_plugins/test/node_test.launch"])
launch.start()
rospy.loginfo("started")
# 20 seconds later
rospy.sleep(20)
launch.shutdown()

class dataset:
    def __init__(self): # Random important function
        pub_start = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 1)
        pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)
        sub_path = rospy.Subscriber("visualization_marker", Marker, self.get_path)
        Sentences = sentence_generator.Sentences
        map_info = r'/home/kanishk/ros_ws/annotated_map_dataset/annotations/map_' # map_3.txt
        viz_marker_data = Marker()

    def copy_map_png(self, num): # Copy map's .png file from contours folder and paste into Robot Navigation package
        map_source = r'/home/kanishk/ros_ws/annotated_map_dataset/map_image/map_' + str(num) + '.png' # For example, map_3.png
        map_target = r'/home/kanishk/ros_ws/wheelchair/src/dependencies/robot_navigation/dlux_plugins/test/map.png'
        shutil.copyfile(map_source, map_target)

    def copy_map_info(self, num): # Get locations with their Y, X coordinates
        map_info_source = r'/home/kanishk/ros_ws/annotated_map_dataset/annotations/map_' + str(num) + '.txt' # For example, map_3.txt
        file = open(map_info_source, "r")
        locations_unparsed = []
        for line in file:
            locations_unparsed.append(line)
        locations_parsed = []
        for line in locations_unparsed:
            locations_parsed.append([line.split(' ')[0].split('\'')[1], float(line.split(',')[1]),float(line.split(',')[2].split(')')[0])])
        return locations_parsed

    def get_path(self.message): # ROS topic callback function to get path data
        self.viz_marker_data = message

    def save_data():








