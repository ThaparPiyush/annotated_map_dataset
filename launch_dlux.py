#!/usr/bin/env python3
import roslaunch # Python API for launching launch files
from geometry_msgs.msg import PoseWithCovarianceStamped # Publish this as start locations
from geometry_msgs.msg import PoseStamped # Publish this as goal location
from visualization_msgs.msg import Marker # Subscribe this to get planned path
import rospy # Random necessary library
import sentence_generator # Import sentence dataset with corresponding waypoints
import shutil # For copying and pasting data between directories
import cv2 # For some very cool CV stuff
import random # Literally a random library, not kidding
import os

print(os.getcwd())

rospy.init_node("mynode")
class dataset:
    def __init__(self): # Random important function

        # Just some initialization
        self.pub_start = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 1)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)
        self.sub_path = rospy.Subscriber("visualization_marker", Marker, self.get_path)
        self.start = PoseWithCovarianceStamped()
        self.goal = PoseStamped()
        self.viz_marker_data = Marker()
        self.start.header.stamp = self.goal.header.stamp = self.viz_marker_data.header.stamp = rospy.Time.now()
        self.start.header.frame_id = self.goal.header.frame_id = self.viz_marker_data.header.frame_id = "map"
        self.start.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.start.pose.pose.orientation.w = self.goal.pose.orientation.w = self.viz_marker_data.pose.orientation.w = 1
        self.viz_marker_data.scale.x = self.viz_marker_data.scale.y = self.viz_marker_data.scale.z = 2
        
        # Loading data and doing some stuff
        self.sentences = {} # sentences = sentences.sentences(nDoors, nRooms, nTables)
        self.map_info = r'/home/kanishk/ros_ws/annotated_map_dataset/annotations/map_' # Filling it in function 'copy_map_info' below. Example: map_3.txt
        self.map_source = r'/home/kanishk/ros_ws/annotated_map_dataset/map_image/map_' # Filling it in function 'copy_map_png' below. Example: map_3.png
        self.map_target = r'/home/kanishk/ros_ws/wheelchair/src/dependencies/robot_navigation/dlux_plugins/test/map.png'
        self.map_info_source = r'/home/kanishk/ros_ws/annotated_map_dataset/annotations/map_'
        self.map_color_source = r'/home/kanishk/ros_ws/annotated_map_dataset/color_map_image/map_' # Filling it in function 'do_stuff' below. Example: map_3_color.png
        self.map_color_target = r'/home/kanishk/ros_ws/annotated_map_dataset/color_map_image/map_' # Filling it in function 'do_stuff' below. Example: map_3_17.png, for sentence 17 in map 3.

        # Launch file setup before starting
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/kanishk/ros_ws/wheelchair/src/dependencies/robot_navigation/dlux_plugins/test/node_test.launch"])

        # Start doing something
        self.do_stuff()


    def copy_map_png(self, num): # Copy map's .png file from contours folder and paste into Robot Navigation package
        if (self.map_source[-1] == 'g'):
            self.map_source = self.map_source[0:-5]
        self.map_source = self.map_source + str(num) + '.png' 
        shutil.copyfile(self.map_source, self.map_target)

    def copy_map_info(self, num): # Get locations with their Y, X coordinates
        if(self.map_info_source[-1] == 't'):
            self.map_info_source = self.map_info_source[0:-5]
        self.map_info_source = self.map_info_source + str(num) + '.txt' # For example, map_3.txt
        file = open(self.map_info_source, "r")
        locations_unparsed = [] # Raw data taken from file in String format
        for line in file:
            locations_unparsed.append(line)
        locations_parsed = [] # Made a list of places out of raw string
        for line in locations_unparsed:
            locations_parsed.append([line.split(' ')[0].split('\'')[1], float(line.split(',')[1]),float(line.split(',')[2].split(')')[0])])
        return locations_parsed

    def get_path(self, message): # ROS topic callback function to get path data
        self.viz_marker_data = message

    def returnStartPoint(self, array): # Returns a point (x,y) that lies inside the given map
        height = int(array.shape[0])
        width = int(array.shape[1])
        pointIsInside = 0 # An integer variable that is even whenever point is outside the map and odd when point is inside it
        while(pointIsInside %2 == 0): # Generate random white points until one is found lying inside the map
            pointIsInside = 0
            x = random.randint(0, height-1)
            y = random.randint(0, width-1)
            if (array[x][y][0] == 0):
                continue
            pointIsInside = 0
            for i in range(x, width-1):
                if (array[x][i-1][0] !=0 and array[x][i][0] == 0):
                    pointIsInside += 1
        return (x, y)

    def returnSafePoint(self, point, array): # Return safest point near a place (a white cell, in our case)
        height = int(array.shape[0])
        width = int(array.shape[1])
        safe = 0
        distance = 0

        if(array[point[0]][point[1]][0] != 0): # If point is already safe, return it as it is
            return point

        while(safe == 0): # But if point is not safe, return the closest safest point
            distance += 1
            if (point[0]+distance <= height):
                if (array[point[0]+distance][point[1]][0] != 0):
                    x = point[0]+distance
                    y = point[1]
                    safe = 1
            elif (point[0]-distance >= 0):
                if (array[point[0]-distance][point[1]][0] != 0):
                    x = point[0]-distance
                    y = point[1]
                    safe = 1
            elif (point[1]+distance <= width):
                if (array[point[0]][point[1]+distance][0] != 0):
                    x = point[0]
                    y = point[1]+distance
                    safe = 1
            elif (point[1]-distance >=0):
                if (array[point[0]][point[1]-distance][0] != 0):
                    x = point[0]
                    y = point[1]-distance
                    safe = 1
        return (x, y)

    def do_stuff(self):
        for map_num in range (1,11): # Do the following with every map
            num = map_num
            self.copy_map_png(num)
            locations = self.copy_map_info(num)
            sentencesObject = sentence_generator.sentences(locations)
            sentences = sentencesObject.returnSentences()
            map_image = cv2.imread(self.map_target)
            self.launch.start() # Started navigation launch file
            
            for sentence_index, (sentence, waypoints) in enumerate(sentences.items()): # Do the following with every sentence
                start_img_x = start_img_y = goal_img_x = goal_img_y = start_rviz_x = start_rviz_y = goal_rviz_x = goal_rviz_y = 0
                
                for waypoint_index in range(0, len(waypoints)):
                    
                    if waypoint_index == 0:
                        (start_img_x, start_img_y) = self.returnStartPoint(map_image)
                        for location in locations:
                            if location[0] == waypoints[waypoint_index]:
                                goal_img_x = int(location[2]*100)
                                goal_img_y = int(location[1]*100)
                                (goal_img_x, goal_img_y) = self.returnSafePoint((goal_img_x, goal_img_y), map_image)
                    else:
                        for location in locations:
                            if location[0] == waypoints[waypoint_index-1]:
                                start_img_x = int(location[2]*100)
                                start_img_y = int(location[1]*100)
                                (start_img_x, start_img_y) = self.returnSafePoint((start_img_x, start_img_y), map_image)
                        for location in locations:
                            if location[0] == waypoints[waypoint_index]:
                                goal_img_x = int(location[2]*100)
                                goal_img_y = int(location[1]*100)
                                (goal_img_x, goal_img_y) = self.returnSafePoint((goal_img_x, goal_img_y), map_image)
                    
                    print("========")
#                    (start_img_x, start_img_y) = (88, 1310)
#                    (goal_img_x, goal_img_y) = (800, 500)
                    print("\nOn map %d, sentence %d, waypoint %d : Start = (%d, %d), Goal = (%d, %d)" % (map_num, sentence_index, waypoint_index, start_img_x, start_img_y, goal_img_x, goal_img_y))
                    print(sentence, " -> ", waypoints)
                    print("Navigation started...")
                    self.start.pose.pose.position.x = start_img_y/100
                    self.start.pose.pose.position.y = (map_image.shape[0] - start_img_x)/100
                    self.goal.pose.position.x = goal_img_y/100
                    self.goal.pose.position.y = (map_image.shape[0] - goal_img_x)/100
                    self.pub_start.publish(self.start)
                    self.pub_goal.publish(self.goal)
                    rospy.sleep(4)
                    print("... Navigation finished")
                    points_out_list = []
                    color_map_in = cv2.imread(self.map_color_source + str(map_num) + '_color.png')
                    for point in range(0, len(self.viz_marker_data.points)):
                        h = int(color_map_in.shape[0] - (100*self.viz_marker_data.points[point].y))
                        w = int(100*self.viz_marker_data.points[point].x)
                        if point == 0:
                            color_map_in = cv2.circle(color_map_in, (w, h), 20, (0, 255, 0), -1) # Make a circle with color BGR (B = 0, G = 255, R = 0) filled inside it
                            points_out_list.append((h, w))
                        elif point == len(self.viz_marker_data.points)-1:
                            color_map_in = cv2.circle(color_map_in, (w, h), 20, (255, 0, 0), -1)
                            points_out_list.append((h, w))
                        elif point % (int(len(self.viz_marker_data.points)/10)) == 0:
                            color_map_in = cv2.circle(color_map_in, (w, h), 20, (0, 0, (point/len(self.viz_marker_data.points) + 150)), -1)
                            points_out_list.append((h, w))
                    if len(self.viz_marker_data.points) != 0:
                        color_map_out = self.map_color_source + str(map_num) + '_' + str(sentence_index) + '_color.png'
                        cv2.imwrite(color_map_out, color_map_in)
                        map_info_target = self.map_info_source + str(map_num) + '_' + str(sentence_index) + '.txt'
                        map_info_file = open(map_info_target, 'a')
                        map_info_file.write('===\n' + str(map_info_target) + '\n' + str(sentence) + ' -> ' + str(waypoints) + '\n' + str(points_out_list) + 'n')
                    
dataset = dataset()
rospy.spin()
rospy.loginfo("\nNode exited\n")
