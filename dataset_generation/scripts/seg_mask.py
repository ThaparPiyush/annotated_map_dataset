#!/usr/bin/env python3

import sentence_generator # Import sentence dataset with corresponding waypoints
import shutil # For copying and pasting data between directories
import cv2 # For some very cool CV stuff
import numpy as np # numpy = numerical python, np = no problem
import random # Literally a random library, not kidding
import os # To get absolute paths for reading a writing the dataset
from os.path import abspath # To get absolute file path
from inspect import getsourcefile # To get absolute file path
import subprocess # To get location of package 'dlux_plugins'
from ast import literal_eval
import math
# import high_res_obstacle_generator # Start initializing dataset

class dataset:
    def __init__(self): # Random important function

        # Loading data and doing some stuff
        self.sentences = {} 
        self.cwd = abspath(getsourcefile(lambda:0))
        self.cwd = self.cwd[0:-20]
        self.map_binary_source = os.path.join(self.cwd, 'data/map_image/map_') 
        self.map_info_source = os.path.join(self.cwd, 'data/annotations/map_') 
        self.map_color_source = os.path.join(self.cwd, 'data/color_map_image/map_') 
        self.map_seg_target = os.path.join(self.cwd, 'data/map_seg_mask/')
        self.sentences_target = os.path.join(self.cwd, 'data/waypoints/') 

        # Start doing something
        self.do_stuff()

    def copy_map_info(self, num): # Get locations with their Y, X coordinates
        #if(self.map_info_source[-1] == 't'):
        #    self.map_info_source = self.map_info_source.split('')[0]
        #    #self.map_info_source = self.map_info_source[0:-5]
        f_open = self.map_info_source + str(num) + '.txt' # For example, map_3.txt
        file = open(f_open, "r")
        file.seek(0)
        locations_parsed = []
        for line in file:
            locations_parsed.append(list(literal_eval(line)))
        return locations_parsed

    def returnStartPoint(self, array): # Returns a random point (x,y) lying inside the given map
        pointIsInside = 0 # An integer variable that is even whenever point is outside the map and odd when point is inside it
        while(pointIsInside %2 == 0): # Generate random white points until one is found lying inside the map
            pointIsInside = 0
            x = random.randint(0, int(array.shape[0])-1)
            y = random.randint(0, int(array.shape[1])-1)
            if (array[x][y][0] == 0):
                continue
            pointIsInside = 0
            for i in range(x, int(array.shape[1])-1):
                if (array[x][i-1][0] !=0 and array[x][i][0] == 0):
                    pointIsInside += 1
        return (x, y)

    def returnSafePoint(self, point, array): # Return safest point near a given point in the given map array (a white cell, in our case)
        safe = 0
        distance = 0

        if(array[point[0]][point[1]][0] != 0): # If point is already safe, return it as it is
            return point

        while(safe == 0): # But if point is not safe, return the closest safest point
            distance += 1
            if (point[0]+distance <= int(array.shape[0])):
                if (array[point[0]+distance][point[1]][0] != 0):
                    x = point[0]+distance
                    y = point[1]
                    safe = 1
            elif (point[0]-distance >= 0):
                if (array[point[0]-distance][point[1]][0] != 0):
                    x = point[0]-distance
                    y = point[1]
                    safe = 1
            elif (point[1]+distance <= int(array.shape[1])):
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
        for map_num in range (1,51): # Do the following with every map
            locations_parsed = self.copy_map_info(map_num)
            sentencesObject = sentence_generator.sentences(locations_parsed)
            self.sentences = sentencesObject.returnSentences()
            map_image = cv2.imread(self.map_binary_source + str(map_num) + '.png')
            
            for sentence_index, (sentence, waypoints) in enumerate(self.sentences.items()): # Do the following with every sentence
                start_img_x = start_img_y = goal_img_x = goal_img_y = 0
                #map_seg_mask = map_image

                for waypoint_index in range(0, len(waypoints)):
                    print("Map number: ", map_num, " Sentence number: ", sentence_index, "Waypoint number: ", waypoint_index, "      ", end='\r')
                    if waypoint_index == 0:
                        (start_img_x, start_img_y) = self.returnStartPoint(map_image)
                        for location in locations_parsed:
                            if location[0] == waypoints[waypoint_index]:
                                goal_img_x = int(location[2]*100)
                                goal_img_y = int(location[1]*100)
                                #(goal_img_x, goal_img_y) = self.returnSafePoint((goal_img_x, goal_img_y), map_image)
                    else:
                        for location in locations_parsed:
                            if location[0] == waypoints[waypoint_index-1]:
                                start_img_x = int(location[2]*100)
                                start_img_y = int(location[1]*100)
                                #(start_img_x, start_img_y) = self.returnSafePoint((start_img_x, start_img_y), map_image)
                        for location in locations_parsed:
                            if location[0] == waypoints[waypoint_index]:
                                goal_img_x = int(location[2]*100)
                                goal_img_y = int(location[1]*100)
                                #(goal_img_x, goal_img_y) = self.returnSafePoint((goal_img_x, goal_img_y), map_image)
                    
                    #map_seg_mask = cv2.circle(map_seg_mask, (start_img_y, start_img_x), 20, (255, 255, 255), -1)
                    for location in locations_parsed:
                        if location[0] == waypoints[waypoint_index]:
                            map_seg_mask = np.zeros_like(map_image)
                            map_seg_mask = cv2.circle(map_seg_mask, (goal_img_y, goal_img_x), math.ceil(location[-1]), (255, 255, 255), -1)
                            cv2.imwrite((self.map_seg_target + 'm' + str(map_num) + '_s' + str(sentence_index) + '_w' + str(waypoint_index) + '.png'), map_seg_mask)

                info_target = self.sentences_target + 'm' + str(map_num) + "_s" + str(sentence_index) + ".txt"
                info_target = open(info_target, 'a')
                info_target.write(str(sentence))
                info_target.close()

dataset = dataset()
print("\n")
