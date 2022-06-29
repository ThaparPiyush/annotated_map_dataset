#!/usr/bin/env python3

# Python3 program to draw circle
# shape on solid image
import numpy as np
import cv2
import math
import json
import os
import argparse
import yaml
import time
import subprocess
import codecs
import random
from easydict import EasyDict as edict

pwd = os.getcwd()

config = {


    "map_info": {

        "json_path": os.path.join(pwd, "HouseExpo/HouseExpo/json"), #"/home/rrc/wheelchair/src/dependencies/HouseExpo/HouseExpo/json",
        "map_id_set_file": os.path.join(pwd, "HouseExpo/HouseExpo/map_id_10.txt"), #"/home/rrc/wheelchair/src/dependencies/HouseExpo/HouseExpo/map_id_10.txt",
        "save_path": pwd, #"/home/rrc/annotated_map_dataset",
        "dark_colors_file": os.path.join(pwd, "textures/dark_colors.txt"), #"/home/rrc/annotated_map_dataset/textures/dark_colors.txt",
        "light_colors_file": os.path.join(pwd, "textures/light_colors.txt") #"/home/rrc/annotated_map_dataset/textures/light_colors.txt",
    },


    "world_params": {

        "meter2pixel": 100,
        "border_pad": 25,
        "obs_num_min": 5,
        "obs_num_max": 10,
        "obs_size_range": [0.02, 0.1],
        "min_dist_between_obs": 0,
        "bounding_box_pad": 20,

    },

    "data_collect_params": {
        
        "step_size": 50, # In pixels
        "bot_size": 0.49, # In meters

    }


}

class DataGenerator:

    def __init__(self,
                 config: dict) -> None:

        
        self.config = config
        self.annotation_list = []
        # Variables to load map
        self.json_path = ""
        self.map_file = ""
        self.save_path = ""
        self.map_ids = []
        
        self.light_colors_file = os.path.abspath(os.path.join(os.getcwd(), self.config.map_info.light_colors_file))
        self.dark_colors_file = os.path.abspath(os.path.join(os.getcwd(), self.config.map_info.dark_colors_file))
        self.load_map_file()

        self.map_img_path = os.path.join(self.save_path, "map_image")
        self.color_map_img_path = os.path.join(self.save_path, "color_map_image")
        self.map_contours_info_path = os.path.join(self.save_path, "map_contours_info")
        self.map_yaml_path = os.path.join(self.save_path, "map_yaml")

        if not os.path.exists(self.save_path): os.mkdir(self.save_path)
        if not os.path.exists(self.map_img_path): os.mkdir(self.map_img_path)
        if not os.path.exists(self.color_map_img_path): os.mkdir(self.color_map_img_path)
        if not os.path.exists(self.map_contours_info_path): os.mkdir(self.map_contours_info_path)
        if not os.path.exists(self.map_yaml_path): os.mkdir(self.map_yaml_path)
        
        self.light_colors = np.loadtxt(self.light_colors_file, str).tolist()
        self.dark_colors = np.loadtxt(self.dark_colors_file, str).tolist()

        for i in range(len( self.light_colors)):
             self.light_colors[i][1] = int( self.light_colors[i][1])
             self.light_colors[i][2] = int( self.light_colors[i][2])
             self.light_colors[i][3] = int( self.light_colors[i][3])

        for i in range(len( self.dark_colors)):
             self.dark_colors[i][1] = int( self.dark_colors[i][1])
             self.dark_colors[i][2] = int( self.dark_colors[i][2])
             self.dark_colors[i][3] = int( self.dark_colors[i][3])

    def load_map_file(self):
        self.json_path = os.path.abspath(os.path.join(os.getcwd(), self.config.map_info.json_path))
        self.map_file = os.path.abspath(os.path.join(os.getcwd(), self.config.map_info.map_id_set_file))
        self.save_path = os.path.abspath(os.path.join(os.getcwd(), self.config.map_info.save_path))
        print("---------------------------------------------------------------------")
        print("|map id set file path        |{}".format(self.map_file))
        print("---------------------------------------------------------------------")
        print("|json file path              |{}".format(self.json_path))
        print("---------------------------------------------------------------------")
        print("|Save path                   | {}".format(self.save_path))
        print("---------------------------------------------------------------------")


        self.map_ids = np.loadtxt(self.map_file, str)
        print(f"Number of maps to process: {len(self.map_ids)}")

    def generate_data(self):
        # self.load_map_file()
        number =1
        for map_id in self.map_ids:
            self.draw_map(map_id, number)
            number = number+1
            # break
            
        # while True:
        #     pass



    def draw_map(self, file_name, number):
        print("Processing... ", file_name)

        print(number)
        with open(self.json_path + '/' + file_name + '.json') as json_file:
            json_data = json.load(json_file)

        # Draw the contour
        verts = (np.array(json_data['verts']) * self.config.world_params.meter2pixel).astype(int)
        x_max, x_min, y_max, y_min = np.max(verts[:, 0]), np.min(verts[:, 0]), np.max(verts[:, 1]), np.min(verts[:, 1])
        cnt_map = np.ones((y_max - y_min + self.config.world_params.border_pad * 2,
                            x_max - x_min + self.config.world_params.border_pad * 2)) 
        cnt_map = cnt_map * 255

        verts[:, 0] = verts[:, 0] - x_min + self.config.world_params.border_pad
        verts[:, 1] = verts[:, 1] - y_min + self.config.world_params.border_pad
        cv2.drawContours(cnt_map, [verts], 0, 0, 5)

        noise_removal_threshold=25
        corners_threshold=0.05
        room_closing_max_length=200
        room_closing_min_length= 25
        gap_in_wall_threshold=10
        block_size = 3
        aperture_size = 3
        k = 0.04

        dst = cv2.cornerHarris(cnt_map.astype(np.uint8) ,block_size,aperture_size,k)
        dst = cv2.dilate(dst,None)
        corners = dst > corners_threshold * dst.max()
        # print(dst)

        # Draw lines to close the rooms off by adding a line between corners on the same x or y coordinate

        # This gets some false positives.
        # You could try to disallow drawing through other existing lines for example.
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        text_thickness = 1
        x3 = x4 = 0
        # img = cv2.cvtColor(cnt_map.astype(np.uint8),cv2.COLOR_GRAY2RGB)
        for y,row in enumerate(corners):
            x_same_y = np.argwhere(row)
            for x1, x2 in zip(x_same_y[:-1], x_same_y[1:]):
                if room_closing_min_length < x2[0] - x1[0] < room_closing_max_length:
                    color = (0,0,0)
                    cv2.line(cnt_map , (int(x1), int(y)), (int(x2), int(y)), color, 1)
                # cv2.imshow('dst',img)

        for x,col in enumerate(corners.T):
            y_same_x = np.argwhere(col)
            for y1, y2 in zip(y_same_x[:-1], y_same_x[1:]):
                if room_closing_min_length < y2[0] - y1[0] < room_closing_max_length:
                    color = 0
                    cv2.line(cnt_map ,(int(x), int(y1)), (int(x), int(y2)), color, 1)

        # Mark the outside of the house as black
        mask = np.zeros_like(cnt_map.astype(np.uint8))
        contours, _ = cv2.findContours(~cnt_map.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
        biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
        mask = np.zeros_like(mask)
        cv2.fillPoly(mask, [biggest_contour], 255)
        cnt_map[mask == 0] = 0

        # Find the connected components in the house
        output = cv2.connectedComponentsWithStats(cnt_map.astype(np.uint8))
        (ret, labels, stats, centroids) = output
        # print(centroids)
        color_img = cv2.cvtColor(cnt_map.astype(np.uint8),cv2.COLOR_GRAY2RGB)
        unique = np.unique(labels)
        rooms = []
        doors = []
        room_center = []
        door_center = []
        color_enumerator = 0

        for label in unique:
            component = labels == label
            if color_img[component].sum() == 0 or np.count_nonzero(component) < gap_in_wall_threshold:
                if np.count_nonzero(component) < gap_in_wall_threshold:
                    color = 255
                else:
                    color = 0
            else:
                area = stats[label, cv2.CC_STAT_AREA]
                if area > 6000:
                    # print('ROOM_AREA:',area)
                    rooms.append(component)                    
                    room_center.append(centroids[label])
                    color =(self.light_colors[color_enumerator][1],self.light_colors[color_enumerator][2],self.light_colors[color_enumerator][3])
                    color_enumerator = color_enumerator +1
                elif area < 300:
                    # print('DOOR_AREA:',area)
                    doors.append(component)
                    door_center.append(centroids[label])
                    (cX, cY) = centroids[label]
                    orgC = (int(cX), int(cY))
                    # cv2.circle(cnt_map, orgC, 25, 0,thickness=2)
                    # cv2.imshow('dst',cnt_map)
                    # if cv2.waitKey(0) & 0xff == 27:
                    #     cv2.destroyAllWindows()
                    color =255
                # print(color_enumerator)
                # color = np.random.randint(0, 255, size=3)
            color_img[component] = color
        # print('LENGTH1: ' ,len(door_center))
        for center1 in door_center:
            i = 0
            for center2 in door_center:
                # print ('x1: ', center1[0],'x2: ', center2[0],'y1: ', center1[1],'y2: ', center2[1])
                if center1[0] != center2[0] or center1[1] != center2[1]:
                    distance = np.sqrt((center2[0]-center1[0])**2 + (center2[1]-center1[1])**2)
                    # print ('Distance: ',distance)
                    if distance < 100:
                        # print(center2)
                        door_center.pop(i)
                i = i+1
        # print(door_center)
        # print('LENGTH2: ' ,len(door_center))
        # cv2.imshow('dst',img)
        # if cv2.waitKey(0) & 0xff == 27:
        #     cv2.destroyAllWindows()

        # Add Random Obstacles
        images = self.add_obstacles(number, corners, room_closing_max_length, room_center, door_center, verts, color_img, cnt_map)
        cnt_map = images[0]
        color_img = images[1]
        # cnt_map = cv2.cvtColor(cnt_map.astype(np.uint8),cv2.COLOR_GRAY2RGB)

        # #Remoiving Doors
        # for y,row in enumerate(corners):
        #     x_same_y = np.argwhere(row)
        #     for x1, x2 in zip(x_same_y[:-1], x_same_y[1:]):

        #         if x2[0] - x1[0] < room_closing_max_length:
        #             color = (255, 255, 255)
        #             cv2.line(cnt_map , (int(x1), int(y)), (int(x2), int(y)), color, 1)

        # for x,col in enumerate(corners.T):
        #     y_same_x = np.argwhere(col)
        #     for y1, y2 in zip(y_same_x[:-1], y_same_x[1:]):
        #         if y2[0] - y1[0] < room_closing_max_length:
        #             color = (255, 255, 255)
        #             cv2.line(cnt_map ,(int(x), int(y1)), (int(x), int(y2)), color, 1)
        # redraw the layout
        cv2.drawContours(cnt_map, [verts], 0, 0, 70)



        #Remoiving Doors
        for y,row in enumerate(corners):
            x_same_y = np.argwhere(row)
            for x1, x2 in zip(x_same_y[:-1], x_same_y[1:]):

                if room_closing_min_length < x2[0] - x1[0] < room_closing_max_length:
                    color = (255, 255, 255)
                    cv2.line(color_img , (int(x1), int(y)), (int(x2), int(y)), color, 1)

        for x,col in enumerate(corners.T):
            y_same_x = np.argwhere(col)
            for y1, y2 in zip(y_same_x[:-1], y_same_x[1:]):
                if room_closing_min_length < y2[0] - y1[0] < room_closing_max_length:
                    color = (255, 255, 255)
                    cv2.line(color_img ,(int(x), int(y1)), (int(x), int(y2)), color, 1)
        # redraw the layout
        cv2.drawContours(color_img, [verts], 0, 0, 5)


        # Copy to global vars
        self.current_world_img = cnt_map.copy()
        self.current_world_contours = [verts]

        cnt_map[mask == 0] = 255
        color_img[mask == 0] = 255

        # cnt_map = cv2.addWeighted(color_img,0.2,cnt_map,0.8,0)
        # Save map
        if not os.path.exists(self.map_img_path): os.mkdir(self.map_img_path)
        if not os.path.exists(self.color_map_img_path): os.mkdir(self.color_map_img_path)
        if not os.path.exists(self.map_contours_info_path): os.mkdir(self.map_contours_info_path)

        output_file_name = 'map_{}'.format(number)
        cv2.imwrite(self.color_map_img_path + "/" + output_file_name + '_color.png', color_img)
        cv2.imwrite(self.map_img_path + "/" + output_file_name + '.png', cnt_map)
        np.save(self.map_contours_info_path + '/' + output_file_name + '.npy', verts)


        
    def add_obstacles(self, number, corners, room_closing_max_length, room_center, door_center, verts, color_img, cnt_map:np.array) -> np.array:


        obs_num_min = self.config.world_params.obs_num_min
        obs_num_max = self.config.world_params.obs_num_max
        obs_num = np.random.randint(obs_num_min, obs_num_max)
        bounding_box_pad = self.config.world_params.bounding_box_pad
        # print (obs_num)
        obs_sizeRange = self.config.world_params.obs_size_range
        # obs_sizeRange = obs_sizeRange * self.config.world_params.meter2pixel
        # obs_sizeRange = [np.round(obs_sizeRange[0]), np.round(obs_sizeRange[1])]
        prox_min = 0 # min distance in pixel between added obstacle & obstacle in map
        
        (h,w)= cnt_map.shape
        # cnt_map = cv2.cvtColor(cnt_map.astype(np.uint8),cv2.COLOR_GRAY2RGB)
        # # cnt_map = color_img
        obs_sizeRange = np.array(obs_sizeRange)
        obs_sizeRange = obs_sizeRange * min(h, w)
        # print(obs_sizeRange)

        count_ = 0
        annotate_number = 0
        room_annotate_number = 0
        door_annotate_number = 0
        color = (0, 0, 0)
        room_color = (0,0,0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.7
        text_thickness = 2
        
        #Remoiving Doors
        for y,row in enumerate(corners):
            x_same_y = np.argwhere(row)
            for x1, x2 in zip(x_same_y[:-1], x_same_y[1:]):

                if x2[0] - x1[0] < room_closing_max_length:
                    door_color = (255, 255, 255)
                    cv2.line(cnt_map , (int(x1), int(y)), (int(x2), int(y)), door_color, 1)

        for x,col in enumerate(corners.T):
            y_same_x = np.argwhere(col)
            for y1, y2 in zip(y_same_x[:-1], y_same_x[1:]):
                if y2[0] - y1[0] < room_closing_max_length:
                    door_color = (255, 255, 255)
                    cv2.line(cnt_map ,(int(x), int(y1)), (int(x), int(y2)), door_color, 1)

        # Annotating Rooms
        for i in range(len(room_center)):
            (cX, cY) = room_center[i]
            orgC = (int(cX), int(cY))
            # print(cX)
            room_annotate_number = room_annotate_number + 1
            annotate = 'Room_%d (%d, %d) '%(room_annotate_number, cX/100, cY/100)
            #text = cv2.putText(cnt_map, annotate, orgC, font,fontScale, room_color, text_thickness, cv2.LINE_AA)
            self.annotation_list.append(('Room_%d' %(room_annotate_number), cX/100, cY/100))
        # Annotating Doors
        for i in range(len(door_center)):
            (cX, cY) = door_center[i]
            orgC = (int(cX), int(cY)+50)
            # print(cX)
            door_annotate_number = door_annotate_number + 1
            annotate = 'Door_%d (%d, %d) '%(door_annotate_number, cX/100, cY/100)
            #text = cv2.putText(cnt_map, annotate, orgC, font,fontScale, room_color, text_thickness, cv2.LINE_AA)
            self.annotation_list.append(('Door_%d' %(door_annotate_number), cX/100, cY/100))
 
        # Annotating Obstacles
        for i in range(obs_num):
            while (1):
                # randomly generate obstacle orientation & obstacle size that fall within obstacle_sizeRange
                obs_a = np.random.randint(obs_sizeRange[0], obs_sizeRange[1])*0.5
                obs_b = np.random.randint(obs_sizeRange[0], obs_sizeRange[1])*0.5
                obs_theta = np.random.random()*360
                bound_obs_a = obs_a + bounding_box_pad
                bound_obs_b = obs_b + bounding_box_pad

                # randomly select shape type of obstacle [0: rectangle; 1: ellipse; 2: circle]
                obs_type = np.random.randint(0, 2)
                # obs_type = 0
                if obs_type == 2:
                    obs_b = obs_a

                # randomly generate obstacle center coordinate that the obstacle would not exceed world boundary
                bound = 0.0

                if obs_type == 0:
                    bound = math.sqrt((obs_a * 2) ** 2 + (obs_b * 2) ** 2) + prox_min
                    # print(bound)
                elif obs_type == 1:
                    bound = math.sqrt((obs_a * 2) ** 2 + (obs_b * 2) ** 2) + prox_min
                else:
                    bound = obs_a * 2
                
                bound = int(bound)

                if bound * 2 >= h or bound * 2 >= w:
                    continue


                obs_y = np.random.randint(bound, h-bound)
                obs_x = np.random.randint(bound, w-bound)
                # check if the location of obstacle to be added intersect with other obstacle present
                if np.sum(cnt_map[obs_y-bound:obs_y+bound, obs_x-bound:obs_x+bound]==0)!= 0:
                    count_ += 1
                    if count_ >= 1000:
                        break
                    continue
                # cnt_map = color_img
                # check if point is in the map
                result = cv2.pointPolygonTest(verts, (obs_x, obs_y), False) 
                if result != 1:
                    break
                org = (obs_x, obs_y)

                annotate_number = annotate_number + 1
                annotate = 'Table_%d (%d, %d) '%(annotate_number, obs_x/100, obs_y/100)
                self.annotation_list.append(('Table_%d' %(annotate_number), obs_x/100, obs_y/100))

                with open(r'annotations/map_{}.txt'.format(number), 'w') as fp:
                    for annotations in self.annotation_list:
                        fp.write('{0}\n' .format(annotations))
                    fp.close()
                # print(self.annotation_list)   
                
                # Green color in BGR

                box_color =(self.dark_colors[i][1],self.dark_colors[i][2],self.dark_colors[i][3])
                thickness = 8
                isClosed = False
                # create obstacle patch
                if obs_type == 0:
                    cthe = np.cos(np.pi/180* obs_theta)
                    sthe = np.sin(np.pi/180* obs_theta)
                    rect = np.array([[obs_x + (-obs_a * cthe - -obs_b * sthe), obs_y + (-obs_a * sthe + -obs_b * cthe)],
                                        [obs_x + (-obs_a * cthe - obs_b * sthe), obs_y + (-obs_a * sthe + obs_b * cthe)],
                                        [obs_x + (obs_a * cthe - obs_b * sthe), obs_y + (obs_a * sthe + obs_b * cthe)],
                                        [obs_x + (obs_a * cthe - -obs_b * sthe), obs_y + (obs_a * sthe + -obs_b * cthe)]],
                                    np.int32)
                    # create a bounding box
                    bound_rect = np.array([[obs_x + (-bound_obs_a * cthe - -bound_obs_b * sthe), obs_y + (-bound_obs_a * sthe + -bound_obs_b * cthe)],
                                        [obs_x + (-bound_obs_a * cthe - bound_obs_b * sthe), obs_y + (-bound_obs_a * sthe + bound_obs_b * cthe)],
                                        [obs_x + (bound_obs_a * cthe - bound_obs_b * sthe), obs_y + (bound_obs_a * sthe + bound_obs_b * cthe)],
                                        [obs_x + (bound_obs_a * cthe - -bound_obs_b * sthe), obs_y + (bound_obs_a * sthe + -bound_obs_b * cthe)]],
                                    np.int32)
                    #cv2.polylines(cnt_map, [bound_rect], True, color, 2)
                    cv2.polylines(cnt_map, [bound_rect], True, color, thickness=1)
                    cv2.fillPoly(cnt_map, [bound_rect], (0, 0, 0))
                    cv2.polylines(cnt_map, [rect], isClosed, color,thickness)

                    # color_img = cv2.polylines(color_img, [bound_rect], True, color, 2)
                    color_img =cv2.fillPoly(color_img, [bound_rect], box_color)
                    color_img = cv2.polylines(color_img, [rect], isClosed, color,thickness)
                    # print(i)
                    # cv2.imshow('dst',color_img)
                    # if cv2.waitKey(0) & 0xff == 27:
                    #     cv2.destroyAllWindows()
                    # image = cv2.addWeighted(image,0.3,cnt_map,0.7,0)
                    #text = cv2.putText(cnt_map, annotate, org, font,fontScale, color, text_thickness, cv2.LINE_AA)

                else:
                    if annotate_number%2 == 0:
                        cthe = np.cos(np.pi/180* obs_theta)
                        sthe = np.sin(np.pi/180* obs_theta)
                        rect1 = np.array([[obs_x + (-obs_a * cthe - -obs_b * sthe), obs_y + (-obs_a * sthe + -obs_b * cthe)],
                                            [obs_x + (-obs_a * cthe - obs_b * sthe), obs_y + (-obs_a * sthe + obs_b * cthe)],
                                            [obs_x + (obs_a * cthe - obs_b * sthe), obs_y + (obs_a * sthe + obs_b * cthe)]],
                                        np.int32)
                        bound_rect1 = np.array([[obs_x + (-bound_obs_a * cthe - -bound_obs_b * sthe), obs_y + (-bound_obs_a * sthe + -bound_obs_b * cthe)],
                                            [obs_x + (-bound_obs_a * cthe - bound_obs_b * sthe), obs_y + (-bound_obs_a * sthe + bound_obs_b * cthe)],
                                            [obs_x + (bound_obs_a * cthe - bound_obs_b * sthe), obs_y + (bound_obs_a * sthe + bound_obs_b * cthe)],
                                            [obs_x + (bound_obs_a * cthe - -bound_obs_b * sthe), obs_y + (bound_obs_a * sthe + -bound_obs_b * cthe)]],
                                        np.int32)
                        #cv2.polylines(cnt_map, [bound_rect1], True, 0, 2)
                        cv2.polylines(cnt_map, [bound_rect1], True, 0, thickness=1)
                        cv2.fillPoly(cnt_map, [bound_rect1], (0, 0, 0))
                        cv2.polylines(cnt_map, [rect1],isClosed, color,thickness=25)

                        # color_img = cv2.polylines(color_img, [bound_rect1], True, 0, 2)
                        color_img =cv2.fillPoly(color_img, [bound_rect1], box_color)
                        color_img = cv2.polylines(color_img, [rect1],isClosed, color,thickness)
                        #text = cv2.putText(cnt_map, annotate, org, font,fontScale, color, text_thickness, cv2.LINE_AA)            
                    else:
                        cv2.circle(cnt_map, (obs_x,obs_y), int(bound_obs_a), 0,thickness=-1)
                        cv2.circle(cnt_map, (obs_x,obs_y), int(obs_a), 0,thickness=8)

                        cv2.circle(color_img, (obs_x,obs_y), int(bound_obs_a), box_color,thickness=25)
                        # cv2.circle(color_img, (obs_x,obs_y), int(bound_obs_a), 0,thickness=1)
                        cv2.circle(color_img, (obs_x,obs_y), int(obs_a), 0,thickness=8)
                        #text = cv2.putText(cnt_map, annotate, org, font,fontScale, color, text_thickness, cv2.LINE_AA)
                break
        
        images = [cnt_map, color_img]
        self.annotation_list.clear()
        return images


config = edict(config)
data_generator = DataGenerator(config)
data_generator.load_map_file()
data_generator.generate_data()        
