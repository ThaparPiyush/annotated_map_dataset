#!/bin/sh
#rm data/annotations/* > /dev/null 2>&1
#rm data/map_contours_info/* > /dev/null 2>&1
#rm data/map_yaml/* > /dev/null 2>&1
#rm data/color_map_image/* > /dev/null 2>&1
#rm data/map_image/* > /dev/null 2>&1
rm data/map_seg_mask/* > /dev/null 2>&1
rm data/waypoints/* > /dev/null 2>&1
#touch data/annotations/.gitkeep
#touch data/map_contours_info/.gitkeep
#touch data/map_yaml/.gitkeep
#touch data/color_map_image/.gitkeep
#touch data/map_image/.gitkeep
touch data/map_seg_mask/.gitkeep
touch data/waypoints/.gitkeep
