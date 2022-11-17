import os, shutil

data_path = '/scratch/wheelchair/annotated_map_dataset/dataset_generation/data'
split_folders = ['train', 'val']
data_folders = ['color_map_image', 'map_seg_mask', 'waypoints']
split_ratio = 0.75

seg_orig = os.listdir(os.path.join(data_path, 'map_seg_mask'))
col_orig = os.listdir(os.path.join(data_path, 'color_map_image'))
way_orig = os.listdir(os.path.join(data_path, 'waypoints'))

for col_map_i in range(len(col_orig)):
	for seg_map in seg_orig:
		if col_orig[col_map_i].split('.')[0] == seg_map.split('_')[0][1:]:
			if col_map_i <= int(len(col_orig))*split_ratio:
				shutil.copy(os.path.join(data_path, 'color_map_image', col_orig[col_map_i]), os.path.join(data_path, 'data_texture', split_folders[0], 'color_map_image', col_orig[col_map_i]))
				shutil.copy(os.path.join(data_path, 'map_seg_mask', seg_map), os.path.join(data_path, 'data_texture', split_folders[0], 'map_seg_mask', seg_map))
				shutil.copy(os.path.join(data_path, 'waypoints', str(seg_map.split('w')[0][0:-1]) + str('.txt')), os.path.join(data_path, 'data_texture', split_folders[0], 'waypoints', str(seg_map.split('w')[0][0:-1]) + str('.txt')))
			else:
				shutil.copy(os.path.join(data_path, 'color_map_image', col_orig[col_map_i]), os.path.join(data_path, 'data_texture', split_folders[1], 'color_map_image', col_orig[col_map_i]))
				shutil.copy(os.path.join(data_path, 'map_seg_mask', seg_map), os.path.join(data_path, 'data_texture', split_folders[1], 'map_seg_mask', seg_map))
				shutil.copy(os.path.join(data_path, 'waypoints', str(seg_map.split('w')[0][0:-1]) + str('.txt')), os.path.join(data_path, 'data_texture', split_folders[1], 'waypoints', str(seg_map.split('w')[0][0:-1]) + str('.txt')))

