import os
import os.path
import shutil

data_path = '/scratch/wheelchair/annotated_map_dataset/dataset_generation/data'

directories = ['train', 'val']
split_ratio = [0.75, 0.25]   # train, val, test
data_folders = ['color_map_image', 'map_seg_mask', 'waypoints']
prev_count = 0
count = 0 
for i in range(len(directories)):
    path = ''
    count = count + split_ratio[i] * len(os.listdir(data_path+'/color_map_image'))
    for j in range(len(data_folders)):
        # create the train/test/val directory
        path = os.path.join(data_path,'data_texture')
        if not os.path.exists(path):
            os.mkdir(path)
        path = os.path.join(data_path,'data_texture', directories[i])
        if not os.path.exists(path):
            os.mkdir(path)

        # create the color_map etc directory
        target_path = os.path.join(data_path, data_folders[j])
        targetf = [name for name in os.listdir(target_path) if os.path.isfile(os.path.join(target_path, name))]
        # color_len = len(targetf)

        path = os.path.join(path, data_folders[j])
        if not os.path.exists(path):
            os.mkdir(path)
        # count = split_ratio[i] * len(targetf)

        # copy files
        for f in targetf:
            print(f.split('_'))
            if int(f.split('_')[1]) <= count and int(f.split('_')[1]) > prev_count:
                shutil.copy2(os.path.join(target_path, f), path)

    print("Directory '% s' created" % directories[i])
    prev_count = count
