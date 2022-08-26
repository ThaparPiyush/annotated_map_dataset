# Example: python calc_mean_stddev.py --paths train/color_map_image val/color_map_image

import argparse
import os
from PIL import Image
import numpy as np

parser = argparse.ArgumentParser("Calculate Means and Standard Deviations", add_help=False)
parser.add_argument('--paths', nargs='+', help='<Required> Set flag', required=True)
args = parser.parse_args()

image_paths = []

for path in args.paths:
    image_paths.append(path)
print(f"Calculating means and standard deviations of RGB channels of images present inside the following folders:\n{image_paths}")

images = []
for path in image_paths:
    for image_path in os.listdir(path):
        images.append(np.array(Image.open(path + image_path)))

means = [0, 0, 0]
stds = [0, 0, 0]
for channel in range(0, 3):
    n1 = X_1 = S_1_2 = 0
    for image in images:
        if n1 == 0:
            n1 = image.shape[0]*image.shape[1]
            X_1 = np.mean(image[:,:,channel])
            S_1_2 = np.var(image[:,:,channel])
        else:
            n2 = image.shape[0]*image.shape[1]
            X_2 = np.mean(image[:,:,channel])
            S_2_2 = np.var(image[:,:,channel])
            X_c = (n1*X_1 + n2*X_2)/(n1+n2)
            S_c_2 = (n1*(S_1_2 + (X_1 - X_c)**2) + n2*(S_2_2 + (X_2 - X_c)**2))/(n1+n2)
            n1 += n2
            X_1 = X_c
            S_1_2 = S_c_2
            means[channel] = X_1
            stds[channel] = S_1_2**0.5
print(f"Means RGB: {means}, Standard Deviation RGB: {stds}")
