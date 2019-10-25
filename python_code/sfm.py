# Test of feature detection and matching
# Import dependence
import os
import cv2
print('OpenCV version:', cv2.__version__)
import numpy as np
from matplotlib import pyplot as plt
import tensorflow as tf
from itertools import combinations
from feature_match import pairwise_match


data_path = '../test_data/images_25/'

files = []
for r, d, f in os.walk(data_path):
    for file in f:
        if '.png' in file:
            files.append(os.path.join(r, file))

# Import Images
imgs=[cv2.imread(file,cv2.IMREAD_GRAYSCALE) for file in files]
print('Import', len(imgs),'images.')

# Pairwise feature matching between all the images
img_pairs=list(combinations(imgs,2))

feature_type='SURF'
match_strategy='ratio_test'

matches = [pairwise_match(img_pair[0],img_pair[1],feature_type,match_strategy) for img_pair in img_pairs]
print('Matches shape', len(matches))
