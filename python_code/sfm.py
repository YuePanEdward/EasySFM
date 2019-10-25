# Test of feature detection and matching
# Import dependence
import cv2
print('OpenCV version:', cv2.__version__)
import numpy as np
from matplotlib import pyplot as plt
import tensorflow as tf
from feature_match import pairwise_match

# Import Images
img1 = cv2.imread('../test_data/images_25/0000.png',cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread('../test_data/images_25/0001.png',cv2.IMREAD_GRAYSCALE)

feature_type='SURF'
match_strategy='ratio_test'
pairwise_match(img1,img2,feature_type,match_strategy)
