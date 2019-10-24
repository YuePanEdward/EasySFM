# Test of feature detection and matching
# Import dependence
import cv2
print('OpenCV version:', cv2.__version__)
import numpy as np
from matplotlib import pyplot as plt
import tensorflow as tf

# Import Images
img1 = cv2.imread('../test_data/images_25/0000.png',cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread('../test_data/images_25/0001.png',cv2.IMREAD_GRAYSCALE)

# Detect keypoints and extract the SURF descriptors
hessian_threshold=800
surf = cv2.xfeatures2d.SURF_create(hessian_threshold)
(kps1, descs1) = surf.detectAndCompute(img1, None)
print('[Img 1] # kps: {}, descriptors: {}'.format(len(kps1), descs1.shape))
(kps2, descs2) = surf.detectAndCompute(img2, None)
print("[Img 2] # kps: {}, descriptors: {}".format(len(kps2), descs2.shape))

# Brute-Force Matching
bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
matches = bf.match(descs1, descs2)
matches = sorted(matches, key = lambda x:x.distance, reverse=False) # small to big

matching_result = cv2.drawMatches(img1, kps1, img2, kps2, matches[ :50: ], None, flags=2)

# Show the result
img_kp_1 = cv2.drawKeypoints(img1, kps1, None,(0,0,255),4)
img_kp_2 = cv2.drawKeypoints(img2, kps2, None,(255,0,0),4)

cv2.imshow("Img1", img_kp_1)
cv2.imshow("Img2", img_kp_2)
cv2.imshow("Matching result", matching_result)
cv2.waitKey(0)
cv2.destroyAllWindows()