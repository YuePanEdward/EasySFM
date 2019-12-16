#data path
image_folder=../test_data/images;
image_list=../test_data/image_list.txt;
calib_k_file=../test_data/k/K.txt;
calib_distort_file=none; # not neccessary, if not provided, just set it as none
output_file=../test_data/output/sfm_sparse_point_cloud_fountain.ply;

#processing parameters
feature_type=S;                 # S for SURF an O for ORB
feature_parameter=300;          # For SURF, this is the minHessian, for ORB, this is the max number of features [int]
repro_dis_ransac=1.0;           # The initial value of reprojection distance threshold for RANSAC (Initialization 5points and PnP)
ba_calib_change_tolerance=0;  # How much can the calib matrix change when doing BA
ba_frequency=4;                 # Do Bundle Adjustment for each X frame

#display parameters
launch_viewer=1;                # Open the real-time viewer (2: display all the processing details, 1: display neccessary details, 0: only dispaly the final result)
view_sphere=0;                  # render the point as sphere during visualization (sphere 1, point 0)

# Debug mode
#gdb --args \  
./bin/sfm ${image_folder} ${image_list} ${calib_k_file} ${calib_distort_file} ${output_file} \
${feature_type} ${feature_parameter} ${repro_dis_ransac} ${ba_calib_change_tolerance} ${ba_frequency} \
${launch_viewer} ${view_sphere}  
