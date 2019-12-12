#data path
image_folder=../test_data/images;
image_list=../test_data/image_list.txt;
calib_k_file=../test_data/k/K.txt;
calib_distort_file=none; # not neccessary, if not provided, just set it as none
output_folder=../test_data/output;

#processing parameters
feature_type=S; # S for SURF an O for ORB
ba_frequency=4; # Do Bundle Adjustment each X frame

#display parameters
view_sphere=0; # render the point as sphere during visualization (sphere 1, point 0)

#gdb --args \ # Debug mode
./bin/sfm ${image_folder} ${image_list} ${calib_k_file} ${calib_distort_file} ${output_folder} ${feature_type} ${ba_frequency} ${view_sphere}  