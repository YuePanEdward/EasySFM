image_folder=../test_data/images_25;
image_list=../test_data/image_list.txt;
calib_file=../test_data/k_25/K.txt;
output_folder=../test_data/output;
feature_type=S; # S for SURF an O for ORB

#gdb --args \
./bin/sfm ${image_folder} ${image_list} ${calib_file} ${output_folder} ${feature_type}