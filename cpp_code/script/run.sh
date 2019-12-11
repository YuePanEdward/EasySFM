image_folder=/media/edward/BackupPlus/Data/sfm_data/zurich_urban/test_img;
image_list=/media/edward/BackupPlus/Data/sfm_data/zurich_urban/sub_image_list.txt;
calib_file=/media/edward/BackupPlus/Data/sfm_data/zurich_urban/Calibration/K.txt;
output_folder=../test_data/output;
feature_type=S; # S for SURF an O for ORB
ba_frequency=5; # Do Bundle Adjustment each X frame

#gdb --args \ # Debug mode
./bin/sfm ${image_folder} ${image_list} ${calib_file} ${output_folder} ${feature_type} ${ba_frequency} # Release mode