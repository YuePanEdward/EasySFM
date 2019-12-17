
//
// This Cpp programme is written for the general implement of structure from motion sparse reconstruction
// For the course 'Photogrammetry and 3D Vison Lab' at ETH Zurich
// Dependent 3rd Libs: OpenCV (>=3) with contrib ,PCL (>=1.7), Eigen3, Ceres
// Enviroment: Linux (Ubuntu 16.04) compiled passed
// Project homepage: https://github.com/YuePanEdward/EasySFM
// Please refer to the project homepage for problem on installing, compiling, running and data preparing
// Author: Yue Pan @ ETH Zurich D-BAUG
// Contact: yuepan@student.ethz.ch
// License: MIT License [Copyright (c) 2019 Yue Pan]
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include "utility.h"
#include "data_io.h"
#include "feature_matching.h"
#include "estimate_motion.h"
#include "ba.h"
#include "viewer.h"
#include "cloudprocessing.hpp"

using namespace cv;
using namespace std;
using namespace p3dv;

int main(int argc, char **argv)
{
    // The file to read from.
    std::string image_data_path = argv[1];
    std::string image_list_path = argv[2];
    std::string calib_file_path = argv[3];
    std::string distort_file_path = argv[4];
    std::string output_file_path = argv[5];

    // Key processing and visualization parameters
    std::string use_feature = argv[6];
    std::string feature_extraction_parameter = argv[7];
    std::string ransac_reproj_dist_thre = argv[8];
    std::string find_best_initialization = argv[9];
    std::string ba_fix_calib_tolerance = argv[10];
    std::string ba_frequency = argv[11];

    std::string launch_viewer_or_not = argv[12];
    std::string view_sphere_or_not = argv[13];

    char using_feature = use_feature.c_str()[0]; //Use SURF (S) or ORB (O) feature
    // feature_extract_parameter: if you choose to use SURF , it is the min Hessian for SURF feature, the larger this value is, the less features would be extracted (a reference value would be 300)
    // if you choose to use ORB , it would be the largest number of feature that would be extracted (a reference value would be 8000)
    int feature_extract_parameter = stoi(feature_extraction_parameter);
    double ransac_reproj_distance = stod(ransac_reproj_dist_thre);  //the initial value of reprojection distance threshold for RANSAC (Initialization 5points and PnP). This value may increase before the next BA due to possible error accumulaion.
    bool use_track_frames_as_init = stoi(find_best_initialization); //Find best frame pairs for initialization or not (1: find pair, 0: just use the first two frames for initialization)
    double fix_calib_tolerance_BA = stod(ba_fix_calib_tolerance);   //How much can the calib matrix change when doing BA (Default 0: calib matrix is fixed, others should be positive)
    int frequency_BA = stoi(ba_frequency);                          //frequency of doing BA.
    int launch_viewer = stoi(launch_viewer_or_not);                 //Launch the real-time viewer (2: display all the processing details, 1: display neccessary details, 0: only dispaly the final result)
    bool view_sphere = stoi(view_sphere_or_not);                    //Render point cloud as sphere or just point

    //Functional Classes
    DataIO io;
    FeatureMatching fm;
    MotionEstimator ee;
    MapViewer mv;

    //Frames: basic processing objects: refer to utility.h for detail
    std::vector<frame_t> frames;

    //import image filenames
    io.importImageFilenames(image_list_path, image_data_path, frames);
    int frame_number = frames.size();

    //import calibration file
    Eigen::Matrix3f K_mat;
    cv::Mat distort_coeff = cv::Mat::zeros(1, 4, CV_64FC1);
    io.importCalib(calib_file_path, K_mat);                  // import K matrix ([fx, 0 ,cx; 0, fy, cy; 0 0 1])
    if (!io.importDistort(distort_file_path, distort_coeff)) // import distort coefficences ([k1, k2, p1, p2]) not neccessary
        std::cout << "No distortion coefficients imported. Use defualt one (0)." << std::endl;

    std::vector<std::vector<frame_pair_t>> img_match_graph;
    pointcloud_sparse_t sfm_sparse_points; // Stucture from motion sparse point cloud

    // Detect feature points in all the images
    std::cout << "Begin feature extraction" << std::endl;
    int keypoints_total_count = 0;
    for (int i = 0; i < frame_number; i++) // for each frame
    {
        //Import images
        io.importImages(frames[i], false);

        //Set K
        frames[i].K_cam = K_mat;

        //Undistort the images
        ee.doUnDistort(frames[i], distort_coeff);

        //Use this one if the camera matrix is the same for all the processing images
        if (i == 0)
        {
            std::cout << "Import calibration data done" << std::endl;
            std::cout << "K Matrix:\n"
                      << frames[i].K_cam << std::endl;
        }
        std::cout << "Feature extraction of Frame [ " << i << " ]" << std::endl;

        //Detect keypoints and extract feature
        switch (using_feature)
        {
        case 'S': //Use SURF feature (slower but more accurate), feature_extract_parameter would be the min Hessian (default:350)
            fm.detectFeaturesSURF(frames[i], feature_extract_parameter);
            break;
        case 'O': //Use ORB feature (faster but has some problem), feature_extract_parameter would be the max number (default:5000)
            fm.detectFeaturesORB(frames[i], feature_extract_parameter);
            break;
        default:
            std::cout << "Wrong feature input. Use SURF as default feature." << std::endl;
            fm.detectFeaturesSURF(frames[i], feature_extract_parameter);
        }
        keypoints_total_count += frames[i].keypoints.size();
        frames[i].init_pixel_ids();

        if (launch_viewer > 1)
            mv.displayFrame(frames[i]);
    }
    std::cout << "Feature extraction done" << std::endl;

    // Match feature points
    std::cout << "Begin pairwise feature matching" << std::endl;
    int num_min_pair = 20; // Minimum number of point pair for claiming two images are really overlapping
    int max_total_feature_num = keypoints_total_count;

    //Feature track matrix: row: frames; colum: unique feature points
    std::vector<std::vector<bool>> feature_track_matrix(frame_number, std::vector<bool>(max_total_feature_num, 0));

    int cur_num_unique_points = 0;
    // Pairwise matching
    for (int i = 0; i < frame_number; i++)
    {
        std::vector<frame_pair_t> temp_row_pairs;
        for (int j = 0; j < i; j++)
        {
            std::vector<cv::DMatch> temp_matches;
            std::vector<cv::DMatch> inlier_matches;
            Eigen::Matrix4f T_mat = Eigen::Matrix4f::Identity();
            double relative_depth = 1;

            switch (using_feature)
            {
            case 'S': //You can tune the Lowe test ratio here (default 0.5 for SURF)
                fm.matchFeaturesSURF(frames[i], frames[j], temp_matches);
                break;
            case 'O': //You can tune the Lowe test ratio here (default 0.8 for ORB)
                fm.matchFeaturesORB(frames[i], frames[j], temp_matches);
                break;
            default:
                cout << "Wrong feature input. Use SURF as default feature." << endl;
                fm.matchFeaturesSURF(frames[i], frames[j], temp_matches);
            }

            if (temp_matches.size() > num_min_pair) // Double check with RANSAC
            {
                ee.estimate2D2D_E5P_RANSAC(frames[i], frames[j], temp_matches, inlier_matches, T_mat, ransac_reproj_distance); //Filter the matching by epipolar geometry 5 points RANSAC
                ee.getDepthFast(frames[i], frames[j], T_mat, inlier_matches, relative_depth);                                  // Approximately estimate the mean depth

                if (launch_viewer > 1)
                    mv.displayFrameMatch(frames[i], frames[j], inlier_matches);
            }

            // Assign i frame's keypoints unique id by finding its correspondence in already labeled j frame
            for (int k = 0; k < inlier_matches.size(); k++)
            {
                if (frames[i].unique_pixel_ids[inlier_matches[k].queryIdx] < 0 ||
                    frames[i].unique_pixel_ids[inlier_matches[k].queryIdx] != frames[j].unique_pixel_ids[inlier_matches[k].trainIdx])
                {
                    bool is_duplicated = 0;
                    for (int m = 0; m < frames[i].unique_pixel_ids.size(); m++) // check duplication
                    {
                        if (frames[j].unique_pixel_ids[inlier_matches[k].trainIdx] == frames[i].unique_pixel_ids[m])
                        {
                            is_duplicated = 1;
                            break;
                        }
                    }
                    if (!is_duplicated)
                    {
                        frames[i].unique_pixel_ids[inlier_matches[k].queryIdx] = frames[j].unique_pixel_ids[inlier_matches[k].trainIdx];
                        frames[i].unique_pixel_has_match[inlier_matches[k].queryIdx] = 1;
                    }
                }
            }

            frame_pair_t temp_pair(i, j, inlier_matches, T_mat, relative_depth);

            temp_row_pairs.push_back(temp_pair);

            std::cout << "Frame [" << i << "] and Frame [" << j << "] matching done." << std::endl;
        }

        img_match_graph.push_back(temp_row_pairs);

        //Assign unique id for non-matched keypoints
        int count_new_unique_point = 0;
        for (int k = 0; k < frames[i].unique_pixel_ids.size(); k++)
        {
            if (frames[i].unique_pixel_ids[k] < 0)
            {
                frames[i].unique_pixel_ids[k] = cur_num_unique_points + count_new_unique_point;
                count_new_unique_point++;
            }

            //Update the feature tracking matrix
            feature_track_matrix[i][frames[i].unique_pixel_ids[k]] = 1;
        }
        cur_num_unique_points += count_new_unique_point;
    }
    std::cout << "Pairwise feature matching done." << std::endl;
    std::cout << "Feature tracking done, there are " << cur_num_unique_points << " unique points in total." << std::endl;

    //Find frame pair for initialization using feature track matrix
    int init_frame_1, init_frame_2;
    // This would be applied if you chose to initialize from the first two frames
    init_frame_1 = 1;
    init_frame_2 = 0;
    double depth_init = 10.0; // initial frame pair's relative depth (10.0 set as the default value)

    // Two conditions for the initialization frame pair:
    // 1. baseline length should not be too small (avoid the pure-rotation problem)
    // 2. more common feature tracks would be prefered
    if (use_track_frames_as_init)
        fm.findInitializeFramePair(feature_track_matrix, frames, img_match_graph, init_frame_1, init_frame_2, depth_init);

    //SfM initialization
    frames[init_frame_1].pose_cam = Eigen::Matrix4f::Identity();
    std::cout << "Frame [" << init_frame_1 << "] 's pose: " << std::endl
              << frames[init_frame_1].pose_cam << std::endl;
    if (launch_viewer)
        mv.displayFrame(frames[init_frame_1]);

    frames[init_frame_2].pose_cam = img_match_graph[init_frame_1][init_frame_2].T_21 * frames[init_frame_1].pose_cam;
    std::cout << "Frame [" << init_frame_2 << "] 's pose: " << std::endl
              << frames[init_frame_2].pose_cam << std::endl;
    if (launch_viewer)
        mv.displayFrame(frames[init_frame_2]);
    //Triangulation of the first two frames
    ee.doTriangulation(frames[init_frame_1], frames[init_frame_2], img_match_graph[init_frame_1][init_frame_2].matches, sfm_sparse_points, 0);

    std::vector<bool> frames_to_process(frames.size(), 1); //indicate the frames which has not been processed yet
    frames_to_process[init_frame_1] = 0;                   //processed->0
    frames_to_process[init_frame_2] = 0;                   //processed->0

    //Launch the on-fly viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> sfm_viewer(new pcl::visualization::PCLVisualizer("EasySFM viewer"));
    sfm_viewer->setBackgroundColor(255, 255, 255);
    if (launch_viewer)
        mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, init_frame_1, sfm_sparse_points, depth_init, 8500, view_sphere);

    //BA of initialization
    BundleAdjustment ba;
    ba.doSFMBA(frames, frames_to_process, sfm_sparse_points, fix_calib_tolerance_BA);
    std::cout << "BA for initialization done" << std::endl;
    //mv.displaySFM(frames, frames_to_process, sfm_sparse_points, "SfM Initialization with BA", 0);
    if (launch_viewer > 1)
        mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, init_frame_2, sfm_sparse_points);

    std::cout << "Now register the next frame" << std::endl;

    //SfM keeps registering new frames
    int frames_to_process_count = frames.size() - 2;
    double ransac_reproj_dist = ransac_reproj_distance;

    while (frames_to_process_count > 0) //Till all the frames are processed
    {
        int next_frame;
        //Find the next frame according to the feature tracking matrix
        fm.findNextFrame(feature_track_matrix, frames_to_process, sfm_sparse_points.unique_point_ids, next_frame);
        //Use PnP to register new frame
        bool pnp_success = ee.estimate2D3D_P3P_RANSAC(frames[next_frame], sfm_sparse_points, ransac_reproj_dist);

        //Since BA would not be done every time due to effciency consideration, relax the threshold a bit
        ransac_reproj_dist += 1.0;

        std::cout << "Frame [" << next_frame << "] 's pose: " << std::endl
                  << frames[next_frame].pose_cam << std::endl;

        for (int i = 0; i < frames.size(); i++) //Keep triangulate new 3d points, add to sfm sprase point cloud
        {
            if (!frames_to_process[i])
            {
                if (next_frame > i) // frame 1 id should larger than frame 2 id
                    ee.doTriangulation(frames[next_frame], frames[i], img_match_graph[next_frame][i].matches, sfm_sparse_points);
                else
                    ee.doTriangulation(frames[i], frames[next_frame], img_match_graph[i][next_frame].matches, sfm_sparse_points);
            }
        }
        if (!pnp_success)                        //If PnP encounter some problem (when the mean reprojection error is too big or the inlier ratio is too small)
            ee.outlierFilter(sfm_sparse_points); //Use SOR to filter outlier points

        frames_to_process[next_frame] = 0;
        frames_to_process_count--;

        if (launch_viewer)
        {
            mv.displayFrame(frames[next_frame]);
            mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, next_frame, sfm_sparse_points);
        }

        //Do BA each frequency_BA time
        if (frames_to_process_count % frequency_BA == 0)
        {
            ba.doSFMBA(frames, frames_to_process, sfm_sparse_points, fix_calib_tolerance_BA);
            std::cout << "Temporal BA done." << std::endl;
            if (launch_viewer > 1)
                mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, next_frame, sfm_sparse_points);
            ransac_reproj_dist = ransac_reproj_distance; //re-define the threshold
        }
        std::cout << "Progress: [ " << frames.size() - frames_to_process_count << " / " << frames.size() << " ]" << std::endl;
    }
    std::cout << "Adding all the cameras done." << std::endl;

    // Do Gloabl BA
    ba.doSFMBA(frames, frames_to_process, sfm_sparse_points);
    std::cout << "Final BA done." << std::endl;
    //Visualize the final result
    mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, -1, sfm_sparse_points, depth_init, 1000000);

    // Filter the final point cloud
    CProceesing<pcl::PointXYZRGB> cp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cp.SORFilter(sfm_sparse_points.rgb_pointcloud, output_pointcloud);

    // Output the sparse point cloud to the output folder in ply format, you can further visualize it better using softwares like CloudCompare or MeshLab
    std::string output_file = output_file_path;
    io.writePlyFile(output_file, output_pointcloud);

    return 1;
}

// TO DO List
// Add a image semantic segmentation frontend using some pre-trained facade semantic segmentation model
// Assign the point with the corresponding semantic label of the pixel
// Also use the semantic label to better do initialization and PnP
// For the generated point cloud, use pre-trained point cloud semantic segmentation neural network's reuslt to supervised the alreday got semantic label
