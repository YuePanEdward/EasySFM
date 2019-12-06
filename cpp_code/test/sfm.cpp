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

using namespace cv;
using namespace std;
using namespace p3dv;

int main(int argc, char **argv)
{
    // The file to read from.
    string image_data_path = argv[1];
    string image_list_path = argv[2];
    string calib_file_path = argv[3];
    string output_file_path = argv[4];
    string use_feature = argv[5];
    char using_feature = use_feature.c_str()[0];

    vector<frame_t> frames;

    ifstream image_list_file(image_list_path.c_str(), std::ios::in);
    if (!image_list_file.is_open())
    {
        cout << "open lidar_file_list failed, file is: " << image_list_path;
    }

    int count = 0;
    while (image_list_file.peek() != EOF)
    {
        string cur_file;
        image_list_file >> cur_file;
        cur_file = image_data_path + "/" + cur_file;
        frame_t cur_frame(count, cur_file);
        frames.push_back(cur_frame);
        cout << cur_file << endl;
        count++;
    }

    int frame_number = frames.size();
    cout << "Frame number is " << frame_number << endl;

    DataIO io;
    Eigen::Matrix3f K_mat;
    io.importCalib(calib_file_path, K_mat);

    FeatureMatching fm;
    vector<vector<frame_pair_t>> img_match_graph;

    MotionEstimator ee;
    
    // Detect feature points in all the images
    cout << "Begin feature extraction" << endl;
    int keypoints_total_count=0;
    for (int i = 0; i < frame_number; i++)
    {
        //Import images
        io.importImages(frames[i], false);

        //Set K
        frames[i].K_cam = K_mat;
        
        //Detect keypoints and extract feature
        switch (using_feature)
        {
        case 'S':
            fm.detectFeaturesSURF(frames[i]);
            break;
        case 'O':
            fm.detectFeaturesORB(frames[i],0);
            break;
        default:
            cout << "Wrong feature input. Use ORB as default feature." << endl;
            fm.detectFeaturesORB(frames[i]);
        }
        keypoints_total_count+=frames[i].keypoints.size();

    }
    cout << "Feature extraction done" << endl;

    // Match feature points
    cout << "Begin pairwise feature matching" << endl;

    int num_min_pair = 15;
    int max_total_feature_num = keypoints_total_count;

    std::vector<std::vector<bool>> feature_track_matrix(frame_number, std::vector<bool>(max_total_feature_num, 0));
    int cur_num_unique_points = 0;

    for (int i = 0; i < frame_number; i++)
    {
        std::vector<frame_pair_t> temp_row_pairs;
        for (int j = 0; j < i; j++)
        {
            std::vector<cv::DMatch> temp_matches;
            std::vector<cv::DMatch> inlier_matches;
            Eigen::Matrix4f T_mat = Eigen::Matrix4f::Identity();

            switch (using_feature)
            {
            case 'S':
                fm.matchFeaturesSURF(frames[i], frames[j], temp_matches, 0.75, 0);
                break;
            case 'O':
                fm.matchFeaturesORB(frames[i], frames[j], temp_matches, 0.75, 0);
                break;
            default:
                cout << "Wrong feature input. Use ORB as default feature." << endl;
                fm.matchFeaturesORB(frames[i], frames[j], temp_matches);
            }

            if (temp_matches.size() > num_min_pair)
            {
                ee.estimate2D2D_E5P_RANSAC(frames[i], frames[j], temp_matches, inlier_matches, K_mat, T_mat);
            }

            // Assign i frame's keypoints unique id by finding its correspondence in already labeled j frame
            for (int k = 0; k < inlier_matches.size(); k++)
            {
                frames[i].unique_pixel_ids[inlier_matches[k].queryIdx] = frames[j].unique_pixel_ids[inlier_matches[k].trainIdx];
            }

            frame_pair_t temp_pair(i, j, inlier_matches, T_mat);
            temp_row_pairs.push_back(temp_pair);
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
    cout << "Pairwise feature matching done" << endl;
    cout << "Feature tracking done, there are " << cur_num_unique_points << " unique points in total." << endl;

    //Find frame pair for initialization using feature track
    int init_frame_1;
    int init_frame_2;

    fm.findInitializeFramePair(feature_track_matrix,frames,cur_num_unique_points,init_frame_1,init_frame_2);

    //SfM initialization
    //frames[init_frame_1].pose_cam = Eigen::Matrix4f::Identity();
    //frames[init_frame_2].pose_cam = img_match_graph[init_frame_1][init_frame_2].T_21 * frames[init_frame_1].pose_cam;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    ee.doTriangulation(frames[init_frame_1], frames[init_frame_2], img_match_graph[init_frame_1][init_frame_2].matches, temp_cloud);

    return 1;
}

// test
//frames[i].pose_cam = Eigen::Matrix4f::Identity();
//frames[j].pose_cam = T_mat * frames[i].pose_cam;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//ee.doTriangulation(frames[i], frames[j], inlier_matches, temp_cloud);
