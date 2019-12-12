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
    std::string use_feature = argv[6];
    std::string ba_frequency = argv[7];
    std::string launch_viewer_or_not = argv[8];
    std::string view_sphere_or_not = argv[9];

    char using_feature = use_feature.c_str()[0];     //Use SURF (S) or ORB (O) feature
    int frequency_BA = stoi(ba_frequency);           //frequency of doing BA.
    bool launch_viewer = stoi(launch_viewer_or_not); //Launch the real-time viewer
    bool view_sphere = stoi(view_sphere_or_not);     //Render point cloud as sphere or just point

    std::vector<frame_t> frames;

    ifstream image_list_file(image_list_path.c_str(), std::ios::in);
    if (!image_list_file.is_open())
    {
        std::cout << "open lidar_file_list failed, file is: " << image_list_path << std::endl;
    }

    int count = 0;
    while (image_list_file.peek() != EOF)
    {
        std::string cur_file;
        image_list_file >> cur_file;
        if (!cur_file.empty())
        {
            cur_file = image_data_path + "/" + cur_file;
            frame_t cur_frame(count, cur_file);
            frames.push_back(cur_frame);
            cout << cur_file << endl;
            count++;
        }
    }

    int frame_number = frames.size();
    std::cout << "Frame number is " << frame_number << std::endl;

    //Class used
    DataIO io;
    FeatureMatching fm;
    MotionEstimator ee;
    MapViewer mv;

    Eigen::Matrix3f K_mat;
    cv::Mat distort_coeff = cv::Mat::zeros(1, 4, CV_64FC1);
    io.importCalib(calib_file_path, K_mat);
    if (!io.importDistort(distort_file_path, distort_coeff))
        std::cout << "No distortion coefficients imported. Use defualt one (0)." << std::endl;

    std::vector<std::vector<frame_pair_t>> img_match_graph;

    pointcloud_sparse_t sfm_sparse_points;
    //sfm_sparse_points.rgb_pointcloud=pcl::PointCloud<pcl::PointXYZRGB>::Ptr();

    // Detect feature points in all the images
    cout << "Begin feature extraction" << endl;
    int keypoints_total_count = 0;
    for (int i = 0; i < frame_number; i++)
    {
        //Import images
        io.importImages(frames[i], false);

        //Set K
        frames[i].K_cam = K_mat;

        //undistort the images
        ee.doUnDistort(frames[i], distort_coeff);

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
        case 'S':
            fm.detectFeaturesSURF(frames[i]);
            break;
        case 'O':
            fm.detectFeaturesORB(frames[i]);
            break;
        default:
            std::cout << "Wrong feature input. Use ORB as default feature." << std::endl;
            fm.detectFeaturesORB(frames[i]);
        }
        keypoints_total_count += frames[i].keypoints.size();
        frames[i].init_pixel_ids();
    }
    std::cout << "Feature extraction done" << std::endl;

    // Match feature points
    std::cout << "Begin pairwise feature matching" << std::endl;

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
            double relative_depth = 1;

            switch (using_feature)
            {
            case 'S':
                fm.matchFeaturesSURF(frames[i], frames[j], temp_matches);
                break;
            case 'O':
                fm.matchFeaturesORB(frames[i], frames[j], temp_matches);
                break;
            default:
                cout << "Wrong feature input. Use ORB as default feature." << endl;
                fm.matchFeaturesORB(frames[i], frames[j], temp_matches);
            }

            if (temp_matches.size() > num_min_pair)
            {
                ee.estimate2D2D_E5P_RANSAC(frames[i], frames[j], temp_matches, inlier_matches, T_mat);
                ee.getDepthFast(frames[i], frames[j], T_mat, inlier_matches, relative_depth);
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
                        frames[i].unique_pixel_ids[inlier_matches[k].queryIdx] = frames[j].unique_pixel_ids[inlier_matches[k].trainIdx];
                }
            }

            frame_pair_t temp_pair(i, j, inlier_matches, T_mat, relative_depth);

            temp_row_pairs.push_back(temp_pair);

            cout << "Frame [" << i << "] and Frame [" << j << "] matching done." << endl;
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

    //Find frame pair for initialization using feature track
    int init_frame_1, init_frame_2;
    double depth_init; // initial frame pair's relative depth
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

    ee.doTriangulation(frames[init_frame_1], frames[init_frame_2], img_match_graph[init_frame_1][init_frame_2].matches, sfm_sparse_points, 0);

    std::vector<bool> frames_to_process(frames.size(), 1);
    frames_to_process[init_frame_1] = 0;
    frames_to_process[init_frame_2] = 0;

    //Launch the on-fly viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> sfm_viewer(new pcl::visualization::PCLVisualizer("EasySFM viewer"));
    sfm_viewer->setBackgroundColor(255, 255, 255);
    if (launch_viewer)
        mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, sfm_sparse_points, depth_init, 3000, view_sphere);

    //BA of initialization
    BundleAdjustment ba;
    ba.doSFMBA(frames, frames_to_process, sfm_sparse_points);
    std::cout << "BA for initialization done" << std::endl;
    //mv.displaySFM(frames, frames_to_process, sfm_sparse_points, "SfM Initialization with BA", 0);
    if (launch_viewer)
        mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, sfm_sparse_points);

    std::cout << "Now add the next view" << std::endl;

    //SfM adding view
    int frames_to_process_count = frames.size() - 2;
    while (frames_to_process_count > 0)
    {
        int next_frame;

        fm.findNextFrame(feature_track_matrix, frames_to_process, sfm_sparse_points.unique_point_ids, next_frame);
        bool pnp_success = ee.estimate2D3D_P3P_RANSAC(frames[next_frame], sfm_sparse_points);

        cout << "Frame [" << next_frame << "] 's pose: " << endl
             << frames[next_frame].pose_cam << endl;

        for (int i = 0; i < frames.size(); i++)
        {
            if (!frames_to_process[i])
            {
                if (next_frame > i) // frame 1 id should larger than frame 2 id
                    ee.doTriangulation(frames[next_frame], frames[i], img_match_graph[next_frame][i].matches, sfm_sparse_points);
                else
                    ee.doTriangulation(frames[i], frames[next_frame], img_match_graph[i][next_frame].matches, sfm_sparse_points);
            }
        }
        ee.outlierFilter(sfm_sparse_points, !pnp_success);

        frames_to_process[next_frame] = 0;
        frames_to_process_count--;

        if (launch_viewer)
        {
            mv.displayFrame(frames[next_frame]);
            mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, sfm_sparse_points);
        }

        if (frames_to_process_count % frequency_BA == 0)
        {
            ba.doSFMBA(frames, frames_to_process, sfm_sparse_points);
            std::cout << "Temporal BA done." << std::endl;
            // if (launch_viewer)
            //     mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, sfm_sparse_points);
        }
    }

    cout << "Adding all the cameras done." << endl;

    // Do Gloabl BA
    ba.doSFMBA(frames, frames_to_process, sfm_sparse_points);
    std::cout << "Final BA done." << std::endl;
    if (launch_viewer)
        mv.displaySFM_on_fly(sfm_viewer, frames, frames_to_process, sfm_sparse_points, depth_init, 1000000);

    //mv.displaySFM(frames, frames_to_process, sfm_sparse_points, "SfM Result with BA", 0);

    // Filter the final point cloud
    CProceesing<pcl::PointXYZRGB> cp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cp.SORFilter(sfm_sparse_points.rgb_pointcloud, output_pointcloud);

    // Output the sparse point cloud with BA
    std::string output_file = output_file_path;
    io.writePlyFile(output_file, output_pointcloud);

    return 1;
}
