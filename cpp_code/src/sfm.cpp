#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include "utility.h"
#include "data_io.h"
#include "feature_matching.h"
#include "estimate_motion_eg.h"

using namespace cv;
using namespace std;
using namespace p3dv;

int main(int argc, char **argv)
{
    // The file to read from.
    string image_data_path = argv[1];
    string image_list_path = argv[2];
    string calib_file_path = argv[3];
    string use_feature = argv[4];
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

    EpipolarEstimator ee;

    // Detect feature points in all the images
    cout << "Begin feature extraction" << endl;
    for (int i = 0; i < frame_number; i++)
    {
        io.importImages(frames[i], false);
        switch (using_feature)
        {
        case 'S':
            fm.detectFeaturesSURF(frames[i]);
            break;
        case 'O':
            fm.detectFeaturesORB(frames[i]);
            break;
        default:
            cout << "Wrong feature input. Use ORB as default feature." << endl;
            fm.detectFeaturesORB(frames[i]);
        }
    }
    cout << "Feature extraction done" << endl;

    // Match feature points
    cout << "Begin pairwise feature matching" << endl;

    for (int i = 0; i < frame_number; i++)
    {
        std::vector<frame_pair_t> temp_row_pairs;
        for (int j = 0; j < frame_number; j++)
        {
            std::vector<cv::DMatch> temp_matches;
            Eigen::Matrix4f T_mat = Eigen::Matrix4f::Identity();
            if (i < j)
            {
                switch (using_feature)
                {
                case 'S':
                    fm.matchFeaturesSURF(frames[i], frames[j], temp_matches, 0.7, false);
                    break;
                case 'O':
                    fm.matchFeaturesORB(frames[i], frames[j], temp_matches, 0.7, true);
                    break;
                default:
                    cout << "Wrong feature input. Use ORB as default feature." << endl;
                    fm.matchFeaturesORB(frames[i], frames[j], temp_matches);
                }
                
                if (temp_matches.size() > 15)
                {
                    ee.estimateE8Points(frames[i].keypoints, frames[j].keypoints, temp_matches, K_mat, T_mat);
                }
            }
            frame_pair_t temp_pair(i, j, temp_matches, T_mat);

            temp_row_pairs.push_back(temp_pair);
        }
        img_match_graph.push_back(temp_row_pairs);
    }
    cout << "Pairwise feature matching done" << endl;

    return 1;
}
