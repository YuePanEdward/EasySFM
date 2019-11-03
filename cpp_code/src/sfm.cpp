#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include "utility.h"
#include "data_io.h"
#include "feature_matching.h"


using namespace cv;
using namespace std;
using namespace p3dv;

int main(int argc, char **argv)
{
    // The file to read from.
    string data_path = argv[1];
    string image_list_path = argv[2];

    vector<frame_t> frames;

    ifstream image_list_file(image_list_path.c_str(), std::ios::in);
    if (!image_list_file.is_open())
    {
        cout << "open lidar_file_list failed, file is: " << image_list_path;
    }

    int count=0;
    while (image_list_file.peek() != EOF)
    {
        string cur_file;
        image_list_file >> cur_file;
        cur_file = data_path + "/" + cur_file;
        frame_t cur_frame(count, cur_file);
        frames.push_back(cur_frame);
        cout<<cur_file<<endl;
        count++;
    }

    int frame_number = frames.size();
    cout<<"Frame number is "<< frame_number<<endl;
    
    DataIO io;
    FeatureMatching fm;
    vector<vector<frame_pair_t>> img_match_graph;
    
    // Detect feature points in all the images
    cout<<"Begin feature extraction"<<endl;
    for (int i = 0; i < frame_number; i++)
    {
        io.ImportImages(frames[i], false);
        fm.DetectFeaturesORB(frames[i]);
    }
    cout<<"Feature extraction done"<<endl;

    // Match feature points 
    cout<<"Begin pairwise feature matching"<<endl;
    
    for (int i = 0; i < frame_number; i++)
    {
        std::vector<frame_pair_t> temp_row_pairs;
        for (int j = 0; j < frame_number; j++)
        {
            std::vector<cv::DMatch> temp_matches;
            if (i < j)
            {   
                fm.MatchFeaturesORB(frames[i], frames[j], temp_matches);
            }
            frame_pair_t temp_pair(i,j,temp_matches);

            temp_row_pairs.push_back(temp_pair);
        }
        img_match_graph.push_back(temp_row_pairs);
    }
    cout<<"Pairwise feature matching done"<<endl;

    return 1;
}
