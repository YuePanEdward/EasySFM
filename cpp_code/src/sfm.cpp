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
    
    IO io_;
    FeatureMatching fm;
    frame_graph_t img_match_graph(frame_number);
    
    // Detect feature points in all the images
    cout<<"Begin feature extraction"<<endl;
    for (int i = 0; i < frame_number; i++)
    {
        io_.ImportImages(frames[i], false);
        fm.DetectFeatures(frames[i], true);
    }
    cout<<"Feature extraction done"<<endl;

    // Match feature points 
    cout<<"Begin pairwise feature matching"<<endl;
    for (int i = 0; i < frame_number; i++)
    {
        for (int j = 0; j < frame_number; j++)
        {
            if (i != j)
            {   
                fm.MatchFeatures(frames[i], frames[j], img_match_graph.frame_graph[i][j].matches, true);
            }
        }
    }
    cout<<"Pairwise feature matching done"<<endl;


    // // Pairwise featrue matching
    // for (int i = 0; i < image_number; i++)
    // {
    //     for (int j = 0; j < image_number; j++)
    //     {
    //         if (i != j)
    //         {
    //             //-- Import Images
    //             Mat img_1 = imread(images_path[i], CV_LOAD_IMAGE_COLOR);
    //             Mat img_2 = imread(images_path[j], CV_LOAD_IMAGE_COLOR);
    //             assert(img_1.data != nullptr && img_2.data != nullptr);

    //             std::vector<KeyPoint> keypoints_1, keypoints_2;
    //             Mat descriptors_1, descriptors_2;
    //             Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
    //             Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("ORB");
    //             // Ptr<FeatureDetector> detector = ORB::create();
    //             // Ptr<DescriptorExtractor> descriptor = ORB::create();
    //             Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    //             //-- Detect Fast keypoints
    //             chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    //             detector->detect(img_1, keypoints_1);
    //             detector->detect(img_2, keypoints_2);

    //             //-- Calculate Brief
    //             descriptor->compute(img_1, keypoints_1, descriptors_1);
    //             descriptor->compute(img_2, keypoints_2, descriptors_2);
    //             chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    //             chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    //             cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

    //             Mat outimg1;
    //             drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    //             imshow("ORB features", outimg1);

    //             //-- Match ORBs
    //             vector<DMatch> matches;
    //             t1 = chrono::steady_clock::now();
    //             matcher->match(descriptors_1, descriptors_2, matches);
    //             t2 = chrono::steady_clock::now();
    //             time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    //             cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    //             //-- Filter the correspondences
    //             auto min_max = minmax_element(matches.begin(), matches.end(),
    //                                           [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    //             double min_dist = min_max.first->distance;
    //             double max_dist = min_max.second->distance;

    //             printf("-- Max dist : %f \n", max_dist);
    //             printf("-- Min dist : %f \n", min_dist);

    //             std::vector<DMatch> good_matches;
    //             for (int i = 0; i < descriptors_1.rows; i++)
    //             {
    //                 if (matches[i].distance <= max(2 * min_dist, 30.0))
    //                 {
    //                     good_matches.push_back(matches[i]);
    //                 }
    //             }

    //             //-- Draw
    //             Mat img_match;
    //             Mat img_goodmatch;
    //             drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    //             drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    //             imshow("all matches", img_match);
    //             imshow("good matches", img_goodmatch);
    //             waitKey(0);

    //         }
    //     }
    // }

    return 1;
}
