#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include "utility.h"


using namespace cv;
using namespace std;
using namespace p3dv;

int main(int argc, char **argv)
{
    // The file to read from.
    string data_path = argv[1];
    string image_list_path = argv[2];

    vector<string> images_path;

    ifstream image_list_file(image_list_path.c_str(), std::ios::in);
    if (!image_list_file.is_open())
    {
        cout << "open lidar_file_list failed, file is: " << image_list_path;
    }

    while (image_list_file.peek() != EOF)
    {
        string cur_file;
        image_list_file >> cur_file;
        cur_file = data_path + "/" + cur_file;
        images_path.push_back(cur_file);
    }

    int image_number = images_path.size();

    for (int i = 0; i < image_number; i++)
    {
        Mat cur_image;
        cur_image = imread(images_path[i], CV_LOAD_IMAGE_COLOR); // Read the file
        if (!cur_image.data)
        { // Check for invalid input
            cout << "No more images" << endl;
            break;
        }
        else
        {
            cout << "Import Image [ " << images_path[i] << " ] done." << endl;
        }

        namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
        imshow("Display window", cur_image);            // Show our image inside it.

        waitKey(0); // Wait for a keystroke in the window
    }

    // Pairwise featrue matching
    for (int i = 0; i < image_number; i++)
    {
        for (int j = 0; j < image_number; j++)
        {
            if (i != j)
            {   
                //-- Import Images
                Mat img_1 = imread(images_path[i], CV_LOAD_IMAGE_COLOR);
                Mat img_2 = imread(images_path[j], CV_LOAD_IMAGE_COLOR);
                assert(img_1.data != nullptr && img_2.data != nullptr);
                
                std::vector<KeyPoint> keypoints_1, keypoints_2;
                Mat descriptors_1, descriptors_2;
                Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
                Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("ORB");
                // Ptr<FeatureDetector> detector = ORB::create();
                // Ptr<DescriptorExtractor> descriptor = ORB::create();
                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

                //-- Detect Fast keypoints
                chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
                detector->detect(img_1, keypoints_1);
                detector->detect(img_2, keypoints_2);

                //-- Calculate Brief
                descriptor->compute(img_1, keypoints_1, descriptors_1);
                descriptor->compute(img_2, keypoints_2, descriptors_2);
                chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
                chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
                cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

                Mat outimg1;
                drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
                imshow("ORB features", outimg1);

                //-- Match ORBs
                vector<DMatch> matches;
                t1 = chrono::steady_clock::now();
                matcher->match(descriptors_1, descriptors_2, matches);
                t2 = chrono::steady_clock::now();
                time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
                cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

                //-- Filter the correspondences
                auto min_max = minmax_element(matches.begin(), matches.end(),
                                              [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
                double min_dist = min_max.first->distance;
                double max_dist = min_max.second->distance;

                printf("-- Max dist : %f \n", max_dist);
                printf("-- Min dist : %f \n", min_dist);

                std::vector<DMatch> good_matches;
                for (int i = 0; i < descriptors_1.rows; i++)
                {
                    if (matches[i].distance <= max(2 * min_dist, 30.0))
                    {
                        good_matches.push_back(matches[i]);
                    }
                }

                //-- Draw
                Mat img_match;
                Mat img_goodmatch;
                drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
                drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
                imshow("all matches", img_match);
                imshow("good matches", img_goodmatch);
                waitKey(0);

            }
        }
    }
}
