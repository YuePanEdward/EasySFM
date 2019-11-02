//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <chrono>
#include <iostream>

#include "feature_matching.h"

using namespace p3dv;

bool FeatureMatching::DetectFeatures(frame_t &cur_frame, bool show)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("ORB");
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("ORB");

    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
    detector->detect(cur_frame.rgb_image, cur_frame.keypoints);

    descriptor->compute(cur_frame.rgb_image, cur_frame.keypoints, cur_frame.descriptors);

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "extract ORB cost = " << time_used.count() << " seconds. " << std::endl;
    std::cout << "Found "<< cur_frame.descriptors.size() <<" feature points."<< std::endl;
    if (show)
    {
        cv::Mat feature_image;
        cv::drawKeypoints(cur_frame.rgb_image, cur_frame.keypoints, feature_image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("ORB features", feature_image);
        cv::waitKey(0); // Wait for a keystroke in the window
    }

    return true;
}

bool FeatureMatching::MatchFeatures(frame_t &cur_frame_1, frame_t &cur_frame_2, std::vector<cv::DMatch> &matches, bool show)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> initial_matches;

    // Initial match
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
    matcher->match(cur_frame_1.descriptors, cur_frame_2.descriptors, initial_matches);
    std::cout<<"Initial matching done."<<std::endl;
    
    // Filter the correspondences
    auto get_min_max = minmax_element(initial_matches.begin(), initial_matches.end(),
                                      [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dis = get_min_max.first->distance;
    double max_dis = get_min_max.second->distance;

    printf("-- Max dist : %f \n", max_dis);
    printf("-- Min dist : %f \n", min_dis);

    for (int i = 0; i < cur_frame_1.descriptors.rows; i++)
    {
        if (initial_matches[i].distance <= std::max(2 * min_dis, 30.0))
        {
            matches.push_back(initial_matches[i]);
        }
    }
    
    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "match ORB cost = " << time_used.count() << " seconds. " << std::endl;

    if (show)
    {
        cv::Mat initial_match_image;
        cv::Mat match_image;
        cv::drawMatches(cur_frame_1.rgb_image, cur_frame_1.keypoints, cur_frame_2.rgb_image, cur_frame_2.keypoints, initial_matches, initial_match_image);
        cv::drawMatches(cur_frame_1.rgb_image, cur_frame_1.keypoints, cur_frame_2.rgb_image, cur_frame_2.keypoints, matches, match_image);
        cv::imshow("Initial matches", initial_match_image);
        cv::imshow("Filtered matches", match_image);
        cv::waitKey(0);
    }

    return true;
}

bool FeatureMatching::ImportImages(frame_t &cur_frame, bool show)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
    cur_frame.rgb_image = cv::imread(cur_frame.image_file_path, CV_LOAD_IMAGE_COLOR); // Read the file

    if (!cur_frame.rgb_image.data)
    { // Check for invalid input
        std::cout << "No more images" << std::endl;
        return false;
    }
    else
    {
        std::cout << "Import Image [ " << cur_frame.image_file_path << " ] done." << std::endl;
    }
    
    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Import Image [ " << cur_frame.image_file_path << " ] done in " << time_used.count() << " seconds. " << std::endl;

    if (show)
    {
        cv::namedWindow("Show window", cv::WINDOW_AUTOSIZE); // Create a window for display.
        cv::imshow("Show window", cur_frame.rgb_image);  // Show our image inside it.
        std::cout << "Show frame " << cur_frame.frame_id << std::endl;
        cv::waitKey(0); // Wait for a keystroke in the window
    }

    return true;
}
// bool

//     void extract_keypoints(img1, img2, feature_type, match_strategy, hessian_threshold = 800, nn_ratio = 0.7)
// {
// }

//     if feature_type == 'SURF':
//         # Detect keypoints and extract the SURF descriptors
//         surf = cv2.xfeatures2d.SURF_create(hessian_threshold)
//         (kps1, descs1) = surf.detectAndCompute(img1, None)
//         (kps2, descs2) = surf.detectAndCompute(img2, None)
//     elif feature_type == 'SIFT':
//         # Detect keypoints and extract the SIFT descriptors
//         sift = cv2.xfeatures2d.SIFT_create(hessian_threshold)
//         (kps1, descs1) = sift.detectAndCompute(img1, None)
//         (kps2, descs2) = sift.detectAndCompute(img2, None)

//     print('Extract',feature_type,'feature done')
//     print('[Img 2] # kps: {}, descriptors: {}'.format(len(kps2), descs2.shape))
//     print('[Img 1] # kps: {}, descriptors: {}'.format(len(kps1), descs1.shape))

//     # Feature Matching
//     if match_strategy == 'mutual_nn':
//         # Method 1: Mutual nearest neighbor
//         bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
//         matches = bf.match(descs1, descs2)
//         matches= sorted(matches, key = lambda x:x.distance, reverse=False) # small to big
//         matching_result = cv2.drawMatches(img1, kps1, img2, kps2, matches[:50:], None, flags=2)

//     elif match_strategy == 'ratio_test':
//         # Method 2: Nearest neighbor with ratio test
//         bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
//         matches_2nn = bf.knnMatch(descs1, descs2, k=2)
//         # Apply ratio test
//         matches = []
//         for m, n in matches_2nn:
//             if m.distance < nn_ratio * n.distance:
//                 matches.append([m])
//         matching_result = cv2.drawMatchesKnn(img1, kps1, img2, kps2, matches[:50:], None, flags=2)

//     print('Get',len(matches),'correspondences using',match_strategy,'matching strategy')

// }
