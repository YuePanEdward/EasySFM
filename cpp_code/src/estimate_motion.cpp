//Eigen
#include <Eigen/Core>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <chrono>
#include <iostream>

#include "estimate_motion.h"

using namespace p3dv;

bool MotionEstimator::estimateE8Points(std::vector<cv::KeyPoint> &keypoints1,
                                         std::vector<cv::KeyPoint> &keypoints2,
                                         std::vector<cv::DMatch> &matches,
                                         Eigen::Matrix3f &K,
                                         Eigen::Matrix4f &T)
{
    // std::vector<cv::Point2f> pointset1;
    // std::vector<cv::Point2f> pointset2;

    // for (int i = 0; i < (int)matches.size(); i++)
    // {
    //     pointset1.push_back(keypoints1[matches[i].queryIdx].pt);
    //     pointset2.push_back(keypoints2[matches[i].trainIdx].pt);
    // }

    // cv::Point2d principal_point(K(0,2), K(1,2));
    // double focal_length = (K(0,0)+K(1,1))*0.5;
    // cv::Mat essential_matrix;
    // essential_matrix = cv::findEssentialMat(pointset1, pointset2, focal_length, principal_point);
    // std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;

    // cv::Mat R;
    // cv::Mat t;

    // cv::recoverPose(essential_matrix, pointset1, pointset2, R, t, focal_length, principal_point);

    // Eigen::Matrix3f R_eigen;
    // Eigen::Vector3f t_eigen;
    // cv::cv2eigen(R,R_eigen);
    // cv::cv2eigen(t,t_eigen);
    // T.block(0,0,3,3)=R_eigen;
    // T.block(0,3,3,1)=t_eigen;
    // T(3,0)=0;T(3,1)=0;T(3,2)=0;T(3,3)=1;

    // std::cout << "Transform is " << std::endl
    //      << T << std::endl;
}

bool MotionEstimator::estimateE5Points(std::vector<cv::KeyPoint> &keypoints1,
                                         std::vector<cv::KeyPoint> &keypoints2,
                                         std::vector<cv::DMatch> &matches,
                                         Eigen::Matrix3f &K,
                                         Eigen::Matrix4f &T)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    std::vector<cv::Point2f> pointset1;
    std::vector<cv::Point2f> pointset2;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        pointset1.push_back(keypoints1[matches[i].queryIdx].pt);
        pointset2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    cv::Mat camera_mat;
    cv::eigen2cv(K, camera_mat);

    cv::Mat essential_matrix;

    essential_matrix = cv::findEssentialMat(pointset1, pointset2, camera_mat);

    std::cout << "essential_matrix is " << std::endl
              << essential_matrix << std::endl;

    cv::Mat R;
    cv::Mat t;

    cv::recoverPose(essential_matrix, pointset1, pointset2, camera_mat, R, t);
    
    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Estimate Motion [2D-2D] cost = " << time_used.count() << " seconds. " << std::endl;


    Eigen::Matrix3f R_eigen;
    Eigen::Vector3f t_eigen;
    cv::cv2eigen(R, R_eigen);
    cv::cv2eigen(t, t_eigen);
    T.block(0, 0, 3, 3) = R_eigen;
    T.block(0, 3, 3, 1) = t_eigen;
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;

    std::cout << "Transform is " << std::endl
              << T << std::endl;
}
