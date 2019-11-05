//Eigen
#include <Eigen/Core>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

bool MotionEstimator::estimateE5PRANSAC(frame_t &cur_frame_1, frame_t &cur_frame_2,
                                        std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &inlier_matches,
                                        Eigen::Matrix3f &K, Eigen::Matrix4f &T,
                                        double ransac_prob, double ransac_thre, bool show)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    std::vector<cv::Point2f> pointset1;
    std::vector<cv::Point2f> pointset2;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        pointset1.push_back(cur_frame_1.keypoints[matches[i].queryIdx].pt);
        pointset2.push_back(cur_frame_2.keypoints[matches[i].trainIdx].pt);
    }

    cv::Mat camera_mat;
    cv::eigen2cv(K, camera_mat);

    cv::Mat essential_matrix;

    cv::Mat inlier_matches_indicator;

    essential_matrix = cv::findEssentialMat(pointset1, pointset2, camera_mat,
                                            CV_RANSAC, ransac_prob, ransac_thre, inlier_matches_indicator);

    std::cout << "essential_matrix is " << std::endl
              << essential_matrix << std::endl;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        if (inlier_matches_indicator.at<bool>(0, i) == 1)
        {
            inlier_matches.push_back(matches[i]);
        }
    }

    cv::Mat R;
    cv::Mat t;

    cv::recoverPose(essential_matrix, pointset1, pointset2, camera_mat, R, t, inlier_matches_indicator);

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Estimate Motion [2D-2D] cost = " << time_used.count() << " seconds. " << std::endl;
    std::cout << "Find [" << inlier_matches.size() << "] inlier matches from [" << matches.size() << "] total matches." << std::endl;

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

    if (show)
    {
        cv::Mat ransac_match_image;
        cv::drawMatches(cur_frame_1.rgb_image, cur_frame_1.keypoints, cur_frame_2.rgb_image, cur_frame_2.keypoints, inlier_matches, ransac_match_image);
        cv::imshow("RANSAC inlier matches", ransac_match_image);
        cv::waitKey(0);
    }
}

void MotionEstimator::doTriangulation(frame_t &cur_frame_1, frame_t &cur_frame_2,
                                      const std::vector<cv::DMatch> &matches,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sparse_pointcloud)
{
    cv::Mat T1_mat;
    cv::Mat T2_mat;
    cv::Mat camera_mat;

    Eigen::Matrix<float,3,4> T1 = cur_frame_1.pose_cam.block(0,0,3,4);
    Eigen::Matrix<float,3,4> T2 = cur_frame_2.pose_cam.block(0,0,3,4);

    cv::eigen2cv(cur_frame_1.K_cam, camera_mat);
    cv::eigen2cv(T1, T1_mat);
    cv::eigen2cv(T2, T2_mat);

    std::vector<cv::Point2f> pointset1;
    std::vector<cv::Point2f> pointset2;

    for (cv::DMatch m : matches)
    {
        pointset1.push_back(pixel2cam(cur_frame_1.keypoints[m.queryIdx].pt, camera_mat));
        pointset2.push_back(pixel2cam(cur_frame_1.keypoints[m.trainIdx].pt, camera_mat));
    }

    cv::Mat pts_3d_homo;
    cv::triangulatePoints(T1_mat, T2_mat, pointset1, pointset2, pts_3d_homo);

    // De-homo
    for (int i = 0; i < pts_3d_homo.cols; i++)
    {
        cv::Mat pts_3d = pts_3d_homo.col(i);
        pts_3d /= pts_3d.at<float>(3, 0);

        pcl::PointXYZRGB pt_temp;
        pt_temp.x = pts_3d.at<float>(0, 0);
        pt_temp.y = pts_3d.at<float>(1, 0);
        pt_temp.z = pts_3d.at<float>(2, 0);
        pt_temp.r=255;
        pt_temp.g=0;
        pt_temp.b=0;
        sparse_pointcloud->points.push_back(pt_temp);
    }

    std::cout<< "Generate new sparse point cloud done." <<std::endl;
}
