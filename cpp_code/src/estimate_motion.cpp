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
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

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

bool MotionEstimator::estimate2D2D_E5P_RANSAC(frame_t &cur_frame_1, frame_t &cur_frame_2,
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

bool MotionEstimator::estimate2D3D_P3P_RANSAC(frame_t &cur_frame, pointcloud_sparse_t &cur_map_3d, std::vector<cv::DMatch> &matches,
                                              bool show)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    std::vector<cv::DMatch> good_matches;

    //std::vector<cv::DMatch> good_matches;

    std::vector<cv::Point2f> pointset2d;
    std::vector<cv::Point3f> pointset3d;

    // Construct the 2d-3d initial matchings

    // Assign value for pointset2d and pointset3d

    // Use RANSAC P3P to estiamte the optimal transformation
    int iterationsCount = 1000;    // number of Ransac iterations.
    float reprojectionError = 1.5; // maximum allowed distance to consider it an inlier.
    float confidence = 0.99;       // RANSAC successful confidence.

    cv::Mat R;
    cv::Mat t;
    cv::Mat inliers;

    // cv::solvePnPRansac(pointset3d, pointset2d, camera_mat, distort_para, R, t,
    //                    false, iterationsCount, reprojectionError, confidence, inliers, SOLVEPNP_P3P);

    
}

bool MotionEstimator::doTriangulation(frame_t &cur_frame_1, frame_t &cur_frame_2,
                                      const std::vector<cv::DMatch> &matches,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sparse_pointcloud, bool show)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
    
    cv::Mat T1_mat;
    cv::Mat T2_mat;
    cv::Mat camera_mat;

    Eigen::Matrix<float, 3, 4> T1 = cur_frame_1.pose_cam.block(0, 0, 3, 4);
    Eigen::Matrix<float, 3, 4> T2 = cur_frame_2.pose_cam.block(0, 0, 3, 4);

    cv::eigen2cv(cur_frame_1.K_cam, camera_mat);
    cv::eigen2cv(T1, T1_mat);
    cv::eigen2cv(T2, T2_mat);

    // std::cout<<camera_mat<<std::endl;
    // std::cout<<T1_mat<<std::endl;
    // std::cout<<T2_mat<<std::endl;

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
        pt_temp.r = 255;
        pt_temp.g = 0;
        pt_temp.b = 0;
        sparse_pointcloud->points.push_back(pt_temp);
    }

    std::cout << "Point cloud generated done: [ " << sparse_pointcloud->points.size() << " ] points." << std::endl;
    
    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Triangularization done in " << time_used.count() << " seconds. " << std::endl;
    

    if (show)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Sfm Viewer"));
        viewer->setBackgroundColor(0, 0, 0);

        // for (int i = 0; i < sparse_pointcloud->points.size(); i++)
        // {
        //     char sparse_point[256];
        //     pcl::PointXYZ ptc_temp;
        //     ptc_temp.x = sparse_pointcloud->points[i].x;
        //     ptc_temp.y = sparse_pointcloud->points[i].y;
        //     ptc_temp.z = sparse_pointcloud->points[i].z;
        //     sprintf(sparse_point, "SP_%03u", i);
        //     viewer->addSphere(ptc_temp, 0.2, 1.0, 0.0, 0.0, sparse_point);
        // }

        viewer->addPointCloud(sparse_pointcloud, "sparsepointcloud");

        std::cout << "Click X(close) to continue..." << std::endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    std::cout << "Generate new sparse point cloud done." << std::endl;
    return true;
}
