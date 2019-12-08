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
                                              Eigen::Matrix4f &T, double ransac_prob, double ransac_thre, bool show)
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
    cv::eigen2cv(cur_frame_1.K_cam, camera_mat);

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

    return 1;
}

bool MotionEstimator::estimate2D3D_P3P_RANSAC(frame_t &cur_frame, pointcloud_sparse_t &cur_map_3d,
                                              int iterationsCount, double ransac_prob, double ransac_thre, bool show)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    cv::Mat camera_mat;
    cv::eigen2cv(cur_frame.K_cam, camera_mat);

    //std::cout<< "K Mat:" <<std::endl<< camera_mat<<std::endl;

    std::vector<cv::Point2f> pointset2d;
    std::vector<cv::Point3f> pointset3d;

    int count = 0;
    float dist_thre = 200;

    std::cout << "2D points: " << cur_frame.unique_pixel_ids.size() << std::endl
              << "3D points: " << cur_map_3d.unique_point_ids.size() << std::endl;

    // Construct the 2d-3d initial matchings
    for (int i = 0; i < cur_frame.unique_pixel_ids.size(); i++)
    {
        for (int j = 0; j < cur_map_3d.unique_point_ids.size(); j++)
        {
            if (cur_frame.unique_pixel_ids[i] == cur_map_3d.unique_point_ids[j])
            {

                float x_3d = cur_map_3d.rgb_pointcloud->points[j].x;
                float y_3d = cur_map_3d.rgb_pointcloud->points[j].y;
                float z_3d = cur_map_3d.rgb_pointcloud->points[j].z;
                float x_2d = cur_frame.keypoints[i].pt.x;
                float y_2d = cur_frame.keypoints[i].pt.y;

                if (x_3d < dist_thre && y_3d < dist_thre && z_3d < dist_thre)
                {
                    // Assign value for pointset2d and pointset3d
                    pointset2d.push_back(cv::Point2f(x_2d, y_2d));
                    //pointset2d.push_back(pixel2cam(cur_frame.keypoints[i].pt, camera_mat));

                    pointset3d.push_back(cv::Point3f(x_3d, y_3d, z_3d));
                    count++;
                }
                //std::cout << "2D: " << cur_frame.unique_pixel_ids[i] << " " << cur_frame.keypoints[i].pt << std::endl;
                //std::cout << "3D: " << cur_map_3d.unique_point_ids[j] << " " << cv::Point3f(x_3d, y_3d, z_3d) << std::endl;
            }
        }
    }

    std::cout << count << " initial correspondences are used." << std::endl;

    // Use RANSAC P3P to estiamte the optimal transformation

    cv::Mat distort_para = cv::Mat::zeros(1, 4, CV_64FC1); // Assuming no lens distortion
    cv::Mat r_vec;
    cv::Mat t_vec;
    cv::Mat inliers;

    cv::solvePnPRansac(pointset3d, pointset2d, camera_mat, distort_para, r_vec, t_vec,
                       false, iterationsCount, ransac_thre, ransac_prob, inliers, cv::SOLVEPNP_EPNP);

    //cv::solvePnP(pointset3d, pointset2d, camera_mat, distort_para, r_vec, t_vec, false, cv::SOLVEPNP_EPNP);

    std::cout << "Inlier count: " << inliers.rows << std::endl;

    cv::Mat R_mat;
    cv::Rodrigues(r_vec, R_mat);

    Eigen::Matrix3f R_eigen;
    Eigen::Vector3f t_eigen;
    Eigen::Matrix4f T_mat;

    cv::cv2eigen(R_mat, R_eigen);
    cv::cv2eigen(t_vec, t_eigen);

    T_mat.block(0, 0, 3, 3) = R_eigen;
    T_mat.block(0, 3, 3, 1) = t_eigen;
    T_mat(3, 0) = 0;
    T_mat(3, 1) = 0;
    T_mat(3, 2) = 0;
    T_mat(3, 3) = 1;

    // std::cout << "Transform is: " << std::endl
    //           << T_mat << std::endl;

    cur_frame.pose_cam = T_mat;

    // Calculate the reprojection error
    std::vector<cv::Point2f> proj_points;
    cv::projectPoints(pointset3d, R_mat, t_vec, camera_mat, distort_para, proj_points);

    float reproj_err = 0.0;
    for (int i = 0; i < proj_points.size(); i++)
    {
        float cur_repro_error = norm(proj_points[i] - pointset2d[i]);
        reproj_err += cur_repro_error;

        //std::cout << cur_repro_error << std::endl;
    }

    reproj_err /= proj_points.size();

    std::cout << "Mean reprojection error: " << reproj_err << std::endl;

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Estimate Motion [3D-2D] cost = " << time_used.count() << " seconds. " << std::endl;

    return 1;
}

bool MotionEstimator::doTriangulation(frame_t &cur_frame_1, frame_t &cur_frame_2,
                                      const std::vector<cv::DMatch> &matches,
                                      pointcloud_sparse_t &sparse_pointcloud, bool show)
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

    int count_newly_triangu = 0;
    for (int i = 0; i < matches.size(); i++)
    {
        bool already_in_world = 0;
        for (int k = 0; k < sparse_pointcloud.unique_point_ids.size(); k++)
        {
            if (sparse_pointcloud.unique_point_ids[k] == cur_frame_1.unique_pixel_ids[matches[i].queryIdx])
            {
                already_in_world = 1;
                break;
            }
        }
        if (!already_in_world)
        {
            sparse_pointcloud.unique_point_ids.push_back(cur_frame_1.unique_pixel_ids[matches[i].queryIdx]);

            pointset1.push_back(pixel2cam(cur_frame_1.keypoints[matches[i].queryIdx].pt, camera_mat));
            pointset2.push_back(pixel2cam(cur_frame_2.keypoints[matches[i].trainIdx].pt, camera_mat));

            count_newly_triangu++;
        }
    }

    cv::Mat pts_3d_homo;
    if (pointset1.size() > 0)
        cv::triangulatePoints(T1_mat, T2_mat, pointset1, pointset2, pts_3d_homo);

    // De-homo and assign color
    for (int i = 0; i < pts_3d_homo.cols; i++)
    {
        cv::Mat pts_3d = pts_3d_homo.col(i);

        pts_3d /= pts_3d.at<float>(3, 0);

        pcl::PointXYZRGB pt_temp;
        pt_temp.x = pts_3d.at<float>(0, 0);
        pt_temp.y = pts_3d.at<float>(1, 0);
        pt_temp.z = pts_3d.at<float>(2, 0);

        cv::Point2f cur_key_pixel = cur_frame_1.keypoints[matches[i].queryIdx].pt;

        uchar blue = cur_frame_1.rgb_image.at<cv::Vec3b>(cur_key_pixel.y, cur_key_pixel.x)[0];
        uchar green = cur_frame_1.rgb_image.at<cv::Vec3b>(cur_key_pixel.y, cur_key_pixel.x)[1];
        uchar red = cur_frame_1.rgb_image.at<cv::Vec3b>(cur_key_pixel.y, cur_key_pixel.x)[2];

        pt_temp.r = red;
        pt_temp.g = green;
        pt_temp.b = blue;
        sparse_pointcloud.rgb_pointcloud->points.push_back(pt_temp);
    }

    std::cout << "Triangulate [ " << count_newly_triangu << " ] new points, [ " << sparse_pointcloud.rgb_pointcloud->points.size() << " ] points in total." << std::endl;

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Triangularization done in " << time_used.count() << " seconds. " << std::endl;

    if (show)
    {
        // Show 2D image pair and correspondences
        cv::Mat match_image_pair;
        cv::drawMatches(cur_frame_1.rgb_image, cur_frame_1.keypoints, cur_frame_2.rgb_image, cur_frame_2.keypoints, matches, match_image_pair);
        cv::imshow("Triangularization matches", match_image_pair);
        cv::waitKey(0);

        // for (int i = 0; i < sparse_pointcloud.rgb_pointcloud->points.size(); i++)
        // {
        //     std::cout << sparse_pointcloud.unique_point_ids[i] << " : "
        //               << sparse_pointcloud.rgb_pointcloud->points[i].x << " , "
        //               << sparse_pointcloud.rgb_pointcloud->points[i].y << " , "
        //               << sparse_pointcloud.rgb_pointcloud->points[i].z << std::endl;
        // }

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Sfm Viewer"));
        viewer->setBackgroundColor(0, 0, 0);

        // Draw camera
        char t[256];
        std::string s;
        int n = 0;
        float frame_color_r, frame_color_g, frame_color_b;
        float sphere_size = 0.05;
        float line_size_cam = 0.4;

        pcl::PointXYZ pt_cam1(cur_frame_1.pose_cam(0, 3), cur_frame_1.pose_cam(1, 3), cur_frame_1.pose_cam(2, 3));
        pcl::PointXYZ pt_cam2(cur_frame_2.pose_cam(0, 3), cur_frame_2.pose_cam(1, 3), cur_frame_2.pose_cam(2, 3));
        pcl::PointXYZ point_pt_cam1_x_axis(cur_frame_1.pose_cam(0, 3) + line_size_cam, cur_frame_1.pose_cam(1, 3), cur_frame_1.pose_cam(2, 3));
        pcl::PointXYZ point_pt_cam1_y_axis(cur_frame_1.pose_cam(0, 3), cur_frame_1.pose_cam(1, 3) + line_size_cam, cur_frame_1.pose_cam(2, 3));
        pcl::PointXYZ point_pt_cam1_z_axis(cur_frame_1.pose_cam(0, 3), cur_frame_1.pose_cam(1, 3), cur_frame_1.pose_cam(2, 3) + line_size_cam);
        pcl::PointXYZ point_pt_cam2_x_axis(cur_frame_2.pose_cam(0, 3) + line_size_cam, cur_frame_2.pose_cam(1, 3), cur_frame_2.pose_cam(2, 3));
        pcl::PointXYZ point_pt_cam2_y_axis(cur_frame_2.pose_cam(0, 3), cur_frame_2.pose_cam(1, 3) + line_size_cam, cur_frame_2.pose_cam(2, 3));
        pcl::PointXYZ point_pt_cam2_z_axis(cur_frame_2.pose_cam(0, 3), cur_frame_2.pose_cam(1, 3), cur_frame_2.pose_cam(2, 3) + line_size_cam);

        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(pt_cam1, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(pt_cam2, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(pt_cam1, point_pt_cam1_x_axis, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(pt_cam1, point_pt_cam1_y_axis, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(pt_cam1, point_pt_cam1_z_axis, 0.0, 0.0, 1.0, s);
        n++;

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(pt_cam2, point_pt_cam2_x_axis, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(pt_cam2, point_pt_cam2_y_axis, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(pt_cam2, point_pt_cam2_z_axis, 0.0, 0.0, 1.0, s);
        n++;

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

        viewer->addPointCloud(sparse_pointcloud.rgb_pointcloud, "sparsepointcloud");

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

/**
* \brief Transform a Point Cloud using a given transformation matrix
* \param[in]  cloud_in : A pointer of the Point Cloud before transformation
* \param[out] cloud_out : A pointer of the Point Cloud after transformation
* \param[in]  trans : A 4*4 transformation matrix
*/
bool MotionEstimator::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out,
	Eigen::Matrix4f & trans)
{
	Eigen::Matrix4Xf PC;
	Eigen::Matrix4Xf TPC; 
	PC.resize(4, cloud_in->size());
	TPC.resize(4, cloud_in->size());
	for (int i = 0; i < cloud_in->size(); i++)
	{
		PC(0,i)= cloud_in->points[i].x;
		PC(1,i)= cloud_in->points[i].y;
		PC(2,i)= cloud_in->points[i].z;
		PC(3,i)= 1;
	}
	TPC = trans * PC;
	for (int i = 0; i < cloud_in->size(); i++)
	{
		pcl::PointXYZ pt;
		pt.x = TPC(0, i);
		pt.y = TPC(1, i);
		pt.z = TPC(2, i);
		cloud_out->points.push_back(pt);
	}
	//cout << "Transform done ..." << endl;
}