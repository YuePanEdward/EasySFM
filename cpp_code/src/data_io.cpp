//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>
#include <fstream>

#include "data_io.h"
#include "estimate_motion.h"

using namespace p3dv;

bool DataIO::importImages(frame_t &cur_frame, bool show)
{
	std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
	cur_frame.rgb_image = cv::imread(cur_frame.image_file_path, CV_LOAD_IMAGE_COLOR); // Read the file

	if (!cur_frame.rgb_image.data)
	{ // Check for invalid input
		std::cout << "No more images" << std::endl;
		return false;
	}

	std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
	std::cout << "Import Image [ " << cur_frame.image_file_path << " ] done in " << time_used.count() << " seconds. " << std::endl;

	if (show)
	{
		cv::namedWindow("Show window", cv::WINDOW_AUTOSIZE); // Create a window for display.
		cv::imshow("Show window", cur_frame.rgb_image);		 // Show our image inside it.
		std::cout << "Show frame " << cur_frame.frame_id << std::endl;
		cv::waitKey(0); // Wait for a keystroke in the window
	}

	return true;
}

bool DataIO::importCalib(const std::string &fileName, Eigen::Matrix3f &K_mat)
{
	std::ifstream in(fileName, std::ios::in);
	if (!in)
	{
		return false;
	}
	double a_ = 0, b_ = 0, c_ = 0;
	int i = 0;
	while (!in.eof())
	{
		if (i < 3)
			in >> K_mat(i, 0) >> K_mat(i, 1) >> K_mat(i, 2);
		if (in.fail())
		{
			break;
		}
		++i;
	}
	in.close();
	std::cout << "Import camera calibration file done." << std::endl;
	return true;
}

bool DataIO::writePcdFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud)
{
	std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

	pointCloud->width = 1;
	pointCloud->height = pointCloud->points.size();

	if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1)
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}

	std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
	std::cout << "Output [ " << pointCloud->points.size() << " ] points cost = " << time_used.count() << " seconds. " << std::endl;
	std::cout << "Output pcd file done." << std::endl;
	return true;
}

bool DataIO::writePlyFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud)
{
	std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
	
	pointCloud->width = 1;
	pointCloud->height = pointCloud->points.size();

	if (pcl::io::savePLYFile(fileName, *pointCloud) == -1)
	{
		PCL_ERROR("Couldn't write file \n");
		return false;
	}

	std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
	std::cout << "Output [ " << pointCloud->points.size() << " ] points cost = " << time_used.count() << " seconds. " << std::endl;
	std::cout << "Output ply file done." << std::endl;
	return true;
}

bool DataIO::writeTxtFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud)
{
	std::ofstream ofs;
	ofs.open(fileName);
	if (ofs.is_open())
	{
		for (int i = 0; i < pointCloud->size(); ++i)
		{
			ofs << setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].x << "  "
				<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].y << "  "
				<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].z << "  "
				<< setiosflags(ios::fixed) << setprecision(3) << pointCloud->points[i].r << "  "
				<< setiosflags(ios::fixed) << setprecision(3) << pointCloud->points[i].g << "  "
				<< setiosflags(ios::fixed) << setprecision(3) << pointCloud->points[i].b << "  "
				//<<"  "<< setiosflags(ios::fixed) << setprecision(5) << pointCloud->points[i].intensity
				<< std::endl;
		}
		ofs.close();
	}
	else
	{
		return 0;
	}
	std::cout << "Output done." << std::endl;
	return 1;
}

bool DataIO::displaySFM(std::vector<frame_t> &frames, std::vector<bool> &frames_to_process,
						pointcloud_sparse_t &sparse_pointcloud, std::string viewer_name, bool black_background)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(viewer_name));
	if (black_background)
		viewer->setBackgroundColor(0, 0, 0);
	else
		viewer->setBackgroundColor(255, 255, 255);

	char t[256];
	std::string s;
	int n = 0;
	float frame_color_r, frame_color_g, frame_color_b;
	float sphere_size = 0.02;
	float point_size = 0.04;
	float line_size_cam_z = 0.4;
	float line_size_cam_x = 0.6;
	float line_size_cam_y = 0.4;

	// Draw camera
	for (int i = 0; i < frames.size(); i++)
	{
		if (!frames_to_process[i])
		{

			std::cout << "Add frame [ " << i << " ] , pose:\n"
					  << frames[i].pose_cam << std::endl;

			//std::cout << "Begin" << std::endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr camera_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_camera_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

			pcl::PointXYZ pt_cam_center(0, 0, 0);
			camera_pointcloud->points.push_back(pt_cam_center);
			pcl::PointXYZ pt_cam_corner_1(-line_size_cam_x / 2, -line_size_cam_y / 2, line_size_cam_z);
			camera_pointcloud->points.push_back(pt_cam_corner_1);
			pcl::PointXYZ pt_cam_corner_2(-line_size_cam_x / 2, line_size_cam_y / 2, line_size_cam_z);
			camera_pointcloud->points.push_back(pt_cam_corner_2);
			pcl::PointXYZ pt_cam_corner_3(line_size_cam_x / 2, line_size_cam_y / 2, line_size_cam_z);
			camera_pointcloud->points.push_back(pt_cam_corner_3);
			pcl::PointXYZ pt_cam_corner_4(line_size_cam_x / 2, -line_size_cam_y / 2, line_size_cam_z);
			camera_pointcloud->points.push_back(pt_cam_corner_4);

			Eigen::Matrix4f test_mat = frames[i].pose_cam;
			MotionEstimator ee;
			ee.transformCloud(camera_pointcloud, transformed_camera_pointcloud, test_mat);

			sprintf(t, "point1_%d", n);
			s = t;
			viewer->addSphere(transformed_camera_pointcloud->points[0], sphere_size, 1.0, 1.0, 1.0, s);
			sprintf(t, "point2_%d", n);
			s = t;
			viewer->addSphere(transformed_camera_pointcloud->points[1], sphere_size, 1.0, 1.0, 1.0, s);
			sprintf(t, "point3_%d", n);
			s = t;
			viewer->addSphere(transformed_camera_pointcloud->points[2], sphere_size, 1.0, 1.0, 1.0, s);
			sprintf(t, "point4_%d", n);
			s = t;
			viewer->addSphere(transformed_camera_pointcloud->points[3], sphere_size, 1.0, 1.0, 1.0, s);
			sprintf(t, "point5_%d", n);
			s = t;
			viewer->addSphere(transformed_camera_pointcloud->points[4], sphere_size, 1.0, 1.0, 1.0, s);

			sprintf(t, "line1_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[1], 1.0, 0.0, 0.0, s);
			sprintf(t, "line2_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[2], 1.0, 0.0, 0.0, s);
			sprintf(t, "line3_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[3], 1.0, 0.0, 0.0, s);
			sprintf(t, "line4_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[4], 1.0, 0.0, 0.0, s);
			sprintf(t, "line5_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[1], transformed_camera_pointcloud->points[2], 1.0, 0.0, 0.0, s);
			sprintf(t, "line6_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[2], transformed_camera_pointcloud->points[3], 1.0, 0.0, 0.0, s);
			sprintf(t, "line7_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[3], transformed_camera_pointcloud->points[4], 1.0, 0.0, 0.0, s);
			sprintf(t, "line8_%d", n);
			s = t;
			viewer->addLine(transformed_camera_pointcloud->points[4], transformed_camera_pointcloud->points[1], 1.0, 0.0, 0.0, s);

			n++;
		}
		//std::cout << "Add Camera " << i << " done." << std::endl;
	}

	// Draw point cloud
	for (int i = 0; i < sparse_pointcloud.rgb_pointcloud->points.size(); i++)
	{
		char sparse_point[256];
		pcl::PointXYZ ptc_temp;
		ptc_temp.x = sparse_pointcloud.rgb_pointcloud->points[i].x;
		ptc_temp.y = sparse_pointcloud.rgb_pointcloud->points[i].y;
		ptc_temp.z = sparse_pointcloud.rgb_pointcloud->points[i].z;

		sprintf(sparse_point, "SP_%03u", i);
		viewer->addSphere(ptc_temp, point_size, sparse_pointcloud.rgb_pointcloud->points[i].r / 255.0, sparse_pointcloud.rgb_pointcloud->points[i].g / 255.0, sparse_pointcloud.rgb_pointcloud->points[i].b / 255.0, sparse_point);
	}

	//viewer->addPointCloud(sparse_pointcloud.rgb_pointcloud, "sparsepointcloud");

	std::cout << "Click X(close) to continue..." << std::endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return true;
}