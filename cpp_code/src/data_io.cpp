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

