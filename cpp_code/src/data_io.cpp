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

bool DataIO::importImageFilenames(const std::string image_list_path, const std::string image_data_path,
								  std::vector<frame_t> &frames)
{
	//read image filename
	std::ifstream image_list_file(image_list_path.c_str(), std::ios::in);
	if (!image_list_file.is_open())
	{
		std::cout << "open image_list_file failed, file is: " << image_list_path << std::endl;
		return 0;
	}

	int count = 0;
	while (image_list_file.peek() != EOF)
	{
		std::string cur_file;
		image_list_file >> cur_file;
		if (!cur_file.empty())
		{
			cur_file = image_data_path + "/" + cur_file;
			frame_t cur_frame(count, cur_file);
			frames.push_back(cur_frame);
			std::cout << count << ": " << cur_file << std::endl;
			count++;
		}
	}
	int frame_number = frames.size();
	std::cout << "Frame number is " << frame_number << std::endl;

	return 1;
}

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

bool DataIO::importDistort(const std::string &fileName, cv::Mat &distort_coeff)
{
	std::ifstream in(fileName, std::ios::in);
	if (!in)
	{
		return false;
	}
	int i = 0;
	float k1, k2, p1, p2;
	while (!in.eof())
	{
		if (i < 3)
			in >> k1 >> k2 >> p1 >> p2;
		if (in.fail())
		{
			break;
		}
		++i;
	}
	in.close();

	distort_coeff.at<float>(0, 0) = k1;
	distort_coeff.at<float>(0, 1) = k2;
	distort_coeff.at<float>(0, 2) = p1;
	distort_coeff.at<float>(0, 3) = p2;

	std::cout << "Import camera distortion coefficients file done." << std::endl;
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
