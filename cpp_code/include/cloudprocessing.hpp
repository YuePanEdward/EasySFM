//
// This file is for the general implements of several famous point cloud processing algorithms
// Dependent 3rd Libs: PCL (>1.7)
// Author: Yue Pan et al.
//

#ifndef CLOUD_PRO_H
#define CLOUD_PRO_H

//PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

using namespace std;

template <typename PointT>
class CProceesing
{
public:
	bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr &incloud, typename pcl::PointCloud<PointT>::Ptr &outcloud, int MeanK = 50, double std = 2.0)
	{
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<PointT> sor;

		sor.setInputCloud(incloud);
		sor.setMeanK(MeanK);				 //50
		sor.setStddevMulThresh(std); //1.0
		sor.filter(*outcloud);

		return 1;
	}

	bool GroundFilter_PMF(const typename pcl::PointCloud<PointT>::Ptr &cloud, typename pcl::PointCloud<PointT>::Ptr &gcloud, typename pcl::PointCloud<PointT>::Ptr &ngcloud, int max_window_size = 20, float slope = 1.0, float initial_distance = 0.5, float max_distance = 3.0)
	{
		pcl::PointIndicesPtr ground_points(new pcl::PointIndices);
		pcl::ProgressiveMorphologicalFilter<PointT> pmf;
		pmf.setInputCloud(cloud);
		pmf.setMaxWindowSize(max_window_size);		//20
		pmf.setSlope(slope);											//1.0f
		pmf.setInitialDistance(initial_distance); //0.5f
		pmf.setMaxDistance(max_distance);					//3.0f
		pmf.extract(ground_points->indices);

		// Create the filtering object
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ground_points);
		extract.filter(*gcloud);

		//std::cout << "Ground cloud after filtering (PMF): " << std::endl;
		//std::cout << *gcloud << std::endl;

		// Extract non-ground returns
		extract.setNegative(true);
		extract.filter(*ngcloud);

		//std::out << "Non-ground cloud after filtering (PMF): " << std::endl;
		//std::out << *ngcloud << std::endl;

		return 1;
	}

protected:
private:
};

#endif //CLOUD_PRO_H