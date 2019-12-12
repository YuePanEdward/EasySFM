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

using namespace std;

template <typename PointT>
class CProceesing
{
  public:
	bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr &incloud, typename pcl::PointCloud<PointT>::Ptr &outcloud, int MeanK=50, double std=2.0)
	{
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<PointT> sor;

		sor.setInputCloud(incloud);
		sor.setMeanK(MeanK);		 //50
		sor.setStddevMulThresh(std); //1.0
		sor.filter(*outcloud);
    
		return 1;
	}

  protected:
  private:
};

#endif //CLOUD_PRO_H