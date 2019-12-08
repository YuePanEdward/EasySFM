#ifndef _INCLUDE_UTILITY_H_
#define _INCLUDE_UTILITY_H_

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace p3dv
{

struct pose_3d_t
{
    Eigen::Vector3d trans;
    Eigen::Quaterniond qua;

    pose_3d_t()
    {
        trans << 0, 0, 0;
        qua = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
        qua.normalize();
    }
    pose_3d_t(const pose_3d_t &pose)
    {
        this->copyFrom(pose);
    }
    pose_3d_t(Eigen::Quaterniond qua,
              Eigen::Vector3d trans) : trans(trans), qua(qua)
    {
        qua.normalize();
    }
    pose_3d_t operator*(const pose_3d_t &pose) const
    {
        return pose_3d_t(this->qua.normalized() * pose.qua.normalized(), this->qua.normalized() * pose.trans + this->trans);
    }
    bool operator==(const pose_3d_t &pose) const
    {
        if (this->qua.x() == pose.qua.x() &&
            this->qua.y() == pose.qua.y() &&
            this->qua.z() == pose.qua.z() &&
            this->qua.w() == pose.qua.w() &&
            this->trans == pose.trans)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    Eigen::Matrix4d GetMatrix() const
    {
        //CHECK(qua.norm() == 1) << "NO EQUAL";
        Eigen::Matrix4d transformation_matrix;
        transformation_matrix.block<3, 3>(0, 0) = qua.normalized().toRotationMatrix(); //You need to gurantee the qua is normalized
        transformation_matrix.block<3, 1>(0, 3) = trans;
        transformation_matrix.block<1, 4>(3, 0) << 0, 0, 0, 1;
        return transformation_matrix;
    }
    void SetPose(Eigen::Matrix4d transformation)
    {
        qua = Eigen::Quaterniond(transformation.block<3, 3>(0, 0)).normalized();
        trans << transformation(0, 3), transformation(1, 3), transformation(2, 3);
    }
    void copyFrom(const pose_3d_t &pose)
    {
        trans = pose.trans;
        //  trans << pose.trans[0], pose.trans[1], pose.trans[2];
        qua = Eigen::Quaterniond(pose.qua);
    }
    // inverse and return
    pose_3d_t inverse()
    {
        Eigen::Matrix4d transformation_matrix = GetMatrix();
        SetPose(transformation_matrix.inverse());
        return *this;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct frame_t
{
    unsigned int frame_id;
    std::string image_file_path;

    cv::Mat rgb_image;
    cv::Mat gray_image;
    cv::Mat semantic_seg_image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> unique_pixel_ids;  // assign the initial value as -1

    Eigen::Matrix4f pose_cam;    // extrinsic elements of the camera
    Eigen::Matrix3f K_cam;       // intrinsic elements of the camera
    Eigen::Matrix4f P_cam;       // projection matrix of the camera

    frame_t(unsigned int id, std::string &image_path)
    {
        frame_id = id;
        image_file_path = image_path;
        pose_cam = Eigen::Matrix4f::Identity();
    }

    bool init_pixel_ids()
    {
        for (int i=0;i<keypoints.size();i++ )
        {
            unique_pixel_ids.push_back(-1);
        }
    }
};

struct frame_pair_t
{
    unsigned int frame_id_1;
    unsigned int frame_id_2;
    
    std::vector<cv::DMatch> matches;
    
    std::vector<cv::DMatch> best_matches;

    Eigen::Matrix4f T_21;  

    frame_pair_t(unsigned int i, unsigned int j, std::vector<cv::DMatch>& input_matches, Eigen::Matrix4f & T_mat_21)
    {
        frame_id_1=i;
        frame_id_2=j;
        matches.assign(input_matches.begin(),input_matches.end()); 
        T_21=T_mat_21;
    }
};


struct frame_graph_t
{
     std::vector<std::vector<frame_pair_t>> frame_graph;
     
    //  frame_graph_t(unsigned int frame_num)
    //  {   
    //      frame_graph.resize(frame_num);
    //      for (int i=0;i++;i<frame_num)
    //      {
    //          frame_graph[i].resize(frame_num);
    //      }
    //  }

};

struct pointcloud_sparse_t
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pointcloud;
    
    std::vector<int> unique_point_ids;

    pointcloud_sparse_t()
    {
        rgb_pointcloud= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        //std::cout<< "init done"<<std::endl;
    }

};

enum feature_type
{
    SIFT,SURF,ORB
};

} // namespace p3dv

#endif