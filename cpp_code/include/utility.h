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

    pose_3d_t pose_cam;                                          // extrinsic elements of the camera
    Eigen::Matrix3d K_cam;                                       // intrinsic elements of the camera
    Eigen::Matrix4d P_cam;                                       // projection matrix of the camera
    
};

struct pointcloud_sparse_t
{  
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pointcloud;

};




} // namespace p3dv