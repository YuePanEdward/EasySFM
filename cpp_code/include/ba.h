#ifndef _INCLUDE_BUNDLE_ADJUSTMENT_H_
#define _INCLUDE_BUNDLE_ADJUSTMENT_H_

#include <cmath>

#include <unordered_map>
#include <Eigen/Core>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_plotter.h>

//Opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "utility.h"
#include "data_io.h"

namespace p3dv
{

  // Read a Bundle Adjustment in the Large dataset.
  class BundleAdjustment
  {
  public:
    ~BundleAdjustment()
    {
      std::vector<int>().swap(point_index_);
      std::vector<int>().swap(camera_index_);
      delete[] observations_;
      delete[] parameters_;
    }

    int num_observations() const { return num_observations_; }
    const double *observations() const { return observations_; }

    double *mutable_cameras() { return parameters_; }

    double *mutable_points() { return parameters_ + 6 * num_cameras_; }

    double *mutable_calib() { return parameters_ + 6 * num_cameras_ + 3 * num_points_; }

    double *mutable_camera_for_observation(int i) // Use the observation id to get right parameters
    {
      return mutable_cameras() + camera_index_[i] * 6;
    }

    double *mutable_point_for_observation(int i) // Use the observation id to get right parameters
    {
      return mutable_points() + point_index_[i] * 3;
    }

    bool initBA()
    {
      ceres_problem_ = new ceres::Problem();
      ceres_options_ = new ceres::Solver::Options();
      ceres_summary_ = new ceres::Solver::Summary();

      std::vector<int>().swap(point_index_);
      std::vector<int>().swap(camera_index_);
      std::vector<cv::Point2f>().swap(points_2d_);
      std::vector<Eigen::Matrix3f>().swap(calibs_);
      num_cameras_ = 0;
      num_points_ = 0;
      num_parameters_ = 0;
      num_observations_ = 0;
      ref_process_camera_id_ = -1;
      std::cout << "Bundle Ajustment parameters initialization done." << std::endl;
    }

    bool setBAProblem(std::vector<frame_t> &frames, std::vector<bool> &frame_id, pointcloud_sparse_t &sfm_sparse_points, double fix_calib_tolerance_BA, int reference_frame_id);

    bool setBASolver();

    bool solveBA(double fix_calib_tolerance_BA);

    // main function
    bool doSFMBA(std::vector<frame_t> &frames, std::vector<bool> &frame_id, pointcloud_sparse_t &sfm_sparse_points, double fix_calib_tolerance_BA = 0.0, int reference_frame_id = -1);

  private:
    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;

    ceres::Problem *ceres_problem_;
    ceres::Solver::Options *ceres_options_;
    ceres::Solver::Summary *ceres_summary_;

    std::vector<int> point_index_;
    std::vector<int> camera_index_;

    double *observations_;
    double *parameters_; //The structure of unknown parameters: [Camera parameters (*6, first rotate and then translation), 3D Points (*3)]

    std::vector<cv::Point2f> points_2d_;
    std::vector<Eigen::Matrix3f> calibs_;

    int ref_process_camera_id_;
  };

  class ReprojectErrorTerm_fixcalib
  {
  public:
    ReprojectErrorTerm_fixcalib(cv::Point2f p_2d, Eigen::Matrix3f calib) : p_2d_(p_2d), calib_(calib) {}

    template <typename T>
    bool operator()(const T *const cam_pose, const T *const point_3d, T *residual) const
    {
      T point3d_tr[3];

      T cere_tran[3];
      T cere_rot[3];

      cere_rot[0] = cam_pose[0];
      cere_rot[1] = cam_pose[1];
      cere_rot[2] = cam_pose[2];
      cere_tran[0] = cam_pose[3];
      cere_tran[1] = cam_pose[4];
      cere_tran[2] = cam_pose[5];

      //std::cout << "point 3d: " << point_3d[0] << " " << point_3d[1] << "  " << point_3d[2] << std::endl;

      //Apply transformation to point_3d to point3d_tr
      ceres::AngleAxisRotatePoint(cere_rot, point_3d, point3d_tr); //rotation

      point3d_tr[0] = point3d_tr[0] + cere_tran[0]; //translation
      point3d_tr[1] = point3d_tr[1] + cere_tran[1];
      point3d_tr[2] = point3d_tr[2] + cere_tran[2];

      //de-homo
      const T x = point3d_tr[0] / point3d_tr[2];
      const T y = point3d_tr[1] / point3d_tr[2];

      //Transform to photo coordinate using calib Mat
      const T u_reproj = x * T(calib_(0, 0)) + T(calib_(0, 2));
      const T v_reproj = y * T(calib_(1, 1)) + T(calib_(1, 2));

      T u_original = T(p_2d_.x);
      T v_original = T(p_2d_.y);

      // Reprojection error
      residual[0] = u_original - u_reproj;
      residual[1] = v_original - v_reproj;

      return true;
    }

    static ceres::CostFunction *Create(cv::Point2f p_2d, Eigen::Matrix3f calib)
    {
      return (new ceres::AutoDiffCostFunction<ReprojectErrorTerm_fixcalib, 2, 6, 3>(
          new ReprojectErrorTerm_fixcalib(p_2d, calib)));
    }

  private:
    cv::Point2f p_2d_;
    Eigen::Matrix3f calib_;
  };

  class ReprojectErrorTerm_updatecalib
  {
  public:
    ReprojectErrorTerm_updatecalib(cv::Point2f p_2d) : p_2d_(p_2d) {}

    template <typename T>
    bool operator()(const T *const cam_pose, const T *const point_3d, const T *const cam_calib, T *residual) const
    {
      T point3d_tr[3];

      T cere_tran[3];
      T cere_rot[3];

      cere_rot[0] = cam_pose[0];
      cere_rot[1] = cam_pose[1];
      cere_rot[2] = cam_pose[2];
      cere_tran[0] = cam_pose[3];
      cere_tran[1] = cam_pose[4];
      cere_tran[2] = cam_pose[5];

      //std::cout << "point 3d: " << point_3d[0] << " " << point_3d[1] << "  " << point_3d[2] << std::endl;

      //Apply transformation to point_3d to point3d_tr
      ceres::AngleAxisRotatePoint(cere_rot, point_3d, point3d_tr); //rotation

      point3d_tr[0] = point3d_tr[0] + cere_tran[0]; //translation
      point3d_tr[1] = point3d_tr[1] + cere_tran[1];
      point3d_tr[2] = point3d_tr[2] + cere_tran[2];

      //de-homo
      const T x = point3d_tr[0] / point3d_tr[2];
      const T y = point3d_tr[1] / point3d_tr[2];

      const T fx = cam_calib[0];
      const T cx = cam_calib[1];
      const T fy = cam_calib[2];
      const T cy = cam_calib[3];

      //Transform to photo coordinate using calib Mat
      const T u_reproj = x * fx + cx;
      const T v_reproj = y * fy + cy;

      T u_original = T(p_2d_.x);
      T v_original = T(p_2d_.y);

      // Reprojection error
      residual[0] = u_original - u_reproj;
      residual[1] = v_original - v_reproj;

      return true;
    }

    static ceres::CostFunction *Create(cv::Point2f p_2d)
    {
      return (new ceres::AutoDiffCostFunction<ReprojectErrorTerm_updatecalib, 2, 6, 3, 4>(
          new ReprojectErrorTerm_updatecalib(p_2d)));
    }

  private:
    cv::Point2f p_2d_;
  };

} // namespace p3dv

#endif // _INCLUDE_BUNDLE_ADJUSTMENT_H_
