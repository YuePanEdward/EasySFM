
#if 0

#ifndef _INCLUDE_BUNDLE_ADJUSTMENT_H_
#define _INCLUDE_BUNDLE_ADJUSTMENT_H_

#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unordered_map>
#include <Eigen/Core>

//pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_plotter.h>
//Opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "utility.h"
#include "data_io.h"

namespace p3dv
{

template <typename T>
inline Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
        const Eigen::Quaternion<T> &quaternion)
{
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    // We choose the quaternion with positive 'w', i.e., the one with a smaller
    // angle that represents this orientation.
    if (normalized_quaternion.w() < 0.)
    {
        normalized_quaternion.w() *= T(-1.);
        normalized_quaternion.x() *= T(-1.);
        normalized_quaternion.y() *= T(-1.);
        normalized_quaternion.z() *= T(-1.);
    }
    // We convert the normalized_quaternion into a vector along the rotation axis
    // with length of the rotation angle.
    const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                  normalized_quaternion.w());
    constexpr double kCutoffAngle = 1e-7; // We linearize below this angle.
    const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(),
                                  scale * normalized_quaternion.z());
}

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.

class PoseGraph3dErrorTermQUAT
{
public:
    PoseGraph3dErrorTermQUAT(const pose_qua_t &t_ab_measured,
                             const Eigen::Matrix<double, 6, 6> &sqrt_information)
            : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                    const T *const p_b_ptr, const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        //        Eigen::Quaternion<T> q_a_inverse = q_a.inverse();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
                t_ab_measured_.quat.template cast<T>() * q_ab_estimated.conjugate();
        //        Eigen::Quaternion<T> delta_q =
        //                t_ab_measured_.quat.template cast<T>() * q_ab_estimated.inverse();
        // Eigen::Matrix::cast<typename T> can change num_type in Matrix
        // To use Eigen::cast in template function, you need to write like a.template cast<T>(),
        // so that compiler will know '<" is not a "less than sign"

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
                p_ab_estimated - t_ab_measured_.trans.template cast<T>();

        // use euler-angle, Failed!
        //            Eigen::Matrix<T,3,3> rot = delta_q.toRotationMatrix().template cast<T>();
        //            Eigen::Matrix<T,3,1> vec = rot.eulerAngles(0, 1, 2).template cast<T>();
        //            residuals.template block<3, 1>(3, 0) = vec;
        // use quaternion real part
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
        // use Angle-Axis error
        //            residuals.template block<3, 1>(3, 0) = RotationQuaternionToAngleAxisVector(delta_q);

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &t_ab_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTermQUAT, 6, 3, 4, 3, 4>(
                new PoseGraph3dErrorTermQUAT(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const pose_qua_t t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

class PoseGraph3dErrorTermROT
{
public:
    PoseGraph3dErrorTermROT(const pose_qua_t &t_ab_measured,
                            const Eigen::Matrix<double, 6, 6> &sqrt_information)
            : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                    const T *const p_b_ptr, const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
                t_ab_measured_.quat.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = Eigen::Matrix<T, 3, 1>::Zero();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &t_ab_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTermROT, 6, 3, 4, 3, 4>(
                new PoseGraph3dErrorTermROT(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const pose_qua_t t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// Should be RUN after Orientation is already optimized
class PoseGraph3dErrorTermTRANS
{
public:
    PoseGraph3dErrorTermTRANS(const pose_qua_t &t_ab_measured,
                              const Eigen::Matrix<double, 6, 6> &sqrt_information)
            : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                    const T *const p_b_ptr, const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        //        Eigen::Quaternion<T> q_a_inverse = q_a.inverse();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
                p_ab_estimated - t_ab_measured_.trans.template cast<T>();
        residuals.template block<3, 1>(3, 0) = Eigen::Matrix<T, 3, 1>::Zero();
        // use Angle-Axis error
        //            residuals.template block<3, 1>(3, 0) = RotationQuaternionToAngleAxisVector(delta_q);

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &t_ab_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTermTRANS, 6, 3, 4, 3, 4>(
                new PoseGraph3dErrorTermTRANS(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const pose_qua_t t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// left multiply
// Computes the error term for submap pose that have a gnss measurement.
// Equation: t_submap = t_delta_calib * t_ldr2gnss * t_gnss_measured
// t_delta_calib ^ -1 * t_submap = t_ldr2gnss * t_gnss_measured -- rhs is const for a transaction
// The estimated measurement is:
//   t_sm = [ p_sme ]  = [ R(dc)^T * (p_sm - p_dc) ]
//          [ q_sme ]    [ q_dc^{-1] * q_sm        ]
//
//   error = [ p_sme - \hat{p}_sm                 ]
//           [ 2.0 * Vec(q_sme * \hat{q}_sm^{-1}) ]
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class ReprojectErrorTerm
{
public:
    ReprojectErrorTerm    (const pose_qua_t &gnss_pose_measured,
                           const pose_qua_t &calib_ldr2gnss_measured,
                           const Eigen::Matrix<double, 6, 6> &sqrt_information) : gnss_pose_measured_(gnss_pose_measured), calib_ldr2gnss_measured_(calib_ldr2gnss_measured),
                                                                                  sqrt_information_(sqrt_information), ldr_pose_measured_(calib_ldr2gnss_measured * gnss_pose_measured) {}

    // p_sm_ptr -- submap pose.trans
    // q_sm_ptr -- submap pose.quat
    // p_dc_ptr -- delta calib pose.trans
    // q_dc_ptr -- delta calib pose.quat
    template <typename T>
    bool operator()(const T *const p_sm_ptr, const T *const q_sm_ptr,
                    const T *const p_dc_ptr, const T *const q_dc_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_sm(p_sm_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_sm(q_sm_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_dc(p_dc_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_dc(q_dc_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_dc_inverse = q_dc.conjugate();
        Eigen::Quaternion<T> q_sme_estimated = q_dc_inverse * q_sm;

        Eigen::Matrix<T, 3, 1> p_sme_estimated = q_dc_inverse * (p_sm - p_dc);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
                ldr_pose_measured_.quat.template cast<T>() * q_sme_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
                p_sme_estimated - ldr_pose_measured_.trans.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
        //        residuals.template block<3, 1>(3, 0) = Eigen::Matrix<T, 3, 1>::Zero();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &gnss_pose_measured,
            const pose_qua_t &calib_ldr2gnss_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraphGnssErrorTerm, 6, 3, 4, 3, 4>(
                new PoseGraphGnssErrorTerm(gnss_pose_measured, calib_ldr2gnss_measured,
                                           sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Gnss Measurement
    const pose_qua_t gnss_pose_measured_; // not used
    // Calibration Pose T_ldr_gnss (point_lidar = T_ldr_gnss * point_gnss)
    const pose_qua_t calib_ldr2gnss_measured_; // not used
    const pose_qua_t ldr_pose_measured_;
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

// right multiply
// Computes the error term for submap pose that have a gnss measurement.
// Equation: t_submap = t_gnss_measured * t_gnss2ldr * t_delta_calib
// t_submap * t_delta_calib ^ -1 = t_gnss_measured * t_gnss2ldr -- rhs is const for a transaction
// The estimated measurement is:
//   t_sm = [ p_sme ]  = [ p_sm - R(sm) * R(dc)^T * p_dc ]
//          [ q_sme ]    [  q_sm * q_dc^{-1]             ]
//
//   error = [ p_sme - \hat{p}_sm                 ]
//           [ 2.0 * Vec(q_sme * \hat{q}_sm^{-1}) ]
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseGraphGnssRightErrorTerm
{
public:
    PoseGraphGnssRightErrorTerm(const pose_qua_t &gnss_pose_measured,
                                const pose_qua_t &calib_ldr2gnss_measured,
                                const Eigen::Matrix<double, 6, 6> &sqrt_information) : gnss_pose_measured_(gnss_pose_measured), calib_ldr2gnss_measured_(calib_ldr2gnss_measured),
                                                                                       sqrt_information_(sqrt_information), ldr_pose_measured_(calib_ldr2gnss_measured * gnss_pose_measured) {}

    // p_sm_ptr -- submap pose.trans
    // q_sm_ptr -- submap pose.quat
    // p_dc_ptr -- delta calib pose.trans
    // q_dc_ptr -- delta calib pose.quat
    template <typename T>
    bool operator()(const T *const p_sm_ptr, const T *const q_sm_ptr,
                    const T *const p_dc_ptr, const T *const q_dc_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_sm(p_sm_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_sm(q_sm_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_dc(p_dc_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_dc(q_dc_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_dc_inverse = q_dc.conjugate();
        Eigen::Quaternion<T> q_sme_estimated = q_sm * q_dc_inverse;

        Eigen::Matrix<T, 3, 1> p_sme_estimated = p_sm - q_sm * q_dc_inverse * p_dc;

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
                ldr_pose_measured_.quat.template cast<T>() * q_sme_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = p_sme_estimated - ldr_pose_measured_.trans.template cast<T>();
        //        residuals.template block<3, 1>(0, 0) = Eigen::Matrix<T,3,1>::Zero();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
        //        residuals.template block<3, 1>(3, 0) = Eigen::Matrix<T, 3, 1>::Zero();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &gnss_pose_measured,
            const pose_qua_t &calib_ldr2gnss_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraphGnssRightErrorTerm, 6, 3, 4, 3, 4>(
                new PoseGraphGnssRightErrorTerm(gnss_pose_measured, calib_ldr2gnss_measured,
                                                sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Gnss Measurement
    const pose_qua_t gnss_pose_measured_; // not used
    // Calibration Pose T_ldr_gnss (point_lidar = T_ldr_gnss * point_gnss)
    const pose_qua_t calib_ldr2gnss_measured_; // not used
    const pose_qua_t ldr_pose_measured_;
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

class PoseGraphGnssRightRotErrorTerm
{
public:
    PoseGraphGnssRightRotErrorTerm(const pose_qua_t &gnss_pose_measured,
                                   const pose_qua_t &calib_ldr2gnss_measured,
                                   const Eigen::Matrix<double, 6, 6> &sqrt_information) : gnss_pose_measured_(gnss_pose_measured), calib_ldr2gnss_measured_(calib_ldr2gnss_measured),
                                                                                          sqrt_information_(sqrt_information), ldr_pose_measured_(calib_ldr2gnss_measured * gnss_pose_measured) {}

    template <typename T>
    bool operator()(const T *const p_sm_ptr, const T *const q_sm_ptr,
                    const T *const p_dc_ptr, const T *const q_dc_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_sm(p_sm_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_sm(q_sm_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_dc(p_dc_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_dc(q_dc_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_dc_inverse = q_dc.conjugate();
        Eigen::Quaternion<T> q_sme_estimated = q_sm * q_dc_inverse;

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
                ldr_pose_measured_.quat.template cast<T>() * q_sme_estimated.conjugate();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = Eigen::Matrix<T, 3, 1>::Zero();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &gnss_pose_measured,
            const pose_qua_t &calib_ldr2gnss_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraphGnssRightRotErrorTerm, 6, 3, 4, 3, 4>(
                new PoseGraphGnssRightRotErrorTerm(gnss_pose_measured, calib_ldr2gnss_measured,
                                                   sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Gnss Measurement
    const pose_qua_t gnss_pose_measured_; // not used
    // Calibration Pose T_ldr_gnss (point_lidar = T_ldr_gnss * point_gnss)
    const pose_qua_t calib_ldr2gnss_measured_; // not used
    const pose_qua_t ldr_pose_measured_;
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

class PoseGraphGnssRightTransErrorTerm
{
public:
    PoseGraphGnssRightTransErrorTerm(const pose_qua_t &gnss_pose_measured,
                                     const pose_qua_t &calib_ldr2gnss_measured,
                                     const Eigen::Matrix<double, 6, 6> &sqrt_information) : gnss_pose_measured_(gnss_pose_measured), calib_ldr2gnss_measured_(calib_ldr2gnss_measured),
                                                                                            sqrt_information_(sqrt_information), ldr_pose_measured_(calib_ldr2gnss_measured * gnss_pose_measured) {}

    template <typename T>
    bool operator()(const T *const p_sm_ptr, const T *const q_sm_ptr,
                    const T *const p_dc_ptr, const T *const q_dc_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_sm(p_sm_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_sm(q_sm_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_dc(p_dc_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_dc(q_dc_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_dc_inverse = q_dc.conjugate();
        Eigen::Quaternion<T> q_sme_estimated = q_sm * q_dc_inverse;

        Eigen::Matrix<T, 3, 1> p_sme_estimated = p_sm - q_sm * q_dc_inverse * p_dc;

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
                p_sme_estimated - ldr_pose_measured_.trans.template cast<T>();
        residuals.template block<3, 1>(3, 0) = Eigen::Matrix<T, 3, 1>::Zero();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
            const pose_qua_t &gnss_pose_measured,
            const pose_qua_t &calib_ldr2gnss_measured,
            const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraphGnssRightTransErrorTerm, 6, 3, 4, 3, 4>(
                new PoseGraphGnssRightTransErrorTerm(gnss_pose_measured, calib_ldr2gnss_measured,
                                                     sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Gnss Measurement
    const pose_qua_t gnss_pose_measured_; // not used
    // Calibration Pose T_ldr_gnss (point_lidar = T_ldr_gnss * point_gnss)
    const pose_qua_t calib_ldr2gnss_measured_; // not used
    const pose_qua_t ldr_pose_measured_;
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

class BackEndOptimization
{
public:
    BackEndOptimization();

    BackEndOptimization(const optimization_param_t &params);

    BackEndOptimization(const VectorOfEdges &edges, const MapOfSubMaps &subMaps,
                        const MapOfGnsses &gnss_mea, const Pose3d &calib_ldr2gnss,
                        const Pose3d &calib_ldr2gnss_deltaT);

    ~BackEndOptimization();

    bool BuildProblem();

    bool SolveProblem();

    bool SubmapMae() const;

    bool FrameMae() const;

    bool CheckPGOResult(std::vector<submapid_error_t, Eigen::aligned_allocator<submapid_error_t> > &err_submaps,
                        std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > &err_edges) const;

    bool LoadParams(const optimization_param_t &params);

    bool SetPoseGraph(const VectorOfEdges &edges, const MapOfSubMaps &subMaps,
                      const MapOfGnsses &gnss_mea, const Pose3d &calib_ldr2gnss,
                      const Pose3d &calib_ldr2gnss_deltaT);

    const MapOfSubMaps *GetSubmaps() const;

    void PrintProblemSubmaps() const;

    //void TransformPointCloudForCloudCompare(int begin_id, int end_id, const bool GNSS = false, const bool PGO = false) const;

    void TransformPointCloudForCloudCompare(int begin_id, int end_id, const bool GNSS, const bool PGO,
                                            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_pgo_out_merge,
                                            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gnss_out_merge) const;

    void EvaluateSelectedArea(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &road_bbxs,
                              const std::map<int, int> &map_transaction_id_to_sequence);

    void RoadThickMapGeneration(int begin_id, int num) const;

    bool WritePoseToFile();

    void ShowPoseGraph();

    bool Optimize();

    bool ConsistencyCheck(std::vector<std::string> &pcd_files);

    bool LoadPoseGraphFromFile(MapOfSubMaps *submaps,
                               VectorOfEdges *edges) const;

    bool OptimizeRotAndTrans();

    bool OptimizeRotAndTransAndGnssAndCalib();

    bool OptimizeRot();

    bool OptimizeTrans();

    bool OptimizeRotAndTransAndGnss();

private:
    enum pgo_type_t
    {
        ROT,
        TRANS,
        ROT_TRANS,
        ROT_TRANS_CALIB,
        ROT_TRANS_GNSS,
        ROT_TRANS_GNSS_CALIB
    };

    bool SetParams();

    bool SetMinimizerOptions();

    bool SetPoseGraphEdge(VectorOfEdges &edges, pgo_type_t &pgo_type);

    bool SetPoseGraphGnssEdge(MapOfGnsses &map_of_gnsses, pgo_type_t &pgo_type);

    bool SetAdjacentEdge(pgo_type_t &pgo_type);

    bool SetIntraEdge(pgo_type_t &pgo_type);

    bool SetInterEdge(pgo_type_t &pgo_type);

    bool SetGnssEdge(pgo_type_t &pgo_type);

    bool DelEdges(const std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > &del_edges);

    bool DelEdge(const submap_pair_t &del_edge);

    bool SetLinearSolver();

    bool SetOrdering();

    // Update each frame pose in Submap, this function will be executed only once.
    bool UpdateFramePose();

    // Write Pose Out
    bool WritePoseOut();

    bool CheckPose(const Pose3d &pose) const;
    bool CheckInformationMatrix(const Eigen::Matrix<double, 6, 6> &information_matrix) const;
    bool CheckNanInf(const double &num) const;

    ceres::Problem problem_;
    ceres::Solver::Options options_;
    ceres::Solver::Summary summary_;
    ceres::LossFunction *loss_function_;
    optimization_param_t params_;
    DataIo<map_pose::PointType> data_loader_;
    CRegistration<map_pose::PointType> reg_;
    CFilter<map_pose::PointType> filter_;
    pgo_type_t pgo_type_ = ROT_TRANS;

public:
    Pose3d calib_ldr2gnss_dt_;     // pose graph node
    MapOfSubMaps sub_maps_;        // pose graph node
    VectorOfEdges intra_loops_;    // pose graph edge, loop between transactions
    VectorOfEdges inter_loops_;    // pose graph edge, loop in the same transaction
    VectorOfEdges adjacent_edges_; // pose graph edge, adjacent pose between sub maps
    MapOfGnsses gnsses_mea_;       // pose graph edge, edge between gnss pose and submap pose
    Pose3d calib_ldr2gnss_;        // calib measurement
    std::string pose_graph_output_folder_;
    std::string pose_graph_output_file_;
};

} // namespace map_pose

#endif // _INCLUDE_BACK_END_OPTIMIZATION_H_

