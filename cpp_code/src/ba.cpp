#include <ceres/loss_function.h>
#include <glog/logging.h>
#include <cmath>

#include <chrono>

#include "ba.h"

namespace p3dv
{

bool BundleAdjustment::setBAProblem(std::vector<frame_t> &frames, std::vector<bool> &process_frame_id,
                                    pointcloud_sparse_t &sfm_sparse_points, double fix_calib_tolerance_BA,
                                    int reference_frame_id)
{
    num_observations_ = 0;
    num_cameras_ = 0;
    num_points_ = sfm_sparse_points.unique_point_ids.size();

    //each observation correspond to a point and a camera

    for (int i = 0; i < frames.size(); i++)
    {
        if (!process_frame_id[i]) // 0 for processing
        {
            calibs_.push_back(frames[i].K_cam);
            for (int k = 0; k < sfm_sparse_points.unique_point_ids.size(); k++)
            {
                for (int j = 0; j < frames[i].unique_pixel_ids.size(); j++)
                {
                    if (frames[i].unique_pixel_has_match[j])
                    {
                        if (/*sfm_sparse_points.is_inlier[k] && */
                            (frames[i].unique_pixel_ids[j] == sfm_sparse_points.unique_point_ids[k]))
                        {
                            //Set the none-changed observations
                            points_2d_.push_back(frames[i].keypoints[j].pt);

                            //Set the mapping function
                            point_index_.push_back(k);
                            camera_index_.push_back(num_cameras_);

                            num_observations_++;
                            break;
                        }
                    }
                }
            }
            if (i == reference_frame_id) //Set camera that need to be fixed
            {
                ref_process_camera_id_ = num_cameras_;
            }
            num_cameras_++;
            std::cout << "Process frame " << i << " Done" << std::endl;
        }
    }

    std::cout << "Find the observation done" << std::endl;

    observations_ = new double[2 * num_observations_];

    num_parameters_ = 6 * num_cameras_ + 3 * num_points_;

    if (fix_calib_tolerance_BA != 0)
        num_parameters_ += 4;

    parameters_ = new double[num_parameters_];

    //Set initial guess of parameters for estimating: rot_vec, tran_vec and the point_3d
    int k = 0; //count of the used cameras
    for (int i = 0; i < frames.size(); i++)
    {
        if (!process_frame_id[i]) // 0 for processing
        {
            cv::Mat rot_Mat;
            cv::Mat rot_vec;
            Eigen::Matrix3f rot_Mat_eigen;
            rot_Mat_eigen = frames[i].pose_cam.block<3, 3>(0, 0);

            cv::eigen2cv(rot_Mat_eigen, rot_Mat);

            cv::Rodrigues(rot_Mat, rot_vec);

            //std::cout << rot_vec.rows << "," << rot_vec.cols << std::endl;

            parameters_[6 * k] = rot_vec.at<float>(0, 0);
            parameters_[6 * k + 1] = rot_vec.at<float>(1, 0);
            parameters_[6 * k + 2] = rot_vec.at<float>(2, 0);
            parameters_[6 * k + 3] = frames[i].pose_cam(0, 3);
            parameters_[6 * k + 4] = frames[i].pose_cam(1, 3);
            parameters_[6 * k + 5] = frames[i].pose_cam(2, 3);

            k++;
        }
    }

    std::cout << "Set camera pose parameters inital value done" << std::endl;

    for (int i = 0; i < sfm_sparse_points.unique_point_ids.size(); i++)
    {
        parameters_[6 * num_cameras_ + 3 * i] = sfm_sparse_points.rgb_pointcloud->points[i].x;
        parameters_[6 * num_cameras_ + 3 * i + 1] = sfm_sparse_points.rgb_pointcloud->points[i].y;
        parameters_[6 * num_cameras_ + 3 * i + 2] = sfm_sparse_points.rgb_pointcloud->points[i].z;
    }
    std::cout << "Set points inital value done" << std::endl;

    if (fix_calib_tolerance_BA != 0)
    {
        parameters_[num_parameters_ - 4] = calibs_[0](0, 0);
        parameters_[num_parameters_ - 3] = calibs_[0](0, 2);
        parameters_[num_parameters_ - 2] = calibs_[0](1, 1);
        parameters_[num_parameters_ - 1] = calibs_[0](1, 2);
        std::cout << "Set camera intrinsic parameters inital value done" << std::endl;
    }

    std::cout << "Find [ " << num_observations_ << " ] Observations in total." << std::endl;
    std::cout << "There are [ " << num_cameras_ << " ] cameras and [ " << num_points_ << " ] 3D points." << std::endl;
    std::cout << "So there should be [ " << num_parameters_ << " ] unknown parameters with [ " << 2 * num_observations_ << " ] observation functions." << std::endl;

    if (2 * num_observations_ > num_parameters_)
    {
        std::cout << "Ready to solve " << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Too much unknown parameters ... " << std::endl;
        return 0;
    }
}

bool BundleAdjustment::solveBA(double fix_calib_tolerance_BA)
{
    ceres::Problem problem;

    std::cout << "Begin to calculate the loss (reprojection error)" << std::endl;

    double fixed_threshold = 1e-10; //For reference frame

    if (fix_calib_tolerance_BA == 0) // only camera pose and 3d points are set as parameters
    {
        for (int i = 0; i < num_observations_; i++)
        {
            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.

            ceres::CostFunction *cost_function =
                ReprojectErrorTerm_fixcalib::Create(points_2d_[i], calibs_[camera_index_[i]]);

            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(0.5), // NULL /* squared loss */, /* new CauchyLoss(0.5) */
                                     mutable_camera_for_observation(i), mutable_point_for_observation(i));

            // later figure out how to set the first frame fixed.
            //Fixed reference frame's parameters
            if (camera_index_[i] == ref_process_camera_id_)
            {
                for (int j = 0; j < 6; j++)
                {
                    problem.SetParameterLowerBound(mutable_camera_for_observation(i), j, -fixed_threshold);
                    problem.SetParameterUpperBound(mutable_camera_for_observation(i), j, fixed_threshold);
                }
            }
        }
    }
    else // camera intrinsic parameters are also set as parameters
    {
        for (int i = 0; i < num_observations_; i++)
        {
            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.

            ceres::CostFunction *cost_function =
                ReprojectErrorTerm_updatecalib::Create(points_2d_[i]);

            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(0.5), // NULL /* squared loss */, /* new CauchyLoss(0.5) */
                                     mutable_camera_for_observation(i), mutable_point_for_observation(i), mutable_calib());

            // later figure out how to set the first frame fixed.f
            //Fixed reference frame's parameters
            if (camera_index_[i] == ref_process_camera_id_)
            {
                for (int j = 0; j < 6; j++)
                {
                    problem.SetParameterLowerBound(mutable_camera_for_observation(i), j, -fixed_threshold);
                    problem.SetParameterUpperBound(mutable_camera_for_observation(i), j, fixed_threshold);
                }
            }
        }
        for (int i = 0; i < 4; i++)
        {
            problem.SetParameterLowerBound(mutable_calib(), i, parameters_[num_parameters_ - (4 - i)] - fix_calib_tolerance_BA);
            problem.SetParameterUpperBound(mutable_calib(), i, parameters_[num_parameters_ - (4 - i)] + fix_calib_tolerance_BA);
        }
    }
    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;
    options.num_threads = 4;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << summary.FullReport() << std::endl;
    std::cout << summary.BriefReport() << std::endl;

    return 1;
}

bool BundleAdjustment::doSFMBA(std::vector<frame_t> &frames, std::vector<bool> &process_frame_id,
                               pointcloud_sparse_t &sfm_sparse_points, double fix_calib_tolerance_BA, int reference_frame_id)
{
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    initBA();
    setBAProblem(frames, process_frame_id, sfm_sparse_points, fix_calib_tolerance_BA, reference_frame_id);
    solveBA(fix_calib_tolerance_BA);

    //update the data
    //update camera pose (and K matrix)
    int k = 0;
    for (int i = 0; i < frames.size(); i++)
    {
        if (!process_frame_id[i]) // 0 for processing
        {
            cv::Mat rot_vec = cv::Mat::ones(3, 1, CV_32F);
            cv::Mat rot_Mat;

            Eigen::Matrix3f rot_Mat_eigen;

            rot_vec.at<float>(0, 0) = parameters_[6 * k];
            rot_vec.at<float>(1, 0) = parameters_[6 * k + 1];
            rot_vec.at<float>(2, 0) = parameters_[6 * k + 2];

            cv::Rodrigues(rot_vec, rot_Mat);
            cv::cv2eigen(rot_Mat, rot_Mat_eigen);

            frames[i].pose_cam.block<3, 3>(0, 0) = rot_Mat_eigen;

            frames[i].pose_cam(0, 3) = parameters_[6 * k + 3];
            frames[i].pose_cam(1, 3) = parameters_[6 * k + 4];
            frames[i].pose_cam(2, 3) = parameters_[6 * k + 5];

            k++;

            if (fix_calib_tolerance_BA != 0)
            {
                frames[i].K_cam(0, 0) = parameters_[num_parameters_ - 4];
                frames[i].K_cam(0, 2) = parameters_[num_parameters_ - 3];
                frames[i].K_cam(1, 1) = parameters_[num_parameters_ - 2];
                frames[i].K_cam(1, 2) = parameters_[num_parameters_ - 1];
            }
        }
    }

    if (fix_calib_tolerance_BA != 0)
    {
        std::cout << "Calib intrinsic parameters after BA:" << std::endl
                  << "[fx:" << parameters_[num_parameters_ - 4] << " ,cx:" << parameters_[num_parameters_ - 3]
                  << " ,fy:" << parameters_[num_parameters_ - 2] << " ,cy:" << parameters_[num_parameters_ - 1] << " ]"
                  << std::endl;
    }

    //update 3d points
    int coord_thre = 200;
    for (int i = 0; i < sfm_sparse_points.unique_point_ids.size(); i++)
    {
        // if (std::abs(parameters_[6 * num_cameras_ + 3 * i]) < coord_thre &&
        //     std::abs(parameters_[6 * num_cameras_ + 3 * i + 1]) < coord_thre &&
        //     std::abs(parameters_[6 * num_cameras_ + 3 * i + 2]) < coord_thre)
        // {

        sfm_sparse_points.rgb_pointcloud->points[i].x = parameters_[6 * num_cameras_ + 3 * i];
        sfm_sparse_points.rgb_pointcloud->points[i].y = parameters_[6 * num_cameras_ + 3 * i + 1];
        sfm_sparse_points.rgb_pointcloud->points[i].z = parameters_[6 * num_cameras_ + 3 * i + 2];
        //}
    }

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    std::cout << "Bundle Ajustment cost = " << time_used.count() << " seconds. " << std::endl;

    std::cout << "Bundle Ajustment done." << std::endl;
}

} // namespace p3dv
