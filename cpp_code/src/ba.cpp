#if 0

#include <ceres/loss_function.h>
#include <glog/logging.h>
#include <cmath>

#include "ba.h"

namespace p3dv
{
    double getThetaFromQuaternion(Eigen::Quaterniond rot) {
        // Eigen::Quaternion [x, y, z, w] = [sin(theta/2)n, cos(theta/2)]; define theta in quaternion is [-pi, +pi]
        // use actan [-pi/2, +pi/2](-infinite, +infinite) to calculate theta/2
        double two_theta = 0.0;
        double sin_squared_theta = rot.x() * rot.x() + rot.y() * rot.y() + rot.z() * rot.z();
        if (sin_squared_theta > 0.0)
        {
            double sin_theta = sqrt(sin_squared_theta);
            double cos_theta = rot.w();
            two_theta = 2 * ((cos_theta < 0.0)
                             ? atan2(-sin_theta, -cos_theta)
                             : atan2(sin_theta, cos_theta));
        }
        else
        {
            two_theta = 0.0;
        }

        return two_theta;   // unit rad
    }

bool calc_edge_error(const MapOfSubMaps &submaps, const VectorOfEdges &edges,
        std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > &err_edges) {
    for (int i = 0; i < edges.size(); ++i) {
        edgeid_error_t edgeidError;
        MapOfSubMaps::const_iterator subMaps_begin_itr = submaps.find(edges[i].submap_idx.first_submap);
        CHECK(subMaps_begin_itr != submaps.end())
        << "Submap With ID: " << edges[i].submap_idx.first_submap << "not found.";

        MapOfSubMaps::const_iterator subMaps_end_itr = submaps.find(edges[i].submap_idx.second_submap);
        CHECK(subMaps_end_itr != submaps.end())
        << "Submap With ID: " << edges[i].submap_idx.second_submap << "not found.";
        Eigen::Matrix4d pose_pgo =
                subMaps_begin_itr->second.pose.GetMatrix().inverse() * subMaps_end_itr->second.pose.GetMatrix();
        edgeidError.pose_error.SetPose(pose_pgo.inverse() * edges[i].pose.GetMatrix());
        edgeidError.edge = edges[i];
        edgeidError.theta = getThetaFromQuaternion(edgeidError.pose_error.quat);
        err_edges.push_back(edgeidError);
    }
}

bool greater_submapid_error_t(const submapid_error_t &sie1, const submapid_error_t &sie2) {
    return (sie1.pose_error.trans.norm() - sie2.pose_error.trans.norm()) > 0;
}

bool greater_edgeid_error_t(const edgeid_error_t &eie1, const edgeid_error_t &eie2) {
    return (eie1.pose_error.trans.norm() - eie2.pose_error.trans.norm()) > 0;
}

BackEndOptimization::BackEndOptimization() {}

BackEndOptimization::BackEndOptimization(const VectorOfEdges &edges, const MapOfSubMaps &subMaps,
                                         const MapOfGnsses &gnss_mea, const Pose3d &calib_ldr2gnss,
                                         const Pose3d &calib_ldr2gnss_deltaT)
{
    SetPoseGraph(edges, subMaps, gnss_mea, calib_ldr2gnss, calib_ldr2gnss_deltaT);
}

BackEndOptimization::BackEndOptimization(const optimization_param_t &params)
{
    LoadParams(params);
}

BackEndOptimization::~BackEndOptimization() {}

bool BackEndOptimization::LoadParams(const optimization_param_t &params)
{
    params_ = params;
    SetParams();

    return true;
}

bool BackEndOptimization::SetParams()
{
    SetMinimizerOptions();
    //        SetLinearSolver();
    //        SetOrdering();

    return true;
}

bool BackEndOptimization::SetMinimizerOptions()
{
    options_.max_num_iterations = params_.num_iterations;
    options_.minimizer_progress_to_stdout = true;
    options_.num_threads = params_.num_threads;
    //! Terminate Condition
    //    options_.gradient_tolerance = 1e-12;
    //    options_.function_tolerance = 1e-12;
    //    options_.parameter_tolerance = 1e-12;

    //    options_.logging_type = ceres::SILENT;
    loss_function_ = params_.robustify ? new ceres::HuberLoss(1.0) : NULL;
    CHECK(ceres::StringToTrustRegionStrategyType(params_.trust_region_strategy,
                                                 &options_.trust_region_strategy_type));

    return true;
}

bool BackEndOptimization::SetLinearSolver()
{
    CHECK(ceres::StringToLinearSolverType(params_.linear_solver,
                                          &options_.linear_solver_type));
    CHECK(ceres::StringToSparseLinearAlgebraLibraryType(params_.sparse_linear_algebra_library,
                                                        &options_.sparse_linear_algebra_library_type));
    CHECK(ceres::StringToDenseLinearAlgebraLibraryType(params_.dense_linear_algebra_library,
                                                       &options_.dense_linear_algebra_library_type));

    //options_.num_linear_solver_threads = params_.num_threads;
    //"num_linear_solver_threads" has been deprecated in current release of ceres, use num_threads instead.

    options_.num_threads = params_.num_threads;
    return true;
}

bool BackEndOptimization::SetOrdering()
{
    if (params_.ordering == "automatic")
        return true;
    // TODO
    return true;
}

bool BackEndOptimization::SetPoseGraph(const VectorOfEdges &edges, const MapOfSubMaps &subMaps,
                                       const MapOfGnsses &gnss_mea, const Pose3d &calib_ldr2gnss,
                                       const Pose3d &calib_ldr2gnss_deltaT)
{
        sub_maps_.clear();
        gnsses_mea_.clear();
        adjacent_edges_.clear();
        inter_loops_.clear();
        intra_loops_.clear();
    sub_maps_ = subMaps; // ATTENTION: value copy, address isn't copied
    gnsses_mea_ = gnss_mea;
    calib_ldr2gnss_ = calib_ldr2gnss;
    for (size_t i = 0; i < edges.size(); ++i)
    {
        // delete wrong edges
        if (edges[i].information_matrix == Eigen::Matrix<double, 6, 6>::Zero())
        {
            continue;
        }
        if (edges[i].edge_type == Edge::Adjacent)
            adjacent_edges_.push_back(edges[i]);
        else if (edges[i].edge_type == Edge::Inter)
            inter_loops_.push_back(edges[i]);
        else if (edges[i].edge_type == Edge::Intra)
            intra_loops_.push_back(edges[i]);
        else
            LOG(FATAL) << "WRONG EDGE TYPE!";
    }

    return true;
}

const MapOfSubMaps *BackEndOptimization::GetSubmaps() const
{
    const MapOfSubMaps *sub_maps_ptr = &sub_maps_;
    return sub_maps_ptr;
}

bool BackEndOptimization::SetPoseGraphEdge(VectorOfEdges &edges, pgo_type_t &pgo_type) {
    for (VectorOfEdges::const_iterator edge_iter = edges.begin();
         edge_iter != edges.end(); ++edge_iter) {
        MapOfSubMaps::iterator subMaps_begin_itr = sub_maps_.find(edge_iter->submap_idx.first_submap);
        CHECK(subMaps_begin_itr != sub_maps_.end())
        << "Submap With ID: " << edge_iter->submap_idx.first_submap << "not found.";

        MapOfSubMaps::iterator subMaps_end_itr = sub_maps_.find(edge_iter->submap_idx.second_submap);
        CHECK(subMaps_end_itr != sub_maps_.end())
        << "Submap With ID: " << edge_iter->submap_idx.second_submap << "not found.";

        CHECK(CheckPose(edge_iter->pose)) << "Edge Pose NAN INF, Edge is " << edge_iter->submap_idx;
        CHECK(CheckInformationMatrix(edge_iter->information_matrix)) << "Edge Information Matrix NAN INF, Edge is " << edge_iter->submap_idx;

        const double ratio_registration_over_gnss = 5;
        const Eigen::Matrix<double, 6, 6> sqrt_information =
                ratio_registration_over_gnss * edge_iter->information_matrix;
        CHECK(CheckPose(subMaps_begin_itr->second.pose)) << "Submap NAN INF, submap is " << subMaps_begin_itr->first;
        CHECK(CheckPose(subMaps_end_itr->second.pose)) << "Submap NAN INF, submap is " << subMaps_end_itr->first;

        // Manual Set Information Matrix
        Eigen::Matrix<double, 6, 6> fix_information;
        const double ratio_rot_over_tran = 5;
        fix_information << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, ratio_rot_over_tran, 0, 0,
                0, 0, 0, 0, ratio_rot_over_tran, 0,
                0, 0, 0, 0, 0, ratio_rot_over_tran;

        fix_information *= ratio_registration_over_gnss;

        ceres::CostFunction *cost_function = NULL;
        if (pgo_type == ROT)
        {
            cost_function = PoseGraph3dErrorTermROT::Create(edge_iter->pose, sqrt_information);
        }
        else if (pgo_type == TRANS)
        {
            cost_function = PoseGraph3dErrorTermTRANS::Create(edge_iter->pose, sqrt_information);
        }
        else
        {
            cost_function = PoseGraph3dErrorTermQUAT::Create(edge_iter->pose, fix_information);
            //            cost_function = PoseGraph3dErrorTermQUAT::Create(edge_iter->pose, sqrt_information);
        }
        problem_.AddResidualBlock(cost_function, loss_function_,
                                  subMaps_begin_itr->second.pose.trans.data(),
                                  subMaps_begin_itr->second.pose.quat.coeffs().data(),
                                  subMaps_end_itr->second.pose.trans.data(),
                                  subMaps_end_itr->second.pose.quat.coeffs().data());

        // Set Submap Bounds
        if (problem_.HasParameterBlock(subMaps_begin_itr->second.pose.trans.data()))
        {
            problem_.SetParameterLowerBound(subMaps_begin_itr->second.pose.trans.data(), 0, subMaps_begin_itr->second.pose.trans[0] - params_.submap_lower_bound_x);
            problem_.SetParameterLowerBound(subMaps_begin_itr->second.pose.trans.data(), 1, subMaps_begin_itr->second.pose.trans[1] - params_.submap_lower_bound_y);
            problem_.SetParameterLowerBound(subMaps_begin_itr->second.pose.trans.data(), 2, subMaps_begin_itr->second.pose.trans[2] - params_.submap_lower_bound_z);
            problem_.SetParameterUpperBound(subMaps_begin_itr->second.pose.trans.data(), 0, subMaps_begin_itr->second.pose.trans[0] + params_.submap_upper_bound_x);
            problem_.SetParameterUpperBound(subMaps_begin_itr->second.pose.trans.data(), 1, subMaps_begin_itr->second.pose.trans[1] + params_.submap_upper_bound_y);
            problem_.SetParameterUpperBound(subMaps_begin_itr->second.pose.trans.data(), 2, subMaps_begin_itr->second.pose.trans[2] + params_.submap_upper_bound_z);
        }
        if (problem_.HasParameterBlock(subMaps_end_itr->second.pose.trans.data()))
        {
            problem_.SetParameterLowerBound(subMaps_end_itr->second.pose.trans.data(), 0, subMaps_end_itr->second.pose.trans[0] - params_.submap_lower_bound_x);
            problem_.SetParameterLowerBound(subMaps_end_itr->second.pose.trans.data(), 1, subMaps_end_itr->second.pose.trans[1] - params_.submap_lower_bound_y);
            problem_.SetParameterLowerBound(subMaps_end_itr->second.pose.trans.data(), 2, subMaps_end_itr->second.pose.trans[2] - params_.submap_lower_bound_z);
            problem_.SetParameterUpperBound(subMaps_end_itr->second.pose.trans.data(), 0, subMaps_end_itr->second.pose.trans[0] + params_.submap_upper_bound_x);
            problem_.SetParameterUpperBound(subMaps_end_itr->second.pose.trans.data(), 1, subMaps_end_itr->second.pose.trans[1] + params_.submap_upper_bound_y);
            problem_.SetParameterUpperBound(subMaps_end_itr->second.pose.trans.data(), 2, subMaps_end_itr->second.pose.trans[2] + params_.submap_upper_bound_z);
        }
    }

    return true;
}

bool BackEndOptimization::SetPoseGraphGnssEdge(MapOfGnsses &map_of_gnsses, pgo_type_t &pgo_type)
{
    for (MapOfGnsses::const_iterator gnss_iter = map_of_gnsses.begin();
         gnss_iter != map_of_gnsses.end(); ++gnss_iter)
    {
        auto subMaps_itr = sub_maps_.find(gnss_iter->first);
        CHECK(subMaps_itr != sub_maps_.end())
        << "Submap With ID: " << gnss_iter->first << "not found.";

        CHECK(CheckPose(gnss_iter->second.pose)) << "Gnss Pose NAN INF, Gnss is " << gnss_iter->first;
        CHECK(CheckInformationMatrix(gnss_iter->second.information_matrix)) << "Gnss InformationMatrix NAN INF, Gnss is " << gnss_iter->first;
        const Eigen::Matrix<double, 6, 6> sqrt_information = gnss_iter->second.information_matrix;

        const double ratio_rot_over_tran = 0;
        // Manual Set Information Matrix
        Eigen::Matrix<double, 6, 6> fix_information;
        fix_information << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, ratio_rot_over_tran, 0, 0,
                0, 0, 0, 0, ratio_rot_over_tran, 0,
                0, 0, 0, 0, 0, ratio_rot_over_tran;

        //        ceres::CostFunction *cost_function =
        //                PoseGraphGnssErrorTerm::Create(gnss_iter->second.pose, calib_ldr2gnss_, sqrt_information);
        ceres::CostFunction *cost_function = NULL;
        if (pgo_type == ROT_TRANS_GNSS_CALIB || pgo_type == ROT_TRANS_GNSS)
        {
            //            LOG(WARNING) << "GNSS Edge is Added!";
            cost_function = PoseGraphGnssRightErrorTerm::Create(gnss_iter->second.pose, calib_ldr2gnss_, fix_information);
            //            cost_function = PoseGraphGnssRightErrorTerm::Create(gnss_iter->second.pose, calib_ldr2gnss_, sqrt_information);
            problem_.AddResidualBlock(cost_function, loss_function_,
                                      subMaps_itr->second.pose.trans.data(),
                                      subMaps_itr->second.pose.quat.coeffs().data(),
                                      calib_ldr2gnss_dt_.trans.data(),
                                      calib_ldr2gnss_dt_.quat.coeffs().data());
        }
        else
        {
            //            LOG(WARNING) << "GNSS Edge isn't Added!";
        }

        // Set Submap Bounds
        if (problem_.HasParameterBlock(subMaps_itr->second.pose.trans.data()))
        {
            problem_.SetParameterLowerBound(subMaps_itr->second.pose.trans.data(), 0, subMaps_itr->second.pose.trans[0] - params_.submap_lower_bound_x);
            problem_.SetParameterLowerBound(subMaps_itr->second.pose.trans.data(), 1, subMaps_itr->second.pose.trans[1] - params_.submap_lower_bound_y);
            problem_.SetParameterLowerBound(subMaps_itr->second.pose.trans.data(), 2, subMaps_itr->second.pose.trans[2] - params_.submap_lower_bound_z);
            problem_.SetParameterUpperBound(subMaps_itr->second.pose.trans.data(), 0, subMaps_itr->second.pose.trans[0] + params_.submap_upper_bound_x);
            problem_.SetParameterUpperBound(subMaps_itr->second.pose.trans.data(), 1, subMaps_itr->second.pose.trans[1] + params_.submap_upper_bound_y);
            problem_.SetParameterUpperBound(subMaps_itr->second.pose.trans.data(), 2, subMaps_itr->second.pose.trans[2] + params_.submap_upper_bound_z);
        }
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.trans.data()))
        {
            problem_.SetParameterLowerBound(calib_ldr2gnss_dt_.trans.data(), 0, calib_ldr2gnss_dt_.trans[0] - params_.calib_lower_bound_x);
            problem_.SetParameterLowerBound(calib_ldr2gnss_dt_.trans.data(), 1, calib_ldr2gnss_dt_.trans[1] - params_.calib_lower_bound_y);
            problem_.SetParameterLowerBound(calib_ldr2gnss_dt_.trans.data(), 2, calib_ldr2gnss_dt_.trans[2] - params_.calib_lower_bound_z);
            problem_.SetParameterUpperBound(calib_ldr2gnss_dt_.trans.data(), 0, calib_ldr2gnss_dt_.trans[0] + params_.calib_upper_bound_x);
            problem_.SetParameterUpperBound(calib_ldr2gnss_dt_.trans.data(), 1, calib_ldr2gnss_dt_.trans[1] + params_.calib_upper_bound_y);
            problem_.SetParameterUpperBound(calib_ldr2gnss_dt_.trans.data(), 2, calib_ldr2gnss_dt_.trans[2] + params_.calib_upper_bound_z);
        }
    }

    return true;
}

bool BackEndOptimization::SetAdjacentEdge(pgo_type_t &pgo_type)
{
    CHECK(SetPoseGraphEdge(adjacent_edges_, pgo_type)) << "Set Adjacent Edges Failed!";
    LOG(WARNING) << "Add Adjacent Edge into Pose Graph!";
    return true;
}

bool BackEndOptimization::SetIntraEdge(pgo_type_t &pgo_type)
{
    CHECK(SetPoseGraphEdge(intra_loops_, pgo_type)) << "Set Adjacent Edges Failed!";
    LOG(WARNING) << "Add Intra Edge into Pose Graph!";
    return true;
}

bool BackEndOptimization::SetInterEdge(pgo_type_t &pgo_type)
{
    CHECK(SetPoseGraphEdge(inter_loops_, pgo_type)) << "Set Adjacent Edges Failed!";
    LOG(WARNING) << "Add Inter Edge into Pose Graph!";
    return true;
}

bool BackEndOptimization::SetGnssEdge(pgo_type_t &pgo_type)
{
    CHECK(SetPoseGraphGnssEdge(gnsses_mea_, pgo_type)) << "Set Gnss Edges Failed!";
    LOG(WARNING) << "Add Gnss Edge into Pose Graph!";
    return true;
}

bool BackEndOptimization::BuildProblem()
{
    CHECK(!adjacent_edges_.empty() ||
          !intra_loops_.empty() ||
          !inter_loops_.empty() ||
          !gnsses_mea_.empty())
    << "No Edges!";
    CHECK(!sub_maps_.empty()) << "No Optimization Variable!";

    printf("======================\n"
           "NumParameters is: %d\n"
           "NumParametersBlocks is: %d\n"
           "NumResiduals is: %d\n"
           "NumResidualBlocks is: %d\n"
           "======================\n",
           problem_.NumParameters(), problem_.NumParameterBlocks(), problem_.NumResiduals(), problem_.NumResidualBlocks());

    // clear residual blocks
    std::vector<ceres::ResidualBlockId> residual_blocks;
    problem_.GetResidualBlocks(&residual_blocks);
    for (int i = 0; i < residual_blocks.size(); ++i)
    {
        problem_.RemoveResidualBlock(residual_blocks[i]);
    }

    // add residual blocks
    if (!adjacent_edges_.empty())
        SetAdjacentEdge(pgo_type_);
     if (!intra_loops_.empty())
         SetIntraEdge(pgo_type_);
     if (!inter_loops_.empty())
         SetInterEdge(pgo_type_);
    if (!gnsses_mea_.empty())
        SetGnssEdge(pgo_type_);

    //  NumParameters = (num_cameras_ * camera_block_size + num_points_ * point_block_size) Jacobian cols数
    //  NumParametersBlocks = (num_cameras_ + num_points_) pose数 + landmark数
    //  NumResiduals = 2 * num_observations 图像u,v两个轴误差
    //  NumResidualBlocks = num_observations 观测次数/error数/图优化边数
    printf("======================\n"
           "NumParameters is: %d\n"
           "NumParametersBlocks is: %d\n"
           "NumResiduals is: %d\n"
           "NumResidualBlocks is: %d\n"
           "======================\n",
           problem_.NumParameters(), problem_.NumParameterBlocks(), problem_.NumResiduals(), problem_.NumResidualBlocks());

    // Set All Quaternion
    // 1. quaternion has 4 DOFs, while rotation is a 3 DOF transformation, so quaternion overparameterizes rotation.
    // the constraint is quat = [cos(theta/2), nx*sin(theta/2), ny*sin(theta/2), nz*sin(theta/2)]
    // 2. Eigen stores quaternion as [i, j, k, w]
    // use EigenQuaternionParameterzation to avoid Problem 1 and Problem 2
    static ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
    if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.quat.coeffs().data()))
    {
        problem_.SetParameterization(calib_ldr2gnss_dt_.quat.coeffs().data(), quaternion_local_parameterization);
    }
    for (auto submap_itr = sub_maps_.begin(); submap_itr != sub_maps_.cend(); ++submap_itr)
    {
        if (problem_.HasParameterBlock(submap_itr->second.pose.quat.coeffs().data()))
        {
            problem_.SetParameterization(submap_itr->second.pose.quat.coeffs().data(),
                                         quaternion_local_parameterization);
        }
    }
    // Set Constant/Variable according to pgo_type
    if (pgo_type_ == ROT)
    {
        // Rot -- Variable | Trans -- Constant | calib_dt -- Constant
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.trans.data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.trans.data());
        }
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.quat.coeffs().data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.quat.coeffs().data());
        }
        for (auto submap_itr = sub_maps_.begin(); submap_itr != sub_maps_.cend(); ++submap_itr)
        {
            if (problem_.HasParameterBlock(submap_itr->second.pose.quat.coeffs().data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.quat.coeffs().data());
            }
            if (problem_.HasParameterBlock(submap_itr->second.pose.trans.data()))
            {
                problem_.SetParameterBlockConstant(submap_itr->second.pose.trans.data());
            }
        }
    }
    else if (pgo_type_ == TRANS)
    {
        // Rot -- Constant | Trans -- Variable | calib_dt -- Constant
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.trans.data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.trans.data());
        }
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.quat.coeffs().data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.quat.coeffs().data());
        }
        for (auto submap_itr = sub_maps_.begin(); submap_itr != sub_maps_.cend(); ++submap_itr)
        {
            if (problem_.HasParameterBlock(submap_itr->second.pose.trans.data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.trans.data());
            }
            if (problem_.HasParameterBlock(submap_itr->second.pose.quat.coeffs().data()))
            {
                problem_.SetParameterBlockConstant(submap_itr->second.pose.quat.coeffs().data());
            }
        }
    }
    else if (pgo_type_ == ROT_TRANS)
    {
        // Rot -- Variable | Trans -- Variable | calib_dt -- Constant
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.trans.data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.trans.data());
        }
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.quat.coeffs().data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.quat.coeffs().data());
        }
        for (auto submap_itr = sub_maps_.begin(); submap_itr != sub_maps_.cend(); ++submap_itr)
        {
            if (problem_.HasParameterBlock(submap_itr->second.pose.trans.data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.trans.data());
            }
            if (problem_.HasParameterBlock(submap_itr->second.pose.quat.coeffs().data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.quat.coeffs().data());
            }
        }
    }
    else if (pgo_type_ == ROT_TRANS_GNSS)
    {
        // Rot -- Variable | Trans -- Variable | calib_dt -- Constant
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.trans.data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.trans.data());
        }
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.quat.coeffs().data()))
        {
            problem_.SetParameterBlockConstant(calib_ldr2gnss_dt_.quat.coeffs().data());
        }
        for (auto submap_itr = sub_maps_.begin(); submap_itr != sub_maps_.cend(); ++submap_itr)
        {
            if (problem_.HasParameterBlock(submap_itr->second.pose.trans.data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.trans.data());
            }
            if (problem_.HasParameterBlock(submap_itr->second.pose.quat.coeffs().data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.quat.coeffs().data());
            }
        }
    }
    else if (pgo_type_ == ROT_TRANS_GNSS_CALIB)
    {
        // Rot -- Variable | Trans -- Variable | calib_dt -- Variable
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.trans.data()))
        {
            problem_.SetParameterBlockVariable(calib_ldr2gnss_dt_.trans.data());
        }
        if (problem_.HasParameterBlock(calib_ldr2gnss_dt_.quat.coeffs().data()))
        {
            problem_.SetParameterBlockVariable(calib_ldr2gnss_dt_.quat.coeffs().data());
        }
        for (auto submap_itr = sub_maps_.begin(); submap_itr != sub_maps_.cend(); ++submap_itr)
        {
            if (problem_.HasParameterBlock(submap_itr->second.pose.trans.data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.trans.data());
                //                problem_.SetParameterBlockConstant(submap_itr->second.pose.trans.data());
            }
            if (problem_.HasParameterBlock(submap_itr->second.pose.quat.coeffs().data()))
            {
                problem_.SetParameterBlockVariable(submap_itr->second.pose.quat.coeffs().data());
                //                problem_.SetParameterBlockConstant(submap_itr->second.pose.quat.coeffs().data());
            }
        }
    }
    else
    {
        LOG(FATAL) << "Not Defined!";
    }

    // fix the first submap in each transaction to avoid gauge freedom
#if 1
    size_t fix_submaps_num = 0;
    for (MapOfSubMaps::iterator submaps_iter = sub_maps_.begin();
         submaps_iter != sub_maps_.end(); ++submaps_iter)
    {
        if (submaps_iter->first.submap_id == fix_submaps_num)
        {
            problem_.SetParameterBlockConstant(submaps_iter->second.pose.trans.data());
            problem_.SetParameterBlockConstant(submaps_iter->second.pose.quat.coeffs().data());
            LOG(WARNING) << submaps_iter->first << " is Fixed during Optimization!";
        }
    }
    LOG(WARNING) << "fix_submaps_num is " << fix_submaps_num;
#endif

    return true;
}

bool BackEndOptimization::SolveProblem()
{
    SetParams();
    std::cout << "=============================================  Optimization Start  =============================================" << std::endl;
    ceres::Solve(options_, &problem_, &summary_);

    //        std::cout << summary_.FullReport() << std::endl;
    std::cout << "==============================================  Optimization END  ==============================================" << std::endl;

    return summary_.IsSolutionUsable();
}

bool BackEndOptimization::SubmapMae() const
{
    double rot_err_scalar = 0;
    double trans_err_scalar = 0;
    double trans_err_x = 0;
    double trans_err_y = 0;
    double trans_err_z = 0;
    double trans_err_xy = 0;

    auto submaps_itr = sub_maps_.cbegin();

    while (submaps_itr != sub_maps_.end())
    {
        Eigen::Quaterniond rot_err = submaps_itr->second.raw_data_group[0].raw_gnss.pose.quat.conjugate() * submaps_itr->second.pose.quat;
        Eigen::Vector3d trans_err = submaps_itr->second.raw_data_group[0].raw_gnss.pose.trans - submaps_itr->second.pose.trans;

        // Eigen::Quaternion [x, y, z, w] = [sin(theta/2)n, cos(theta/2)]; define theta in quaternion is [-pi, +pi]
        // use actan [-pi/2, +pi/2](-infinite, +infinite) to calculate theta/2
        double two_theta = 0.0;
        double sin_squared_theta = rot_err.x() * rot_err.x() + rot_err.y() * rot_err.y() + rot_err.z() * rot_err.z();
        if (sin_squared_theta > 0.0)
        {
            double sin_theta = sqrt(sin_squared_theta);
            double cos_theta = rot_err.w();
            two_theta = 2 * ((cos_theta < 0.0)
                             ? atan2(-sin_theta, -cos_theta)
                             : atan2(sin_theta, cos_theta));
        }
        else
        {
            two_theta = 0.0;
        }

        rot_err_scalar += fabs(two_theta);
        trans_err_scalar += trans_err.norm(); // L2-Norm  trans_err.lpNorm<2>();
        trans_err_x += fabs(trans_err[0]);
        trans_err_y += fabs(trans_err[1]);
        trans_err_z += fabs(trans_err[2]);
        trans_err_xy += sqrt(trans_err[0] * trans_err[0] + trans_err[1] * trans_err[1]);
        submaps_itr++;
    }

    rot_err_scalar = rot_err_scalar / sub_maps_.size();
    trans_err_scalar = trans_err_scalar / sub_maps_.size();
    trans_err_x = trans_err_x / sub_maps_.size();
    trans_err_y = trans_err_y / sub_maps_.size();
    trans_err_z = trans_err_z / sub_maps_.size();
    trans_err_xy = trans_err_xy / sub_maps_.size();
    LOG(WARNING) << "*** *** SUBMAP MAE REPORT:\n"
                 << "Rotation Err (mean) is " << (rot_err_scalar * 180 / M_PI) << " degree , Translation Err (mean) is " << trans_err_scalar << " m.\n"
                 << "Translation Err (mean) x-axis " << trans_err_x << " y-axis " << trans_err_y << " z-axis " << trans_err_z << " xy-plane " << trans_err_xy;

    return true;
}

bool BackEndOptimization::FrameMae() const {
    double rot_err_scalar = 0;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> trans_err_vector_array;
    Eigen::Vector3d trans_err_vector_estimated = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rot_err_vector_array;
    Eigen::Vector3d rot_err_vector_estimated = Eigen::Vector3d::Zero();

    auto submaps_itr = sub_maps_.cbegin();
    int frame_num = 0;

    while (submaps_itr != sub_maps_.end()) {
        for (int i = 0; i < submaps_itr->second.frame_number; ++i) {
            Eigen::Quaterniond rot_err = submaps_itr->second.raw_data_group[i].raw_frame.pose.quat.conjugate() *
                                         submaps_itr->second.raw_data_group[i].raw_gnss.pose.quat;
            //            LOG(ERROR) << "qua " << rot_err.x() << " " << rot_err.y() << " " << rot_err.z() << " " << rot_err.w();
            //            LOG(ERROR) << rot_err.toRotationMatrix().eulerAngles(0, 1, 2);
            Eigen::Vector3d eulerAngle = rot_err.toRotationMatrix().eulerAngles(0, 1, 2);
            Eigen::Vector3d trans_err = submaps_itr->second.raw_data_group[i].raw_frame.pose.trans -
                                        submaps_itr->second.raw_data_group[i].raw_gnss.pose.trans;
            rot_err_scalar += fabs(getThetaFromQuaternion(rot_err));
            rot_err_vector_array.push_back(eulerAngle);
            rot_err_vector_estimated += eulerAngle;
            trans_err_vector_array.push_back(trans_err);
            trans_err_vector_estimated += trans_err;
            submaps_itr++;
            frame_num++;
        }
    }

    rot_err_scalar = rot_err_scalar / frame_num;
    trans_err_vector_estimated = trans_err_vector_estimated / frame_num;
    rot_err_vector_estimated = rot_err_vector_estimated / frame_num;

    double cov_x = 0;
    double cov_y = 0;
    double cov_z = 0;
    double cov_xy = 0;
    for (int i = 0; i < trans_err_vector_array.size(); ++i) {
        Eigen::Vector3d trans_err_vector_diff = trans_err_vector_array[i] - trans_err_vector_estimated;
        cov_x += trans_err_vector_diff[0] * trans_err_vector_diff[0];
        cov_y += trans_err_vector_diff[1] * trans_err_vector_diff[1];
        cov_z += trans_err_vector_diff[2] * trans_err_vector_diff[2];
        cov_xy += trans_err_vector_diff[0] * trans_err_vector_diff[0] + trans_err_vector_diff[1] * trans_err_vector_diff[1];
    }
    double cov_roll = 0;
    double cov_pitch = 0;
    double cov_yaw = 0;
    for (int i = 0; i < rot_err_vector_array.size(); ++i)
    {
        Eigen::Vector3d rot_err_vector_diff = rot_err_vector_array[i] - rot_err_vector_estimated;
        cov_roll += rot_err_vector_diff[0] * rot_err_vector_diff[0];
        cov_pitch += rot_err_vector_diff[1] * rot_err_vector_diff[1];
        cov_yaw += rot_err_vector_diff[2] * rot_err_vector_diff[2];
    }
    cov_x = sqrt(cov_x / trans_err_vector_array.size());
    cov_y = sqrt(cov_y / trans_err_vector_array.size());
    cov_z = sqrt(cov_z / trans_err_vector_array.size());
    cov_xy = sqrt(cov_xy / trans_err_vector_array.size());
    cov_roll = sqrt(cov_roll / rot_err_vector_array.size());
    cov_pitch = sqrt(cov_pitch / rot_err_vector_array.size());
    cov_yaw = sqrt(cov_yaw / rot_err_vector_array.size());

    LOG(WARNING) << "*** *** FRAME MAE:\n"
                 << "Rotation Err (mean) is " << (rot_err_scalar * 180 / M_PI) << " degree\n"
                 //<< "EulerAngle Err Expectation(deg) is roll=" << rot_err_vector_estimated[0] * 180 / M_PI << " pitch=" << rot_err_vector_estimated[1] * 180 / M_PI << " yaw=" << rot_err_vector_estimated[2] * 180 / M_PI << "\n"
                 //<< "EulerAngle Err std(deg) roll=" << cov_roll * 180 / M_PI << " pitch=" << cov_pitch * 180 / M_PI << " yaw=" << cov_yaw * 180 / M_PI << "\n"
                 << "Translation Err Expectation(m) is x=" << trans_err_vector_estimated[0] << " y=" << trans_err_vector_estimated[1] << " z=" << trans_err_vector_estimated[2] << "\n"
                 << "Translation Err std(m) x=" << cov_x << " y=" << cov_y << " z=" << cov_z << " xy=" << cov_xy;

    return true;
}

bool BackEndOptimization::CheckPGOResult(std::vector<submapid_error_t, Eigen::aligned_allocator<submapid_error_t> > &err_submaps,
                                         std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > &err_edges) const {
    err_submaps.clear();
    err_edges.clear();

    // Evaluate Edge Error
    calc_edge_error(sub_maps_, adjacent_edges_, err_edges);
    calc_edge_error(sub_maps_, inter_loops_, err_edges);
    calc_edge_error(sub_maps_, intra_loops_, err_edges);
    CHECK(err_edges.size() == (adjacent_edges_.size()+inter_loops_.size()+intra_loops_.size())) << "error edges size wrong";

    // Evaluate Submap Error
    for (auto submaps_itr = sub_maps_.cbegin(); submaps_itr != sub_maps_.cend(); ++submaps_itr) {
        submapid_error_t submapidError;
        // pose_error = T_pgo ^ -1 * T_gnss 这样统计的平移误差不在世界坐标系下， 在pgo坐标系下， 这样统计出来的x,y,z轴的平移误差无意义，应统计世界坐标系下的平移误差，参考FrameMAE()： pose_error平移=pgo平移-gnss平移，pose_error旋转=pgo旋转^-1 * gnss旋转
//        submapidError.pose_error.SetPose(submaps_itr->second.pose.GetMatrix().inverse() *
//                                         submaps_itr->second.raw_data_group[0].raw_gnss.pose.GetMatrix());
        submapidError.pose_error.trans = submaps_itr->second.pose.trans - submaps_itr->second.raw_data_group[0].raw_gnss.pose.trans;
        submapidError.pose_error.quat = submaps_itr->second.pose.quat.conjugate() * submaps_itr->second.raw_data_group[0].raw_gnss.pose.quat;
        submapidError.submap_id = submaps_itr->first;
        submapidError.theta = getThetaFromQuaternion(submapidError.pose_error.quat);
        err_submaps.push_back(submapidError);
    }
    CHECK(err_submaps.size() == sub_maps_.size()) << "err_submaps size should == sub_maps_ size";

    // Sort
#if 0
    std::sort(err_submaps.begin(), err_submaps.end(), greater_submapid_error_t);
    std::sort(err_edges.begin(), err_edges.end(), greater_edgeid_error_t);
#endif

    return true;
}

bool BackEndOptimization::WritePoseToFile()
{
    CHECK(!sub_maps_.empty()) << "Nothing to Write Out";
    UpdateFramePose();

    VectorOfSubmaps sub_maps_out;
    for (auto sub_map_itr = sub_maps_.cbegin(); sub_map_itr != sub_maps_.cend(); ++sub_map_itr)
    {
        Submap submap = sub_map_itr->second;
        submap.submap_id = sub_map_itr->first;
        sub_maps_out.push_back(submap);
    }
    CHECK(data_loader_.writeDeltaCalib(params_.pose_graph_output_folder, params_.pose_graph_output_calib_file, calib_ldr2gnss_dt_))
    << "Delta Calib is Written to " << params_.pose_graph_output_folder + "/" + params_.pose_graph_output_calib_file;
    data_loader_.writeSubmaps(params_.pose_graph_output_folder, params_.pose_graph_output_file, sub_maps_out);
}

bool BackEndOptimization::UpdateFramePose()
{
    for (auto submaps_itr = sub_maps_.begin(); submaps_itr != sub_maps_.end(); ++submaps_itr)
    {
        Pose3d pose_init_inverse = submaps_itr->second.raw_data_group[0].raw_frame.pose;
        pose_init_inverse.inverse();
        for (int i = 0; i < submaps_itr->second.frame_number; ++i)
        {
            //                Pose3d pose1 = submaps_itr->second.pose * pose_init_inverse
            //                               * submaps_itr->second.raw_data_group[i].raw_frame.pose;
            //                Pose3d pose2;
            //                pose2.SetPose(submaps_itr->second.pose.GetMatrix() * pose_init_inverse.GetMatrix() * submaps_itr->second.raw_data_group[i].raw_frame.pose.GetMatrix());
            //                LOG(FATAL) << "\npose1\n" << pose1 << "\npose2\n" << pose2;
            submaps_itr->second.raw_data_group[i].raw_frame.pose.copyFrom(
                    submaps_itr->second.pose * pose_init_inverse * submaps_itr->second.raw_data_group[i].raw_frame.pose);
        }
    }
    return true;
}

// Print Submaps which are too close to bounderies
void BackEndOptimization::PrintProblemSubmaps() const
{
    double threshold = 0.001;
    std::vector<submap_id_t> submap_ids;
    for (auto sub_map_itr = sub_maps_.cbegin(); sub_map_itr != sub_maps_.cend(); ++sub_map_itr)
    {
        // TODO raw_frame->raw_gnss after modify dataio
        double x_dis = sub_map_itr->second.pose.trans[0] - sub_map_itr->second.raw_data_group[0].raw_gnss.pose.trans[0];
        double y_dis = sub_map_itr->second.pose.trans[1] - sub_map_itr->second.raw_data_group[0].raw_gnss.pose.trans[1];
        //        LOG(WARNING) << sub_map_itr->first << " | x_dis is " << x_dis << " y_dis is " << y_dis;
        if (x_dis > params_.submap_upper_bound_x - threshold || x_dis < -(params_.submap_lower_bound_x - threshold) ||
            y_dis > params_.submap_upper_bound_y - threshold || y_dis < -(params_.submap_lower_bound_y + threshold))
        {
            submap_ids.push_back(sub_map_itr->first);
        }
    }
    LOG(ERROR) << "Bound Submaps Num is " << submap_ids.size();
    LOG(ERROR) << "Bound Submaps ID is: ";
    for (int i = 0; i < submap_ids.size(); ++i) {
        std::cout << submap_ids[i];
    }
    std::cout << std::endl;
}

void BackEndOptimization::EvaluateSelectedArea(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &road_bbxs,
                                               const std::map<int, int> &map_transaction_id_to_sequence) {
    // Distance Filter
    float xy_max = 40.0; // xy_max is the filter radius, points outside the circle would be deleted (unit:m)
    float z_min = -2.8;  // z_min is used to filter some noise underground points (unit:m)
    float z_max = 30.0;  // z_max is used to filter some noise unground points (unit:m)

    // Ground Filter (Segment Ground and Unground points)
    // gf_min_grid_num is the min point number in a grid. those grid whose point number < gf_min_grid_num would be ignored
    int gf_min_grid_num = 10;
    // gf_grid_resolution is the size of a grid (unit:m)
    float gf_grid_resolution = 0.75;
    // points whose [(z - min_z of the grid) > gf_max_grid_height_diff] would be regarded as unground points (unit:m)
    float gf_max_grid_height_diff = 0.15;
    // grids whose [(z_min - z_min of the grid's 8 neighbor grids) > gf_neighbor_height_diff would be regarded as unground grids (unit:m)
    float gf_neighbor_height_diff = 1.0;
    // points whose z is larger than gf_max_ground_height would be regarded as unground points whatever (unit:m)
    float gf_max_ground_height = 1.0;
    // gf_downsample_rate_nonground is the random downsample ratio of detected unground points [the downsample is for efficiency concerning]
    int gf_downsample_rate_nonground = 2;
    // only gf_downsample_grid_number_first points would be randomly selected as the ground points in a grid
    // This group of ground points are used for registration [target ground points (more point number)]
    int gf_downsample_grid_number_first = 5;
    // only gf_downsample_grid_number_second points would be randomly selected as the ground points in a grid
    // This group of ground points are used for registration [source ground points (less point number)]
    int gf_downsample_grid_number_second = 3;

    // Feature Points Detection
    // Search neighbor_search_K nearest neighbor for doing neighbor PCA
    // For neighbor PCA result, we define e1,e2,e3 (e1>e2>e3) are three eigen value and v1,v2,v3 are the correponding eigen vector
    // We call v1 the primary vector and v3 the normal vector
    // We define point linearty, planarity and curvature
    // linearty a_1d = (e1-e2)/e1 , planarity a_2d = (e2-e3)/e1 , curvature = e3/(e1+e2+e3)
    int neighbor_search_K = 8;
    // Those candidate edge points whose primary direction's z component < linear_vertical_cosine_min would be rejected
    float linear_vertical_cosine_min = 0.75;
    // Those candidate planar points whose normal direction's z component > planar_horizontal_cosine_max would be rejected
    float planar_horizontal_cosine_max = 0.4;
    // linearty threshold of edge feature points for registration [target edge points (more point number)]
    float neighbor_linear_thre_target = 0.6;
    // planarty threshold of planar feature points for registration [target planar points (more point number)]
    float neighbor_planar_thre_target = 0.5;
    // curvature threshold of sphere feature points for registration [target sphere points (more point number)]
    float neighbor_curvature_thre_target = 0.2;
    // linearty threshold of edge feature points for registration [source edge points (less point number)]
    float neighbor_linear_thre_source = 0.6;
    // planarty threshold of planar feature points for registration [source planar points (less point number)]
    float neighbor_planar_thre_source = 0.5;
    // curvature threshold of sphere feature points for registration [source sphere points (less point number)]
    float neighbor_curvature_thre_source = 0.2;
    // edge_point_source_appro_num points with larger linearity would be regarded as source edge points (less point number)
    int edge_point_source_appro_num = 500;
    // planar_point_source_appro_num points with larger planarity would be regarded as source planar points (less point number)
    int planar_point_source_appro_num = 500;
    // sphere_point_source_appro_num points with larger curvature would be regarded as source sphere points (less point number)
    int sphere_point_source_appro_num = 100;

    float vehicle_lidar_height = 1.8;


    // record frame_file_name and pose
    std::vector<std::string> frame_file_names;
    std::vector<Pose3d, Eigen::aligned_allocator<Pose3d> > gnss_pose;
    std::vector<Pose3d, Eigen::aligned_allocator<Pose3d> > pgo_pose;
    std::vector<int> transaction_ids;
    // find candidate pcd
    pcl::PointCloud<pcl::PointXY>::Ptr cp_cloud(new pcl::PointCloud<pcl::PointXY>());
    //Construct pose kd-tree
    auto submap_itr = sub_maps_.cbegin();
    while (submap_itr != sub_maps_.cend()) {
        const submap_id_t &submap_id = submap_itr->first;
        const Submap &submap = submap_itr->second;
        auto transaction_idx_itr = map_transaction_id_to_sequence.find(submap_id.transaction_id);
        CHECK(transaction_idx_itr != map_transaction_id_to_sequence.cend()) << "transaction id doesn't exist! transaction id is " << submap_id.transaction_id;
        std::string in_root_path = params_.transaction_lidar_root_path[transaction_idx_itr->second];
        for (unsigned int i = 0; i < submap.frame_number; i++) {
            pcl::PointXY cp;
            cp.x = submap.raw_data_group[i].raw_gnss.pose.trans(0);
            cp.y = submap.raw_data_group[i].raw_gnss.pose.trans(1);
            //LOG(INFO)<<"Cloud (x ,y) "<< cp.x<<"," <<cp.y;
            cp_cloud->push_back(cp);
            frame_file_names.push_back(in_root_path + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name);
            gnss_pose.push_back(submap.raw_data_group[i].raw_gnss.pose);
            pgo_pose.push_back(submap.raw_data_group[i].raw_frame.pose);
            transaction_ids.push_back(submap.submap_id.transaction_id);
        }
        submap_itr++;
    }

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(cp_cloud);

    //Set the searching points
    for (unsigned int i = 0; i < road_bbxs.size(); i++) {
        pcl::PointXY cp_search;
        float search_radius = road_bbxs[i][2];
        cp_search.x = road_bbxs[i][0];
        cp_search.y = road_bbxs[i][1];
        std::vector<int> pointIdx;
        std::vector<float> pointSquaredDistance;
//        std::set<int> select_area_transaction_id_num;

        //! For one aera
        if (kdtree.radiusSearch(cp_search, search_radius, pointIdx, pointSquaredDistance) > 0) {
            float z_min_pgo, z_max_pgo, z_min_gnss, z_max_gnss;
            z_min_gnss = FLT_MAX;
            z_min_pgo = FLT_MAX;
            z_max_gnss = -FLT_MAX;
            z_max_pgo = -FLT_MAX;
            LOG(INFO) << ">>> Evaluate Bbox[ " << i << " ], " << pointIdx.size() << " pcds will be Evaluated.";
            VectorOfRawDatas raw_datas;
            raw_datas.resize(pointIdx.size());
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_ground_cld_pgo(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_ground_cld_gnss(new pcl::PointCloud<pcl::PointXYZINormal>);
            for (unsigned int j = 0; j < pointIdx.size(); j++) {
                // determine select_area_transaction_id_num
//                select_area_transaction_id_num.insert(transaction_ids[pointIdx[j]]);
                //! PrePare Data
                std::shared_ptr<PointCloud> pointcloud = std::make_shared<PointCloud>();
                std::shared_ptr<PointCloud> world_pointcloud_pgo = std::make_shared<PointCloud>();
                std::shared_ptr<PointCloud> world_pointcloud_gnss = std::make_shared<PointCloud>();
                data_loader_.readPcdFile(frame_file_names[pointIdx[j]],
                                         pointcloud, 1);
                raw_datas[j].raw_frame.cld_lidar_ptr = pointcloud;
                raw_datas[j].raw_frame.pose.copyFrom(pgo_pose[pointIdx[j]]);
                raw_datas[j].raw_gnss.pose.copyFrom(gnss_pose[pointIdx[j]]);
                raw_datas[j].raw_frame.pcd_file_name = frame_file_names[pointIdx[j]];

                //! Evaluate Ground
                reg_.transformPointCloud(pointcloud, world_pointcloud_pgo, pgo_pose[pointIdx[j]]);
                reg_.transformPointCloud(pointcloud, world_pointcloud_gnss, gnss_pose[pointIdx[j]]);
                //Ground Filter
                std::vector<unsigned int> unground_index;
                std::vector<unsigned int> ground_index;
                std::vector<unsigned int> ground_down_index;
                filter_.FastGroundFilter(pointcloud, ground_index,
                                         ground_down_index, unground_index,
                                         gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff,
                                         gf_neighbor_height_diff, -1.4,
                                         5, 3,
                                         gf_downsample_rate_nonground);

                for (unsigned int kk = 0; kk < ground_index.size(); kk++) {
                    const auto idx = ground_index[kk];
                    if ( (world_pointcloud_pgo->points[idx].x - cp_search.x) * (world_pointcloud_pgo->points[idx].x - cp_search.x) +
                         (world_pointcloud_pgo->points[idx].y - cp_search.y) * (world_pointcloud_pgo->points[idx].y - cp_search.y) <
                         search_radius * search_radius) {
                        pcl::PointXYZINormal point_temp;
                        point_temp.x = world_pointcloud_pgo->points[idx].x;
                        point_temp.y = world_pointcloud_pgo->points[idx].y;
                        point_temp.z = world_pointcloud_pgo->points[idx].z;
                        world_ground_cld_pgo->points.push_back(point_temp);
                    }
                    if ( (world_pointcloud_gnss->points[idx].x - cp_search.x) * (world_pointcloud_gnss->points[idx].x - cp_search.x) +
                         (world_pointcloud_gnss->points[idx].y - cp_search.y) * (world_pointcloud_gnss->points[idx].y - cp_search.y) <
                         search_radius * search_radius) {
                        pcl::PointXYZINormal point_temp;
                        point_temp.x = world_pointcloud_gnss->points[idx].x;
                        point_temp.y = world_pointcloud_gnss->points[idx].y;
                        point_temp.z = world_pointcloud_gnss->points[idx].z;
                        world_ground_cld_gnss->points.push_back(point_temp);
                    }
                }
                pointcloud = std::make_shared<PointCloud>();
            }
            // Use PCA find Ground Normal, then evaluate Ground Thickness
            pcaFeature feature_pgo;
            pcaFeature feature_gnss;
            std::vector<int> search_indices;
            search_indices.resize(world_ground_cld_pgo->points.size());
            for (int i = 0; i < world_ground_cld_pgo->points.size(); ++i) {
                search_indices[i] = i;
            }
            pca_.CalculatePcaFeature(world_ground_cld_pgo, search_indices, feature_pgo);
            // calc coordinate along normal direction
            std::vector<float> ground_height_pgo;
            ground_height_pgo.resize(world_ground_cld_pgo->points.size());
            float ground_height_estimated_pgo = 0;
            for (int i = 0; i < world_ground_cld_pgo->points.size(); ++i) {
                Eigen::Vector3f origin_coor;
                origin_coor << world_ground_cld_pgo->points[i].x, world_ground_cld_pgo->points[i].y, world_ground_cld_pgo->points[i].z;
                ground_height_pgo[i] = feature_pgo.vectors.normalDirection.transpose() * origin_coor;
                ground_height_estimated_pgo += ground_height_pgo[i];
            }
            ground_height_estimated_pgo /= ground_height_pgo.size();
            // calc ground std along normal direction
            float ground_height_std_pgo = 0;
            for (int i = 0; i < ground_height_pgo.size(); ++i) {
                ground_height_std_pgo += (ground_height_pgo[i] - ground_height_estimated_pgo) * (ground_height_pgo[i] - ground_height_estimated_pgo);
            }
            ground_height_std_pgo = sqrt(ground_height_std_pgo / ground_height_pgo.size());

            search_indices.resize(world_ground_cld_gnss->points.size());
            for (int i = 0; i < world_ground_cld_gnss->points.size(); ++i) {
                search_indices[i] = i;
            }
            pca_.CalculatePcaFeature(world_ground_cld_gnss, search_indices, feature_gnss);
            // calc coordinate along normal direction
            std::vector<float> ground_height_gnss;
            ground_height_gnss.resize(world_ground_cld_gnss->points.size());
            float ground_height_estimated_gnss = 0;
            for (int i = 0; i < world_ground_cld_gnss->points.size(); ++i) {
                Eigen::Vector3f origin_coor;
                origin_coor << world_ground_cld_gnss->points[i].x, world_ground_cld_gnss->points[i].y, world_ground_cld_gnss->points[i].z;
                ground_height_gnss[i] = feature_gnss.vectors.normalDirection.transpose() * origin_coor;
                ground_height_estimated_gnss += ground_height_gnss[i];
            }
            ground_height_estimated_gnss /= ground_height_gnss.size();
            // calc ground std along normal direction
            float ground_height_std_gnss = 0;
            for (int i = 0; i < ground_height_gnss.size(); ++i) {
                ground_height_std_gnss += (ground_height_gnss[i] - ground_height_estimated_gnss) * (ground_height_gnss[i] - ground_height_estimated_gnss);
            }
            ground_height_std_gnss = sqrt(ground_height_std_gnss / ground_height_gnss.size());

            //For display
#if 1
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ground_pgo(new pcl::visualization::PCLVisualizer("Ground PGO"));
            viewer_ground_pgo->setBackgroundColor(0, 0, 0);
            viewer_ground_pgo->addPointCloud<pcl::PointXYZINormal>(world_ground_cld_pgo, "pgo");
            cout << "Click X(close) to continue..." << endl;
            while (!viewer_ground_pgo->wasStopped())
            {
                viewer_ground_pgo->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ground_gnss(new pcl::visualization::PCLVisualizer("Ground GNSS"));
            viewer_ground_gnss->setBackgroundColor(0, 0, 0);
            viewer_ground_gnss->addPointCloud<pcl::PointXYZINormal>(world_ground_cld_gnss, "gnss");
            cout << "Click X(close) to continue..." << endl;
            while (!viewer_ground_gnss->wasStopped())
            {
                viewer_ground_gnss->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
#endif
            // Ground Evaluate Done, Free Memory
            world_ground_cld_pgo = nullptr;
            world_ground_cld_gnss = nullptr;

            //! Evaluate Consistency
            // Regitstration INIT
            std::vector<Pose3d, Eigen::aligned_allocator<Pose3d> > pose_errors_pgo;
            std::vector<Pose3d, Eigen::aligned_allocator<Pose3d> > pose_errors_gnss;
            std::vector<Pose3d, Eigen::aligned_allocator<Pose3d> > pose_display_pgo;
            std::vector<Pose3d, Eigen::aligned_allocator<Pose3d> > pose_display_gnss;
            pose_errors_pgo.resize(raw_datas.size());
            pose_errors_gnss.resize(raw_datas.size());
            pose_display_pgo.resize(raw_datas.size());
            pose_display_gnss.resize(raw_datas.size());
            for (int i = 0; i < raw_datas.size(); ++i) {
                int code_pgo = 0;
                int code_gnss = 0;
                // Reg
                while(xy_max <= 60 || gf_min_grid_num > 3) {
                    if (i == 0) {
                        Pose3d pose;
                        pose_display_pgo[i].copyFrom(pose);
                        pose_display_gnss[i].copyFrom(pose);
                        code_pgo = 1;
                        code_gnss = 1;
                        //! Preprocess Part
                        std::vector<unsigned int> cld_unground_index;
                        //Ground Filter
                        filter_.FastGroundFilter(raw_datas[i].raw_frame.cld_lidar_ptr, raw_datas[i].raw_frame.ground_index,
                                                 raw_datas[i].raw_frame.ground_down_index, cld_unground_index,
                                                 gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff,
                                                 gf_neighbor_height_diff, gf_max_ground_height,
                                                 gf_downsample_grid_number_first, gf_downsample_grid_number_second,
                                                 gf_downsample_rate_nonground);
                        //Detected Feature Points
                        filter_.RoughClassify(raw_datas[i].raw_frame.cld_lidar_ptr,
                                              raw_datas[i].raw_frame.edge_down_index, raw_datas[i].raw_frame.edge_index,
                                              raw_datas[i].raw_frame.planar_down_index, raw_datas[i].raw_frame.planar_index,
                                              raw_datas[i].raw_frame.sphere_down_index, raw_datas[i].raw_frame.sphere_index,
                                              cld_unground_index, neighbor_search_K,
                                              neighbor_linear_thre_target, neighbor_planar_thre_target, neighbor_curvature_thre_target,
                                              neighbor_linear_thre_source, neighbor_planar_thre_source, neighbor_curvature_thre_source,
                                              linear_vertical_cosine_min, planar_horizontal_cosine_max,
                                              edge_point_source_appro_num, planar_point_source_appro_num, sphere_point_source_appro_num);
                        break;
                    } else {
                        // backup point cloud
                        PointCloud origin_cld = *raw_datas[i].raw_frame.cld_lidar_ptr;
                        //! Preprocess Part
                        std::shared_ptr<PointCloud> pointcloud_lidar_filtered(new PointCloud);
                        std::vector<unsigned int> cld_unground_index;
//                        raw_datas[i].noise_gnss.pose.copyFrom(raw_datas[i].raw_frame.pose);
                        //Dis Filter, ROI filter
                        filter_.DisFilter(raw_datas[i].raw_frame.cld_lidar_ptr,
                                          pointcloud_lidar_filtered, xy_max, z_min, z_max);
                        raw_datas[i].raw_frame.cld_lidar_ptr->points.swap(pointcloud_lidar_filtered->points);
                        //Ground Filter
                        filter_.FastGroundFilter(raw_datas[i].raw_frame.cld_lidar_ptr, raw_datas[i].raw_frame.ground_index,
                                                 raw_datas[i].raw_frame.ground_down_index, cld_unground_index,
                                                 gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff,
                                                 gf_neighbor_height_diff, gf_max_ground_height,
                                                 gf_downsample_grid_number_first, gf_downsample_grid_number_second,
                                                 gf_downsample_rate_nonground);
                        //Detected Feature Points
                        filter_.RoughClassify(raw_datas[i].raw_frame.cld_lidar_ptr,
                                              raw_datas[i].raw_frame.edge_down_index, raw_datas[i].raw_frame.edge_index,
                                              raw_datas[i].raw_frame.planar_down_index, raw_datas[i].raw_frame.planar_index,
                                              raw_datas[i].raw_frame.sphere_down_index, raw_datas[i].raw_frame.sphere_index,
                                              cld_unground_index, neighbor_search_K,
                                              neighbor_linear_thre_target, neighbor_planar_thre_target, neighbor_curvature_thre_target,
                                              neighbor_linear_thre_source, neighbor_planar_thre_source, neighbor_curvature_thre_source,
                                              linear_vertical_cosine_min, planar_horizontal_cosine_max,
                                              edge_point_source_appro_num, planar_point_source_appro_num, sphere_point_source_appro_num);
                        //! Reg Part
                        // PGO Evaluation
                        LOG(INFO) << "---------------PGO:Registration [" << raw_datas[i].raw_frame.pcd_file_name
                                  << "] to ["
                                  << raw_datas[0].raw_frame.pcd_file_name << "]--------------";
                        raw_datas[0].noise_gnss.pose.copyFrom(raw_datas[0].raw_frame.pose);
                        raw_datas[i].noise_gnss.pose.copyFrom(raw_datas[i].raw_frame.pose);
                        Pose3d error_pgo, error_gnss;
                        code_pgo = reg_.PairwiseReg(raw_datas[0], raw_datas[i], error_pgo, GNSSINSPoseDiff);
                        pose_display_pgo[i].copyFrom(error_pgo);
                        error_pgo.SetPose(error_pgo.GetMatrix() * raw_datas[i].raw_frame.pose.GetMatrix().inverse() * raw_datas[0].raw_frame.pose.GetMatrix());
                        pose_errors_pgo[i].copyFrom(error_pgo);

                        // GNSS Evaluation
                        LOG(INFO) << "---------------GNSS:Registration [" << raw_datas[i].raw_frame.pcd_file_name
                                  << "] to ["
                                  << raw_datas[0].raw_frame.pcd_file_name << "]--------------";
                        raw_datas[0].noise_gnss.pose.copyFrom(raw_datas[0].raw_gnss.pose);
                        raw_datas[i].noise_gnss.pose.copyFrom(raw_datas[i].raw_gnss.pose);
                        code_gnss = reg_.PairwiseReg(raw_datas[0], raw_datas[i], error_gnss, GNSSINSPoseDiff);
                        pose_display_gnss[i].copyFrom(error_gnss);
                        error_gnss.SetPose(error_gnss.GetMatrix() * raw_datas[i].raw_gnss.pose.GetMatrix().inverse() * raw_datas[0].raw_gnss.pose.GetMatrix());
                        pose_errors_gnss[i].copyFrom(error_gnss);

                        //! Check Reg Result
                        if (code_pgo == 1 && code_gnss == 1) {
                            // Reg Success
                            break;
                        } else if (code_pgo == -1 || code_pgo == -2 || code_pgo == -3 ||
                                   code_gnss == -1 || code_gnss == -2 || code_gnss == -3) {
                            // Reg Failed, fix parameters and try it again
                            // Recover Point Cloud
                            LOG(ERROR) << "BEFORE cld size is " << raw_datas[i].raw_frame.cld_lidar_ptr->points.size();
                            *raw_datas[i].raw_frame.cld_lidar_ptr = origin_cld;
                            LOG(ERROR) << "AFTER cld size is " << raw_datas[i].raw_frame.cld_lidar_ptr->points.size();
                            // Expand Dis Filter
                            xy_max += 20;
                            gf_min_grid_num -= 6;
                            if (gf_min_grid_num <= 3)
                                gf_min_grid_num = 3;

                            continue;
                        } else {
                            // code == -4  Registration Failed, Nothing we can do.
                            LOG(FATAL) << "Fix Registration Method Between Frames";
                            break;
                        }
                    }
                }
                //! Reset Filter Param
                xy_max = 40.0; // xy_max is the filter radius, points outside the circle would be deleted (unit:m)
                gf_min_grid_num = 10;

                //! Handle Reg Failed
                if (code_pgo != 1 || code_gnss != 1) {
                    // TODO
                    //For display
#if 1
                    if (code_pgo != 1) {
                        LOG(ERROR) << "*** PGO *** Reg Failed! [ " << i << " ] error code is " << code_pgo;
                        //transform for display
                        Pose3d init_transform_pose;
                        init_transform_pose.SetPose(raw_datas[0].raw_frame.pose.GetMatrix().inverse() * raw_datas[i].raw_frame.pose.GetMatrix());
                        std::shared_ptr<PointCloud> frame2_pointcloud_ptr(new PointCloud);
                        std::shared_ptr<PointCloud> frame2_pointcloud_after_reg_ptr(new PointCloud);

                        reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr, init_transform_pose);
                        reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr, pose_display_pgo[i]);

                        MapViewer<PointType> viewer;
                        viewer.Dispaly2CloudsCompare(raw_datas[0].raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr,
                                                      raw_datas[0].raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr,
                                                      "Left:before registration, Right:after registration", 1);
                        frame2_pointcloud_ptr = std::make_shared<point_cloud_t<PointType>>();
                        frame2_pointcloud_after_reg_ptr = std::make_shared<point_cloud_t<PointType>>();
                    }
                    if (code_gnss != 1) {
                        LOG(ERROR) << "*** GNSS *** Reg Failed! [ " << i << " ] error_code is " << code_gnss;
                        //transform for display
                        Pose3d init_transform_pose;
                        init_transform_pose.SetPose(raw_datas[0].raw_frame.pose.GetMatrix().inverse() * raw_datas[i].raw_frame.pose.GetMatrix());
                        std::shared_ptr<PointCloud> frame2_pointcloud_ptr(new PointCloud);
                        std::shared_ptr<PointCloud> frame2_pointcloud_after_reg_ptr(new PointCloud);

                        reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr, init_transform_pose);
                        reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr, pose_display_pgo[i]);

                        MapViewer<PointType> viewer;
                        viewer.Dispaly2CloudsCompare(raw_datas[0].raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr,
                                                     raw_datas[0].raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr,
                                                     "Left:before registration, Right:after registration", 1);
                        frame2_pointcloud_ptr = std::make_shared<point_cloud_t<PointType>>();
                        frame2_pointcloud_after_reg_ptr = std::make_shared<point_cloud_t<PointType>>();
                    }
#endif
                }
            }
            //Evaluate the accuracy
            // Registration
            Eigen::Vector3d gnss_pose_trans_estimated = Eigen::Vector3d::Zero();
            Eigen::Vector3d pgo_pose_trans_estimated = Eigen::Vector3d::Zero();
            double gnss_pose_rot_estimated = 0;
            double pgo_pose_rot_estimated = 0;
            CHECK(pose_errors_pgo.size() == pose_errors_gnss.size()) << "size must be the same";
            for (int i = 0; i < pose_errors_pgo.size(); ++i) {
                gnss_pose_trans_estimated[0] += fabs(pose_errors_gnss[i].trans[0]);
                gnss_pose_trans_estimated[1] += fabs(pose_errors_gnss[i].trans[1]);
                gnss_pose_trans_estimated[2] += fabs(pose_errors_gnss[i].trans[2]);
                pgo_pose_trans_estimated[0] += fabs(pose_errors_pgo[i].trans[0]);
                pgo_pose_trans_estimated[1] += fabs(pose_errors_pgo[i].trans[1]);
                pgo_pose_trans_estimated[2] += fabs(pose_errors_pgo[i].trans[2]);
                gnss_pose_rot_estimated += fabs(getThetaFromQuaternion(pose_errors_gnss[i].quat));
                pgo_pose_rot_estimated += fabs(getThetaFromQuaternion(pose_errors_pgo[i].quat));
            }
            gnss_pose_trans_estimated /= pose_errors_pgo.size();
            pgo_pose_trans_estimated /= pose_errors_pgo.size();
            gnss_pose_rot_estimated /= pose_errors_pgo.size();
            pgo_pose_rot_estimated /= pose_errors_pgo.size();

            // Report
            LOG(INFO) << "*** *** Consistency Report [ " << i << " ] *** \n" <<
                      " >>> Bbox[ " << i << " ], " << pointIdx.size() << " pcds are Evaluated.\n"
                      " >>> Registration: \n" <<
                      "GNSS  std(m) x=" << gnss_pose_trans_estimated[0] << " y=" << gnss_pose_trans_estimated[1] << " z=" << gnss_pose_trans_estimated[2]
                      << " rotation(deg)=" << gnss_pose_rot_estimated * 180 / M_PI << "\n" <<
                      "PGO   std(m) x=" << pgo_pose_trans_estimated[0] << " y=" << pgo_pose_trans_estimated[1] << " z=" << pgo_pose_trans_estimated[2]
                      << " rotation(deg)=" << pgo_pose_rot_estimated * 180 / M_PI << "\n" <<
                      " >>> Ground: \n" <<
                      "GNSS Ground Thickness std(m)=" << ground_height_std_gnss << "\n" <<
                      "PGO  Ground Thickness std(m)=" << ground_height_std_pgo;

            // VIS
#if 1
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> pgo_gt_clds;
//            std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> gnss_gt_clds;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> pgo_clds;
            std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> gnss_clds;
        for (int i = 0; i < raw_datas.size(); ++i) {
            if (i == 0) {
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gt_pgo(new pcl::PointCloud<pcl::PointXYZINormal>);
//                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gt_gnss(new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_pgo(new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gnss(new pcl::PointCloud<pcl::PointXYZINormal>);
                reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, cld_pgo, raw_datas[0].raw_frame.pose);
                pgo_clds.push_back(cld_pgo);
                reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, cld_gnss, raw_datas[0].raw_gnss.pose);
                gnss_clds.push_back(cld_gnss);
                reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, cld_gt_pgo, raw_datas[0].raw_frame.pose);
                pgo_gt_clds.push_back(cld_gt_pgo);
            } else {
                if ((pose_display_gnss[i].trans - pose_display_pgo[i].trans).norm() > 5e-3 ||
                getThetaFromQuaternion(pose_display_pgo[i].quat.conjugate() * pose_display_gnss[i].quat) > 2e-3) {
                    LOG(WARNING) << "*** Transform PCD [ " << i << "th. " << raw_datas[i].raw_frame.pcd_file_name << " ] ***";
                    LOG(WARNING) << "reg diff between pgo and gnss\n" << "Trans is\n" << pose_display_gnss[i].trans - pose_display_pgo[i].trans <<
                                 "\nRot is " << getThetaFromQuaternion(pose_display_pgo[i].quat.conjugate() * pose_display_gnss[i].quat);
                }
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gt_pgo(new pcl::PointCloud<pcl::PointXYZINormal>);
//                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gt_gnss(new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_pgo(new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gnss(new pcl::PointCloud<pcl::PointXYZINormal>);
                reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, cld_pgo, raw_datas[i].raw_frame.pose);
                pgo_clds.push_back(cld_pgo);
                reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, cld_gnss, raw_datas[i].raw_gnss.pose);
                gnss_clds.push_back(cld_gnss);
                reg_.transformPointCloud(raw_datas[i].raw_frame.cld_lidar_ptr, cld_gt_pgo, raw_datas[0].raw_frame.pose * pose_display_pgo[i]);
                pgo_gt_clds.push_back(cld_gt_pgo);
            }
        }
        map_pose::MapViewer<map_pose::PointType> viewer;
        std::string reg_display_name = std::to_string(i) + "th select_area | Left:PGO Middle:GNSS Right:GroundTruth";
        viewer.DisplayNClouds(pgo_clds, gnss_clds, pgo_gt_clds, reg_display_name, map_pose::FRAME, 1);
#endif

        } else {
            LOG(FATAL) << "Wrong! Couldn't find any Submap in Bbox num=" << i << " x=" << cp_search.x << " y=" << cp_search.y;
        }
    }
}

#if 1
void BackEndOptimization::RoadThickMapGeneration(int begin_id, int num) const
{
    float max_z_diff_thre = 0.8;
    float vehicle_lidar_height = 1.8;

    int road_map_row_num = 12000;
    int road_map_col_num = 12000;

    std::vector<std::vector<road_pixel_t>> road_map;
    road_map.resize(road_map_row_num, std::vector<road_pixel_t>(road_map_col_num, road_pixel_t()));

    std::shared_ptr<PointCloud> pointcloud = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> dis_filter_pointcloud = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> downsample_pointcloud = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> world_pointcloud_pgo = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> world_pointcloud_gnss = std::make_shared<PointCloud>();

    auto submap_itr = sub_maps_.cbegin();
    for (int i = 0; i < begin_id; ++i)
    {
        submap_itr++;
    }
    bounds_t interested_box;
    interested_box.min_x = -2;
    interested_box.max_x = 2;
    interested_box.min_y = -7;
    interested_box.max_y = 7;
    interested_box.min_z = -0.3 - vehicle_lidar_height;
    interested_box.max_z = 0.3 - vehicle_lidar_height;


    int road_map_x_shift = submap_itr->second.raw_data_group[0].raw_gnss.pose.trans(0) - road_map_row_num / 2;
    int road_map_y_shift = submap_itr->second.raw_data_group[0].raw_gnss.pose.trans(1) + road_map_row_num / 2;


    int k = 0;
    while (k < num)
    {
        const submap_id_t &submap_id = submap_itr->first;
        const Submap &submap = submap_itr->second;
        std::string in_root_path = params_.transaction_lidar_root_path[submap_id.transaction_id];

        for (int i = 0; i < submap.frame_number; ++i)
        {
            data_loader_.readPcdFile(in_root_path + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name,
                                     pointcloud, 1);

            filter_.BoxFilter(pointcloud, dis_filter_pointcloud, interested_box);
            filter_.RandomDownsample(dis_filter_pointcloud, downsample_pointcloud, 15, 15, 100.0);

            reg_.transformPointCloud(downsample_pointcloud, world_pointcloud_pgo, submap.raw_data_group[i].raw_frame.pose);
            reg_.transformPointCloud(downsample_pointcloud, world_pointcloud_gnss, submap.raw_data_group[i].raw_gnss.pose);

            int temp_col, temp_row;
            for (int j = 0; j < downsample_pointcloud->points.size(); j++)
            {
                //PGO update
                temp_col = (int)(world_pointcloud_pgo->points[j].x) - road_map_x_shift;
                temp_row = (int)(road_map_y_shift - world_pointcloud_pgo->points[j].y);
                if (world_pointcloud_pgo->points[j].z > road_map[temp_row][temp_col].max_z_pgo)
                    road_map[temp_row][temp_col].max_z_pgo = world_pointcloud_pgo->points[j].z;
                if (world_pointcloud_pgo->points[j].z < road_map[temp_row][temp_col].min_z_pgo)
                    road_map[temp_row][temp_col].min_z_pgo = world_pointcloud_pgo->points[j].z;
                if (!road_map[temp_row][temp_col].is_covered_pgo)
                {
                    road_map[temp_row][temp_col].is_covered_pgo = true;
                    LOG(INFO) << "New pixel (PGO): [" << temp_row << " , " << temp_col << "]";
                }

                //GNSS update
                temp_col = (int)(world_pointcloud_gnss->points[j].x) - road_map_x_shift;
                temp_row = (int)(road_map_y_shift - world_pointcloud_gnss->points[j].y);
                if (world_pointcloud_gnss->points[j].z > road_map[temp_row][temp_col].max_z_gnss)
                    road_map[temp_row][temp_col].max_z_gnss = world_pointcloud_gnss->points[j].z;
                if (world_pointcloud_gnss->points[j].z < road_map[temp_row][temp_col].min_z_gnss)
                    road_map[temp_row][temp_col].min_z_gnss = world_pointcloud_gnss->points[j].z;

                if (!road_map[temp_row][temp_col].is_covered_gnss)
                {
                    road_map[temp_row][temp_col].is_covered_gnss = true;
                    LOG(INFO) << "New pixel (GNSS): [" << temp_row << " , " << temp_col << "]";
                }
            }

            pointcloud = std::make_shared<PointCloud>();
            dis_filter_pointcloud = std::make_shared<PointCloud>();
            downsample_pointcloud = std::make_shared<PointCloud>();
        }

        submap_itr++;
        k++;
    }

    double gnss_road_total_z_dif = 0;
    double pgo_road_total_z_dif = 0;
    int cover_count_gnss = 0;
    int cover_count_pgo = 0;

    cv::Mat road_thick_map_gnss, road_thick_map_pgo;
    road_thick_map_gnss.create(road_map_row_num, road_map_col_num, CV_8UC1);
    road_thick_map_pgo.create(road_map_row_num, road_map_col_num, CV_8UC1); //Switch the color scale later

    //Get the result.
    for (int i = 0; i < road_map_row_num; i++)
    {
        for (int j = 0; j < road_map_col_num; j++)
        {

            if (road_map[i][j].is_covered_pgo)
            {
                pgo_road_total_z_dif += (road_map[i][j].max_z_pgo - road_map[i][j].min_z_pgo);
                road_thick_map_pgo.at<uchar>(i, j) = min_(255 * (road_map[i][j].max_z_pgo - road_map[i][j].min_z_pgo) / max_z_diff_thre, 255);
                cover_count_pgo++;
            }
            else
            {
                road_thick_map_pgo.at<uchar>(i, j) = 0;
            }

            if (road_map[i][j].is_covered_gnss)
            {
                gnss_road_total_z_dif += (road_map[i][j].max_z_gnss - road_map[i][j].min_z_gnss);
                road_thick_map_gnss.at<uchar>(i, j) = min_(255 * (road_map[i][j].max_z_gnss - road_map[i][j].min_z_gnss) / max_z_diff_thre, 255);
                cover_count_gnss++;
            }
            else
            {
                road_thick_map_gnss.at<uchar>(i, j) = 0;
            }
        }
    }
    LOG(INFO) << "Gnss total cover number is " << cover_count_gnss;
    LOG(INFO) << "PGO total cover number is " << cover_count_pgo;

    double gnss_road_mean_z_dif = gnss_road_total_z_dif / cover_count_gnss;
    double pgo_road_mean_z_dif = pgo_road_total_z_dif / cover_count_pgo;

    LOG(WARNING) << "Mean Raod thickness [ PGO: " << pgo_road_mean_z_dif << " m , GNSS: " << gnss_road_mean_z_dif << " m ]";

    cv::Mat road_thick_map_pgo_color, road_thick_map_gnss_color;
    cv::applyColorMap(road_thick_map_pgo, road_thick_map_pgo_color, cv::COLORMAP_JET);
    cv::applyColorMap(road_thick_map_gnss, road_thick_map_gnss_color, cv::COLORMAP_JET);

    cv::imwrite("1_PGO_Road_thick_map.jpg", road_thick_map_pgo_color);
    cv::imwrite("2_GNSS_Road_thick_map.jpg", road_thick_map_gnss_color);
}
#endif

void BackEndOptimization::TransformPointCloudForCloudCompare(int begin_id, int num, const bool GNSS, const bool PGO,
                                                             pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_pgo_out_merge,
                                                             pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_gnss_out_merge) const
{

    //float xy_max = 40.0, z_min = -4.0, z_max = 30.0, leaf_size = 0.25;
    float xy_max = 50.0, z_min = -4.0, z_max = 30.0, leaf_size = 0.25;
    std::shared_ptr<PointCloud> pointcloud = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> filtered_ptr = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> downsample_ptr = std::make_shared<PointCloud>();

    LOG(WARNING) << "Transform point cloud (PGO) should be done after Optimization!";
    auto submap_itr = sub_maps_.cbegin();
    for (int i = 0; i < begin_id; ++i)
    {
        submap_itr++;
    }

    int k = 0;
    while (k < num)
    {
        const submap_id_t &submap_id = submap_itr->first;
        const Submap &submap = submap_itr->second;
        std::string in_root_path = params_.transaction_lidar_root_path[submap_id.transaction_id];
        std::string out_pgo_root_path = params_.transaction_lidar_root_path[submap_id.transaction_id] + "_PGO";
        std::string out_gnss_root_path = params_.transaction_lidar_root_path[submap_id.transaction_id] + "_GNSS";
        data_loader_.checkDir(out_pgo_root_path);
        data_loader_.checkDir(out_gnss_root_path);
        for (int i = 0; i < submap.frame_number; ++i)
        {
            pointcloud->points.clear();
            filtered_ptr->points.clear();
            downsample_ptr->points.clear();
            data_loader_.readPcdFile(in_root_path + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name,
                                     pointcloud, 1);
            filter_.DisFilter(pointcloud, filtered_ptr, xy_max, z_min, z_max);
#if 1
            filter_.VoxelDownsample(filtered_ptr, downsample_ptr, leaf_size);
#else
            downsample_ptr = filtered_ptr;
#endif

#if 1
            if (PGO)
            {
                std::shared_ptr<PointCloud> transformed_ptr = std::make_shared<PointCloud>();
                reg_.transformPointCloud(downsample_ptr, transformed_ptr, submap.raw_data_group[i].raw_frame.pose);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_out(new pcl::PointCloud<pcl::PointXYZINormal>);
                data_loader_.OurPoint2PclPoint(transformed_ptr, cld_out);
                cld_pgo_out_merge->points.insert(cld_pgo_out_merge->points.end(), cld_out->points.begin(), cld_out->points.end());
//                data_loader_.writePcdFile(out_pgo_root_path + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name,cld_out);
            }
            if (GNSS)
            {
                std::shared_ptr<PointCloud> transformed_ptr = std::make_shared<PointCloud>();
                reg_.transformPointCloud(downsample_ptr, transformed_ptr, submap.raw_data_group[i].raw_gnss.pose);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_out(new pcl::PointCloud<pcl::PointXYZINormal>);
                data_loader_.OurPoint2PclPoint(transformed_ptr, cld_out);
                cld_gnss_out_merge->points.insert(cld_gnss_out_merge->points.end(), cld_out->points.begin(), cld_out->points.end());
//                data_loader_.writePcdFile(out_gnss_root_path + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name, cld_out);
            }
#endif

#if 0
            std::vector<std::shared_ptr<point_cloud_t<PointType> > > landmarks;
            int intensity_threshold = 200;
            double high_threshold = -1;
            int min_cluster_size = 40;
            int max_cluster_size = 10000;
            double cluster_tolerance = 0.4;
            filter_.LandmarkClassify(downsample_ptr, landmarks, intensity_threshold, high_threshold, min_cluster_size, max_cluster_size, cluster_tolerance);
#endif
        }
        submap_itr++;
        k++; // num is submap number
    }
    LOG(WARNING) << "Transform point cloud End!";
}

bool BackEndOptimization::LoadPoseGraphFromFile(MapOfSubMaps *submaps,
                                                VectorOfEdges *edges) const
{
    std::string input = params_.pose_graph_output_folder + "/" + params_.pose_graph_input_file;
    CHECK(input != "") << "Need to specify the filename to read.";

    if (data_loader_.LoadPoseGraphFromFile(input, *submaps, *edges))
        return true;
    else
        return false;
}

bool BackEndOptimization::OptimizeRotAndTrans()
{
    pgo_type_ = ROT_TRANS;
    BuildProblem();
    SolveProblem();
}

bool BackEndOptimization::OptimizeRotAndTransAndGnss()
{
    pgo_type_ = ROT_TRANS_GNSS;
    BuildProblem();
    SolveProblem();
}

bool BackEndOptimization::OptimizeRotAndTransAndGnssAndCalib()
{
    pgo_type_ = ROT_TRANS_GNSS_CALIB;
    BuildProblem();
    SolveProblem();
}

bool BackEndOptimization::OptimizeRot()
{
    pgo_type_ = ROT;
    BuildProblem();
    SolveProblem();
}

bool BackEndOptimization::OptimizeTrans()
{
    pgo_type_ = TRANS;
    BuildProblem();
    SolveProblem();
}

bool BackEndOptimization::Optimize()
{
    // Load Vertexes and Edges
    MapOfSubMaps submaps_origin;
    VectorOfEdges edges_origin;
    CHECK(LoadPoseGraphFromFile(&submaps_origin, &edges_origin)) << "Load Pose Graph Failed! File is "
                                                                 << params_.pose_graph_output_folder + "/" + params_.pose_graph_input_file;
    for (auto submap_itr = submaps_origin.begin(); submap_itr != submaps_origin.cend(); ++submap_itr)
    {
        submap_itr->second.pose.quat.normalize();
    }

    // INIT Calib
    map_pose::Pose3d calib_ldr2gnss_dt;
    calib_ldr2gnss_dt.trans << 0, 0, 0;
    calib_ldr2gnss_dt.quat = Eigen::Quaterniond(Eigen::Matrix3d::Identity()).normalized();
    map_pose::Pose3d calib_ldr2gnss;
    calib_ldr2gnss.trans << 0, 0, 0;
    calib_ldr2gnss.quat = Eigen::Quaterniond(Eigen::Matrix3d::Identity()).normalized();

    // Create GNSS Pose
    map_pose::MapOfGnsses gnsses_origin;
    int invalid_submap_num = 0;
    for (auto itr = submaps_origin.begin(); itr != submaps_origin.end(); itr++)
    {
        map_pose::Gnss gnss;
        gnss.pose.copyFrom(itr->second.raw_data_group[0].raw_gnss.pose);
        gnss.information_matrix = Eigen::Matrix<double, 6, 6>::Zero();
        for (int i = 0; i < 6; ++i)
        {
            // if covariance matrix dialog is zero, information matrix is invalid
            if (itr->second.raw_data_group[0].raw_gnss.information_matrix(i, i) == 0)
            {
                invalid_submap_num++;
                gnss.information_matrix = Eigen::Matrix<double, 6, 6>::Zero();
                break;
            }
            gnss.information_matrix(i, i) = 1 / itr->second.raw_data_group[0].raw_gnss.information_matrix(i, i);
        }
        //        LOG(ERROR) << "information matrix is \n" << gnss.information_matrix;
        gnsses_origin.insert(map_pose::MapOfGnsses::value_type(itr->first, gnss));
    }
    LOG(ERROR) << "There are " << invalid_submap_num << " invalid submaps.";

    // Add Noise
#if 0
        map_pose::AddNoise add_noise;
        map_pose::VectorOfEdges edges_calc;
        map_pose::VectorOfEdges edges_wrong;
        std::vector<map_pose::submap_pair_t> submap_pairs;
        std::vector<map_pose::submap_pair_t> submap_loop_pairs;
        std::vector<map_pose::submap_pair_t> submap_loop_wrong_pairs;
        // Correct Adajacent Edge Idx
        for (size_t i = 0; i < submaps_origin.size() - 1; ++i) {
            map_pose::submap_id_t first;
            map_pose::submap_id_t second;
            first.transaction_id = 0;
            first.submap_id = i;
            second.transaction_id = 0;
            second.submap_id = i + 1;
            map_pose::submap_pair_t submap_pair;
            submap_pair.first_submap = first;
            submap_pair.second_submap = second;
            submap_pairs.push_back(submap_pair);
        }
        size_t flag = 100000;
    //    size_t flag = 179;
        for (size_t i = flag; i < edges_origin.size(); ++i) {
            if (i != flag) {
                // Correct Loop Edge Idx
                submap_loop_pairs.push_back(edges_origin[i].submap_idx);
            } else {
                // Wrong Loop Edge Idx
                submap_loop_wrong_pairs.push_back(edges_origin[i].submap_idx);
                flag += 50;
            }
        }
        submap_pairs.insert(submap_pairs.end(), submap_loop_pairs.begin(), submap_loop_pairs.end());
        add_noise.CalcEdge(submaps_origin, submap_pairs, edges_calc);
    //    add_noise.AddWrongEdge(submaps_origin, submap_loop_wrong_pairs, edges_wrong, 1, M_PI/180);
        edges_calc.insert(edges_calc.end(), edges_wrong.begin(), edges_wrong.end());
        map_pose::MapOfSubMaps submaps_noise = submaps_origin;
        add_noise.Perturb(submaps_noise, 10.0, M_PI/2);         // Bad Init
#endif

    //! INIT Evaluate
    std::vector<submapid_error_t, Eigen::aligned_allocator<submapid_error_t> > err_submaps;
    std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > err_edges;
    SetPoseGraph(edges_origin, submaps_origin, gnsses_origin, calib_ldr2gnss, calib_ldr2gnss_dt);
    FrameMae();
    CheckPGOResult(err_submaps, err_edges);

    // Write Init Result Out
    data_loader_.WritePGOResultOut(params_.pose_graph_output_folder, "init_evaluate.txt", err_submaps, err_edges);

    //! PGO Step By Step
#if 0
    //    char ch;
//    LOG(WARNING) << "Wait Key";
//    cin >> ch;
    OptimizeRot();
//    LOG(INFO) << "*** OptimizeRot ***";
    FrameMae();
//    LOG(WARNING) << "Wait Key";
//    cin >> ch;
    OptimizeTrans();
//    LOG(INFO) << "*** OptimizeTrans ***";
    FrameMae();
//    LOG(WARNING) << "Wait Key";
//    cin >> ch;
//    OptimizeRotAndTrans();
//    LOG(INFO) << "*** OptimizeRotAndTrans ***";
//    FrameMae();
//    LOG(WARNING) << "Wait Key";
//    cin >> ch;
    OptimizeRotAndTransAndGnss();
    LOG(INFO) << "*** OptimizeRotAndTransAndGnss ***";
    FrameMae();
        //    LOG(WARNING) << "Wait Key";
    //    cin >> ch;
        //    OptimizeRotAndTrans();
    //    FrameMae();
    //    PrintProblemSubmaps();
    //    LOG(WARNING) << "Delta Calibration is:\n" << calib_ldr2gnss_dt_.GetMatrix();
        //     OptimizeRotAndTransAndGnssAndCalib();
    //     LOG(INFO) << "*** OptimizeRotAndTransAndGnssAndCalib ***";
    //     FrameMae();
    //     LOG(WARNING) << "Delta Calibration is:\n" << calib_ldr2gnss_dt_.GetMatrix();
    //     PrintProblemSubmaps();
#endif

    //! PGO Optimize ROT & TRANS & GNSS, Now Works the best
    SetPoseGraph(edges_origin, submaps_origin, gnsses_origin, calib_ldr2gnss, calib_ldr2gnss_dt);
    LOG(INFO) << "*** OptimizeRotAndTransAndGnss ***";
    OptimizeRotAndTransAndGnss();
    // after optimization Evaluate
    UpdateFramePose();
    SubmapMae();
    FrameMae();
    LOG(WARNING) << "Delta Calibration is:\n"
                 << calib_ldr2gnss_dt_.GetMatrix();
    PrintProblemSubmaps();
    CheckPGOResult(err_submaps, err_edges);
    // New PGO, Delete Wrong Edges According to PGO Result
#if 1
    double theta_threshold = 1 * M_PI / 180;         // unit rad
    double xyz_norm_threshold = 0.1;    // unit m
    std::set<submap_id_t> check_submaps;
    std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > del_edges;
    for (int i = 0; i < err_edges.size(); ++i) {
        if (err_edges[i].theta > theta_threshold) {
            del_edges.push_back(err_edges[i]);
            continue;
        }
        if (err_edges[i].pose_error.trans.norm() > xyz_norm_threshold) {
            del_edges.push_back(err_edges[i]);
        }
    }
    LOG(INFO) << ">> PGO(del wrong edges): There are " << del_edges.size() << " edges to del.";
    for (int i = 0; i < del_edges.size(); ++i) {
        std::cout << del_edges[i].edge.submap_idx << "trans is " << del_edges[i].pose_error.trans << " theta is " << del_edges[i].theta * 180 / M_PI << std::endl;
    }
    std::set<submap_id_t> submaps_link_to_del_edges;
    for (int i = 0; i < del_edges.size(); ++i) {
        if (submaps_link_to_del_edges.find(del_edges[i].edge.submap_idx.first_submap) == submaps_link_to_del_edges.cend()) {
            submaps_link_to_del_edges.insert(del_edges[i].edge.submap_idx.first_submap);
        } else {
            check_submaps.insert(del_edges[i].edge.submap_idx.first_submap);
        }
        if (submaps_link_to_del_edges.find(del_edges[i].edge.submap_idx.second_submap) == submaps_link_to_del_edges.cend()) {
            submaps_link_to_del_edges.insert(del_edges[i].edge.submap_idx.second_submap);
        } else {
            check_submaps.insert(del_edges[i].edge.submap_idx.second_submap);
        }
    }
    LOG(INFO) << ">> PGO(del wrong edges): Following " << check_submaps.size() << " Submaps need to be checked!";
    for (auto itr = check_submaps.cbegin(); itr != check_submaps.cend(); ++itr) {
        std::cout << *itr << std::endl;
    }
#if 0
    // PGO After Del Edges
    DelEdges(del_edges);
    SetPoseGraph(edges_origin, submaps_origin, gnsses_origin, calib_ldr2gnss, calib_ldr2gnss_dt);
    LOG(INFO) << "*** OptimizeRotAndTransAndGnss ***";
    OptimizeRotAndTransAndGnss();
    // after optimization Evaluate
    UpdateFramePose();
    SubmapMae();
    FrameMae();
    LOG(WARNING) << "Delta Calibration is:\n"
                 << calib_ldr2gnss_dt_.GetMatrix();
    PrintProblemSubmaps();
    CheckPGOResult(err_submaps, err_edges);
#endif

#endif


    // Write PGO Result Out
    data_loader_.WritePGOResultOut(params_.pose_graph_output_folder, "pgo_evaluate.txt", err_submaps, err_edges);

    // Sort Submaps and Edges according to Error

    // Write PGO Pose Out
    WritePoseToFile();

    // Visualization
    map_pose::MapViewer<pcl::PointXYZ> map_viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pg_viewer(new pcl::visualization::PCLVisualizer("PoseGraph"));
    map_viewer.DispalySubmaps(pg_viewer, submaps_origin, "origin", 1, 0, 0);
    //    map_viewer.DispalyEdges(viewer, submaps_origin, edges_origin, "origin", 1, 0, 0);
    //    map_viewer.DispalyEdges(viewer, submaps_origin, bad_edges, "bad", 1, 1, 0);
    //    map_viewer.DispalySubmaps(pg_viewer, submaps_noise, "noise", 1, 1, 0);
    //    map_viewer.DispalyEdges(viewer, submaps, edges_calc, "origin", 1, 1, 0);
    const map_pose::MapOfSubMaps *sub_maps_ptr = GetSubmaps();
    map_viewer.DispalySubmaps(pg_viewer, *sub_maps_ptr, "optimize", 0, 1, 0);
    //    map_viewer.DispalyEdges(pg_viewer, *sub_maps_ptr, edges_origin, "optimize", 0, 1, 0);
    map_pose::VectorOfEdges ad_edges;
    map_pose::VectorOfEdges loop_edges;
    for (int i = 0; i < edges_origin.size(); ++i)
    {
        if (edges_origin[i].edge_type == map_pose::Edge::Adjacent)
            ad_edges.push_back(edges_origin[i]);
        else if (edges_origin[i].edge_type == map_pose::Edge::Intra)
            loop_edges.push_back(edges_origin[i]);
    }
    //    map_viewer.DispalyEdges(pg_viewer, submaps_origin, ad_edges, "optimize", 0, 1, 0);
    //    map_viewer.DispalyEdges(pg_viewer, submaps_origin, loop_edges, "optimize", 1, 1, 1);
    //    map_viewer.DispalyEdges(viewer, *sub_maps_ptr, edges_wrong, "wrong_edge", 1, 0, 0);

    while (!pg_viewer->wasStopped())
    {
        pg_viewer->spinOnce(1);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

bool BackEndOptimization::DelEdges(const std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > &del_edges) {
        for (int i = 0; i < del_edges.size(); ++i) {
            CHECK(DelEdge(del_edges[i].edge.submap_idx)) << "Del Edge Failed, edge is " << del_edges[i].edge.submap_idx;
        }
    }

    bool BackEndOptimization::DelEdge(const submap_pair_t &del_edge_idx) {
        for (auto itr = adjacent_edges_.begin(); itr != adjacent_edges_.end(); ++itr) {
            if (del_edge_idx == itr->submap_idx) {
                adjacent_edges_.erase(itr);
                adjacent_edges_.shrink_to_fit();
                return true;
            }
        }
        for (auto itr = inter_loops_.begin(); itr != inter_loops_.end(); ++itr) {
            if (del_edge_idx == itr->submap_idx) {
                inter_loops_.erase(itr);
                inter_loops_.shrink_to_fit();
                return true;
            }
        }
        for (auto itr = intra_loops_.begin(); itr != intra_loops_.end(); ++itr) {
            if (del_edge_idx == itr->submap_idx) {
                intra_loops_.erase(itr);
                intra_loops_.shrink_to_fit();
                return true;
            }
        }
        LOG(WARNING) << "Del Edge Not Found, Edge is " << del_edge_idx;
        return true;
    }

bool BackEndOptimization::CheckPose(const Pose3d &pose) const
{
    for (int i = 0; i < 3; ++i)
    {
        if (CheckNanInf(pose.trans[i]))
            return false;
    }
    if (CheckNanInf(pose.quat.x()))
        return false;
    if (CheckNanInf(pose.quat.y()))
        return false;
    if (CheckNanInf(pose.quat.z()))
        return false;
    if (CheckNanInf(pose.quat.w()))
        return false;
    return true;
}
bool BackEndOptimization::CheckInformationMatrix(const Eigen::Matrix<double, 6, 6> &information_matrix) const
{
    for (int i = 0; i < 36; ++i)
    {
        if (CheckNanInf(information_matrix(i / 6, i % 6)))
            return false;
    }
    return true;
}
bool BackEndOptimization::CheckNanInf(const double &num) const
{
    if (std::isnan(num))
        return true;
    if (std::isinf(num))
        return true;
    return false;
}
} // namespace map_pose

#endif