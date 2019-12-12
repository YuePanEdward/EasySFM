#include "viewer.h"
#include "estimate_motion.h"

using namespace p3dv;

bool MapViewer::displaySFM(std::vector<frame_t> &frames, std::vector<bool> &frames_to_process,
                           pointcloud_sparse_t &sparse_pointcloud, std::string viewer_name,
                           bool black_background, bool render_point_as_sphere)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(viewer_name));
    if (black_background)
        viewer->setBackgroundColor(0, 0, 0);
    else
        viewer->setBackgroundColor(255, 255, 255);

    char t[256];
    std::string s;
    int n = 0;
    float frame_color_r, frame_color_g, frame_color_b;
    float sphere_size = 0.02;
    float point_size = 0.04;
    float line_size_cam_z = 0.4;
    float line_size_cam_x = 0.6;
    float line_size_cam_y = 0.4;

    // Draw camera
    for (int i = 0; i < frames.size(); i++)
    {
        if (!frames_to_process[i])
        {

            std::cout << "Add frame [ " << i << " ] , pose:\n"
                      << frames[i].pose_cam << std::endl;

            //std::cout << "Begin" << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr camera_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_camera_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointXYZ pt_cam_center(0, 0, 0);
            camera_pointcloud->points.push_back(pt_cam_center);
            pcl::PointXYZ pt_cam_corner_1(-line_size_cam_x / 2, -line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_1);
            pcl::PointXYZ pt_cam_corner_2(-line_size_cam_x / 2, line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_2);
            pcl::PointXYZ pt_cam_corner_3(line_size_cam_x / 2, line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_3);
            pcl::PointXYZ pt_cam_corner_4(line_size_cam_x / 2, -line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_4);

            Eigen::Matrix4f test_mat = frames[i].pose_cam;
            MotionEstimator ee;
            ee.transformCloud(camera_pointcloud, transformed_camera_pointcloud, test_mat);

            sprintf(t, "point1_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[0], sphere_size, 1.0, 1.0, 1.0, s);
            sprintf(t, "point2_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[1], sphere_size, 1.0, 1.0, 1.0, s);
            sprintf(t, "point3_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[2], sphere_size, 1.0, 1.0, 1.0, s);
            sprintf(t, "point4_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[3], sphere_size, 1.0, 1.0, 1.0, s);
            sprintf(t, "point5_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[4], sphere_size, 1.0, 1.0, 1.0, s);

            sprintf(t, "line1_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[1], 1.0, 0.0, 0.0, s);
            sprintf(t, "line2_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[2], 1.0, 0.0, 0.0, s);
            sprintf(t, "line3_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[3], 1.0, 0.0, 0.0, s);
            sprintf(t, "line4_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[4], 1.0, 0.0, 0.0, s);
            sprintf(t, "line5_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[1], transformed_camera_pointcloud->points[2], 1.0, 0.0, 0.0, s);
            sprintf(t, "line6_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[2], transformed_camera_pointcloud->points[3], 1.0, 0.0, 0.0, s);
            sprintf(t, "line7_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[3], transformed_camera_pointcloud->points[4], 1.0, 0.0, 0.0, s);
            sprintf(t, "line8_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[4], transformed_camera_pointcloud->points[1], 1.0, 0.0, 0.0, s);

            n++;
        }
        //std::cout << "Add Camera " << i << " done." << std::endl;
    }

    double show_coor_thre = 200;

    if (render_point_as_sphere)
    {
        // Draw point cloud as sphere (slower)
        for (int i = 0; i < sparse_pointcloud.rgb_pointcloud->points.size(); i++)
        {
            char sparse_point[256];
            pcl::PointXYZ ptc_temp;
            ptc_temp.x = sparse_pointcloud.rgb_pointcloud->points[i].x;
            ptc_temp.y = sparse_pointcloud.rgb_pointcloud->points[i].y;
            ptc_temp.z = sparse_pointcloud.rgb_pointcloud->points[i].z;

            sprintf(sparse_point, "SP_%03u", i);
            if (std::abs(ptc_temp.x) < show_coor_thre && std::abs(ptc_temp.y) < show_coor_thre && std::abs(ptc_temp.z) < show_coor_thre)
                viewer->addSphere(ptc_temp, point_size, sparse_pointcloud.rgb_pointcloud->points[i].r / 255.0, sparse_pointcloud.rgb_pointcloud->points[i].g / 255.0, sparse_pointcloud.rgb_pointcloud->points[i].b / 255.0, sparse_point);
        }
    }
    else
    {
        // Draw point cloud with point (faster)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < sparse_pointcloud.rgb_pointcloud->points.size(); i++)
        {
            if (std::abs(sparse_pointcloud.rgb_pointcloud->points[i].x) < show_coor_thre &&
                std::abs(sparse_pointcloud.rgb_pointcloud->points[i].y) < show_coor_thre &&
                std::abs(sparse_pointcloud.rgb_pointcloud->points[i].z) < show_coor_thre)
                show_pointcloud->points.push_back(sparse_pointcloud.rgb_pointcloud->points[i]);
        }
        viewer->addPointCloud(show_pointcloud, "sparsepointcloud");
    }

    //viewer->addPointCloud(sparse_pointcloud.rgb_pointcloud, "sparsepointcloud");

    std::cout << "Click X(close) to continue..." << std::endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return true;
}

bool MapViewer::displaySFM_on_fly(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                                  std::vector<frame_t> &frames, std::vector<bool> &frames_to_process,
                                  pointcloud_sparse_t &sparse_pointcloud, double relative_depth,
                                  int display_time_ms, bool render_point_as_sphere)
{
    if (!is_frist_frame_) // You need to remove the original camera and point cloud first
    {
        viewer->removeAllShapes();
        if (!render_point_as_sphere_)
            viewer->removePointCloud("sparsepointcloud");
    }
    else
    {
        //Set shape size according to relative depth.
        approximate_scale_ = relative_depth / 5.0;
        render_point_as_sphere_ = render_point_as_sphere;
        is_frist_frame_ = 0;
    }

    char t[256];
    std::string s;
    int n = 0;

    // convert unit from meter to baseline_length
    float sphere_size = approximate_scale_ * 0.02;
    float point_size = approximate_scale_ * 0.02;
    float line_size_cam_z = approximate_scale_ * 0.4;
    float line_size_cam_x = approximate_scale_ * 0.6;
    float line_size_cam_y = approximate_scale_ * 0.4;

    // Draw camera
    for (int i = 0; i < frames.size(); i++)
    {
        if (!frames_to_process[i])
        {
            std::cout << "Add frame [ " << i << " ] , pose:\n"
                      << frames[i].pose_cam << std::endl;

            //std::cout << "Begin" << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr camera_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_camera_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointXYZ pt_cam_center(0, 0, 0);
            camera_pointcloud->points.push_back(pt_cam_center);
            pcl::PointXYZ pt_cam_corner_1(-line_size_cam_x / 2, -line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_1);
            pcl::PointXYZ pt_cam_corner_2(-line_size_cam_x / 2, line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_2);
            pcl::PointXYZ pt_cam_corner_3(line_size_cam_x / 2, line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_3);
            pcl::PointXYZ pt_cam_corner_4(line_size_cam_x / 2, -line_size_cam_y / 2, line_size_cam_z);
            camera_pointcloud->points.push_back(pt_cam_corner_4);

            Eigen::Matrix4f test_mat = frames[i].pose_cam;
            MotionEstimator ee;
            ee.transformCloud(camera_pointcloud, transformed_camera_pointcloud, test_mat);

            sprintf(t, "point1_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[0], sphere_size, 1.0, 0.0, 0.0, s);
            sprintf(t, "point2_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[1], sphere_size, 1.0, 0.0, 0.0, s);
            sprintf(t, "point3_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[2], sphere_size, 1.0, 0.0, 0.0, s);
            sprintf(t, "point4_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[3], sphere_size, 1.0, 0.0, 0.0, s);
            sprintf(t, "point5_%d", n);
            s = t;
            viewer->addSphere(transformed_camera_pointcloud->points[4], sphere_size, 1.0, 0.0, 0.0, s);

            sprintf(t, "line1_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[1], 1.0, 0.0, 0.0, s);
            sprintf(t, "line2_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[2], 1.0, 0.0, 0.0, s);
            sprintf(t, "line3_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[3], 1.0, 0.0, 0.0, s);
            sprintf(t, "line4_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[0], transformed_camera_pointcloud->points[4], 1.0, 0.0, 0.0, s);
            sprintf(t, "line5_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[1], transformed_camera_pointcloud->points[2], 1.0, 0.0, 0.0, s);
            sprintf(t, "line6_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[2], transformed_camera_pointcloud->points[3], 1.0, 0.0, 0.0, s);
            sprintf(t, "line7_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[3], transformed_camera_pointcloud->points[4], 1.0, 0.0, 0.0, s);
            sprintf(t, "line8_%d", n);
            s = t;
            viewer->addLine(transformed_camera_pointcloud->points[4], transformed_camera_pointcloud->points[1], 1.0, 0.0, 0.0, s);

            n++;
        }
        //std::cout << "Add Camera " << i << " done." << std::endl;
    }

    double show_coor_thre = 200;
    if (render_point_as_sphere_)
    {
        // Draw point cloud with sphere (slower)
        for (int i = 0; i < sparse_pointcloud.rgb_pointcloud->points.size(); i++)
        {
            char sparse_point[256];
            pcl::PointXYZ ptc_temp;
            ptc_temp.x = sparse_pointcloud.rgb_pointcloud->points[i].x;
            ptc_temp.y = sparse_pointcloud.rgb_pointcloud->points[i].y;
            ptc_temp.z = sparse_pointcloud.rgb_pointcloud->points[i].z;

            sprintf(sparse_point, "SP_%03u", i);
            if (std::abs(ptc_temp.x) < show_coor_thre && std::abs(ptc_temp.y) < show_coor_thre && std::abs(ptc_temp.z) < show_coor_thre)
                viewer->addSphere(ptc_temp, point_size, sparse_pointcloud.rgb_pointcloud->points[i].r / 255.0, sparse_pointcloud.rgb_pointcloud->points[i].g / 255.0, sparse_pointcloud.rgb_pointcloud->points[i].b / 255.0, sparse_point);
        }
    }
    else
    {
        // Draw point cloud with point (faster)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < sparse_pointcloud.rgb_pointcloud->points.size(); i++)
        {
            if (std::abs(sparse_pointcloud.rgb_pointcloud->points[i].x) < show_coor_thre &&
                std::abs(sparse_pointcloud.rgb_pointcloud->points[i].y) < show_coor_thre &&
                std::abs(sparse_pointcloud.rgb_pointcloud->points[i].z) < show_coor_thre)
                show_pointcloud->points.push_back(sparse_pointcloud.rgb_pointcloud->points[i]);
        }
        viewer->addPointCloud(show_pointcloud, "sparsepointcloud");
    }

    std::cout << "Update the viewer done." << std::endl;

    viewer->spinOnce(display_time_ms);
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));
}

bool MapViewer::displayFrame(frame_t &cur_frame, std::string viewer_name, int time_delay_ms)
{
    cv::Mat feature_image;
    cv::namedWindow(viewer_name, 0);
    cv::drawKeypoints(cur_frame.rgb_image, cur_frame.keypoints, feature_image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow(viewer_name, feature_image);
    cv::waitKey(time_delay_ms);
}