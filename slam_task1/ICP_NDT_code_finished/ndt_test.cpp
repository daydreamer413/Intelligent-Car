#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <X11/Xlib.h>
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    XInitThreads();  // 初始化多线程支持
    // 加载目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Could not find target PCD file \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from target_cloud" << std::endl;

    // 加载源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan2.pcd", *source_cloud) == -1)
    {
        PCL_ERROR("Could not find source PCD file \n");
        return (-1);
    }
    std::cout << "Loaded " << source_cloud->size() << " data points from source_cloud" << std::endl;

    // 过滤源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
    approximate_voxel_filter.setInputCloud(source_cloud);
    approximate_voxel_filter.filter(*filter_cloud);
    std::cout << "Filtered cloud contains " << filter_cloud->size() << " data points from source_cloud" << std::endl;

    // 初始化旋转和平移
    Eigen::AngleAxisf init_rotation(0.95, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(2.0, 0, 0); // 
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // =======================   NDT   =======================
    // 创建 NDT 对象
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // 设置 NDT 参数
    ndt.setTransformationEpsilon(0.0001);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);

    // 设置输入点云
    ndt.setInputSource(filter_cloud);
    ndt.setInputTarget(target_cloud);

    // 执行配准
    ndt.align(*output_cloud, init_guess);

    // 检查配准是否成功
    if (ndt.hasConverged())
    {
        std::cout << "NDT has converged." << std::endl;
        std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
    }
    else
    {
        std::cout << "NDT did not converge." << std::endl;
    }

    // 获取最终的变换矩阵
    Eigen::Matrix4f final_transform = ndt.getFinalTransformation();
    std::cout << "Final transformation matrix: \n" << final_transform << std::endl;

    // =======================   NDT   =======================

    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // 可视化目标点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

    // // 可视化源点云（原始点云）
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 0, 0, 255); // 蓝色
    // viewer_final->addPointCloud<pcl::PointXYZ>(source_cloud, source_color, "source_cloud");
    // viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");

    // 可视化输出点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");

    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // 可视化循环
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}



