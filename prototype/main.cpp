#include <iostream>
#include <vector>
#include <thread>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>

using namespace std::chrono_literals;

void FilterPointCLoud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_x, float leaf_y, float leaf_z) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leaf_x, leaf_y, leaf_z);
    filter.filter(*cloud);
}

int main() {
    std::string dataset = "/home/maurice/ETH/HoleDet/prototype/data/hololens.pcd";
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    bool visuals = true;

    const  float leaf_x = 0.1;
    const float leaf_y = 0.1;
    const float leaf_z = 0.1;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (dataset, *cloud) == -1) {
          PCL_ERROR ("Couldn't read file\n");
          return (-1);
    }
    std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points"
                << std::endl;
    // outlier removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud);

    FilterPointCLoud(cloud, leaf_x, leaf_y, leaf_z);
    std::cout << "Points after first filtering:\t" << cloud->width << std::endl;

    // extract floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *projected_cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setMaxIterations(10000);
    // Segment dominant plane
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);


    // Project everything on the floor
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projected_cloud);
    FilterPointCLoud(projected_cloud, leaf_x, leaf_y, leaf_z);
    std::cout << "Points after second filtering:\t" << projected_cloud->width << std::endl;
    viewer->addPointCloud(projected_cloud, "projected");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                1.0f, 0.0f, 1.0f, "projected");


    /*
    // get heights
    pcl::PointCloud<pcl::PointXYZ>::Ptr max_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*projected_cloud, *max_cloud);
    for(auto& point : *max_cloud) {
        const float x_min = point.x - leaf_x / 2.0;
        const float y_min = point.y - leaf_y / 2.0;
        const float x_max = point.x + leaf_x / 2.0;
        const float y_max = point.y + leaf_y / 2.0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr boxed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(x_min, y_min, -1000.0, 1.0));
        box_filter.setMax(Eigen::Vector4f(x_max, y_max, 1000.0, 1.0));
        box_filter.setInputCloud(cloud);
        box_filter.filter(*boxed_cloud);

        if(boxed_cloud->width == 0) {
            continue;
        }

        auto it = std::max_element(boxed_cloud->begin(), boxed_cloud->end(),
                                   [](auto point_a, auto point_b) { return point_a.z < point_b.z; });
        point = *it;
    }

    viewer->addPointCloud(max_cloud, "max");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                              0.0f, 0.2f, 0.8f, "max");

    pcl::PointCloud<pcl::PointXYZ>::Ptr min_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*projected_cloud, *min_cloud);
    for(auto& point : *min_cloud) {
        const float x_min = point.x - leaf_x / 2.0;
        const float y_min = point.y - leaf_y / 2.0;
        const float x_max = point.x + leaf_x / 2.0;
        const float y_max = point.y + leaf_y / 2.0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr boxed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(x_min, y_min, -1000.0, 1.0));
        box_filter.setMax(Eigen::Vector4f(x_max, y_max, 1000.0, 1.0));
        box_filter.setInputCloud(cloud);
        box_filter.filter(*boxed_cloud);

        if(boxed_cloud->width == 0) {
            continue;
        }

        auto it = std::min_element(boxed_cloud->begin(), boxed_cloud->end(),
                                   [](auto point_a, auto point_b) { return point_a.z < point_b.z; });
        point = *it;
    }

    viewer->addPointCloud(min_cloud, "min");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                              1.0f, 0.0f, 0.0f, "max");*/

    //visuals
    if (visuals) {


        while (!viewer->wasStopped()) {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(10ms);
        }
    }
    return 0;
}
