//
// Created by hanlonm on 21.03.22.
//

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv)
{
    using namespace std::chrono_literals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    //reader.read ("../data/table_scene_lms400.pcd", *cloud);
//    reader.read ("/home/hanlonm/HoleDet/Data/projected.pcd", *cloud);
    reader.read ("/home/hanlonm/HoleDet/Data/hololens.pcd", *cloud);
    reader.read ("/home/hanlonm/HoleDet/Data/ba_keyframes.pcd", *trajectory);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // radius outlier removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud);

    // Voxel filter
    std::cerr << *cloud << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (* cloud);




    // extract floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor (new pcl::PointCloud<pcl::PointXYZ>);
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
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *floor);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud);

    // boundary
    // fill in the cloud data here
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // estimate normals and fill in \a normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (floor);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.3);
    ne.compute (*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (floor);
    est.setInputNormals (normals);
    est.setRadiusSearch (0.5);   // 50cm radius
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute (boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i <floor->points.size(); ++i) {
        if(boundaries[i].boundary_point> 0) {
            cloud_boundary->push_back(floor->points[i]);
        }
    }



    viewer->addPointCloud(cloud,"cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"cloud");
    viewer->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

    viewer->addPointCloud(trajectory,"trajectory");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "trajectory");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"trajectory");

    viewer->addPointCloud(cloud_boundary,"bound");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "bound");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"bound");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}