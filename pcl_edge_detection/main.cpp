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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>

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

    // Filter out conference room
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    float conf_min_x = -10;
    float conf_max_x = 10;
    float conf_min_y = -10;
    float conf_max_y = 10;
    boxFilter.setMin(Eigen::Vector4f(conf_min_x, conf_min_y, -5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(conf_max_x, conf_max_y, 5, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud);
    boxFilter.setInputCloud(trajectory);
    boxFilter.filter(*trajectory);



    // radius outlier removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud);

    viewer->addPointCloud(cloud,"cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");

    // Voxel filter
    std::cerr << *cloud << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (* cloud);




    // extract floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected (new pcl::PointCloud<pcl::PointXYZ>);
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

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud (floor);
    proj.setModelCoefficients (coefficients);
    proj.filter (*floor_projected);

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> polygons;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (floor_projected);
    chull.setAlpha (1);
    chull.reconstruct (*cloud_hull,polygons);
    viewer->addPointCloud(cloud_hull,"hull");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"hull");

    // boundary
    // fill in the cloud data here
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // estimate normals and fill in \a normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (floor_projected);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.3);
    ne.compute (*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (floor_projected);
    est.setInputNormals (normals);
    est.setRadiusSearch (0.5);   // 50cm radius
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute (boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i <floor_projected->points.size(); ++i) {
        if(boundaries[i].boundary_point> 0) {
            floor_boundary->push_back(floor_projected->points[i]);
        }
    }

//    for (auto it = floor_boundary->begin(); it != floor_boundary->end(); it++) {
//        pcl::PointXYZ point(it->x, it->y, it->z);
//        float min_dist = 100;
//        for (int i = 0; i < cloud_hull->size(); ++i) {
//            float dist = pcl::euclideanDistance(point, cloud_hull->points[i]);
//            if (dist<min_dist){
//                min_dist = dist;
//            }
//        }
////        std::cout << min_dist;
////        std::cout << "\n";
//        if(min_dist<5){
//            floor_boundary->erase(it);
//        }
//
//    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto point: *floor_boundary) {
        float min_dist = 100;
        for (int i = 0; i < cloud_hull->size(); ++i) {
            float dist = pcl::euclideanDistance(point, cloud_hull->points[i]);
            if (dist<min_dist){
                min_dist = dist;
            }
        }
        if(min_dist>0.5){
         interior_boundaries->push_back(point);
        }
    }
    std::cout << interior_boundaries;
    viewer->addPointCloud(interior_boundaries,"bound");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "bound");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"bound");




    viewer->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

    viewer->addCoordinateSystem(2.0);

    viewer->addPointCloud(trajectory,"trajectory");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "trajectory");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"trajectory");




    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}