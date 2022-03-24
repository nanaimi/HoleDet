// created by Maurice Brunner 23.03

#include <iostream>
#include <vector>
#include <thread>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ml/kmeans.h>

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
    /*
    pcl::Kmeans kmeans(interior_boundaries->size(),3);
    kmeans.setClusterSize(100);
    for (size_t i = 0; i < interior_boundaries->points.size(); i++)
    {
        std::vector<float> data(3);
        data[0] = interior_boundaries->points[i].x;
        data[1] = interior_boundaries->points[i].y;
        data[2] = interior_boundaries->points[i].z;
        kmeans.addDataPoint(data);
    }

    kmeans.kMeans();
    auto centroids = kmeans.get_centroids();
    */

    std::cout << interior_boundaries;
    viewer->addPointCloud(interior_boundaries,"bound");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "bound");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"bound");




    viewer->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

    viewer->addCoordinateSystem(2.0);



//    viewer->addPointCloud(trajectory,"trajectory");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "trajectory");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"trajectory");


//    for (int i = 0; i<centroids.size(); i++)
//    {
//        pcl::PointXYZ center;
//        center.x =centroids[i][0];
//        center.y =centroids[i][1];
//        center.z =centroids[i][2];
//        if (isinf(center.x)) continue;
//        auto name = "center_" + std::to_string(i);
//        viewer->addSphere(center,0.5,1.0,1.0,0.0, name);
//        std::cout << center;
//    }

    //visuals
    if (visuals) {


        while (!viewer->wasStopped()) {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(10ms);
        }
    }
    return 0;
}
