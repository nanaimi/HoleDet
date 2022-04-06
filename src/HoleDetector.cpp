//
// Created by hanlonm on 06.04.22.
//

#include "../include/HoleDetector.h"

HoleDetector::HoleDetector(const std::basic_string<char> &file_name):
raw_cloud(new pcl::PointCloud<pcl::PointXYZ>),
filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>),
floor(new pcl::PointCloud<pcl::PointXYZ>),
floor_projected(new pcl::PointCloud<pcl::PointXYZ>),
hull_cloud(new pcl::PointCloud<pcl::PointXYZ>),
interior_boundaries(new pcl::PointCloud<pcl::PointXYZ>),
viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")),
floor_coefficients (new pcl::ModelCoefficients)
{
    pointcloud_file = file_name;
    min_size = 10;
}

void HoleDetector::init_filters() {
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);

    voxel_filter.setMinimumPointsNumberPerVoxel(2);
    voxel_filter.setLeafSize (0.1, 0.1, 0.1);
}

void HoleDetector::pre_process() {
    outrem.setInputCloud(raw_cloud);
    outrem.filter (*filtered_cloud);

//    voxel_filter.setInputCloud (filtered_cloud);
//    voxel_filter.filter(*filtered_cloud);

}

void HoleDetector::detectHoles() {
    init_filters();
    raw_cloud = readCloud(pointcloud_file, reader);
    pre_process();
    extractAndProjectFloor(filtered_cloud, floor, floor_projected, floor_coefficients);
    createConcaveHull(floor_projected, hull_cloud, hull_polygons, chull);
    getInteriorBoundaries(floor_projected, hull_cloud, interior_boundaries);
    getHoleClouds(interior_boundaries, holes, hole_sizes);
    calcHoleCenters(holes, min_size, centers);

}

void HoleDetector::visualize() {
    viewer->addPointCloud(hull_cloud,"hull");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"hull");

    viewer ->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

    for (int i = 0; i < holes.size(); ++i) {
        if (holes[i]->points.size() < 10) { continue; }
        auto name = "hole_" + std::to_string(i);
        float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        viewer->addPointCloud(holes[i], name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r1, 1 - r1, r3, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

        auto center_name = "hole_center_" + std::to_string(i);
        viewer->addSphere(centers[i],0.3,1.0,1.0,0.0, center_name);


    }
//    viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
//    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);


}

void HoleDetector::pp_callback(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
    std::cout << "Picking event active" << std::endl;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << x << "; " << y << "; " << z << std::endl;
    }
}

void HoleDetector::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    pcl::visualization::PCLVisualizer::Ptr event_viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (event_viewer->contains("cloud")) {
            cout << "c was pressed => removing preprocessed PointCloud" << endl;
            event_viewer->removePointCloud("cloud");
        } else {
            cout << "c was pressed => showing preprocessed PointCloud" << endl;
            event_viewer->addPointCloud(filtered_cloud,"cloud");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
            event_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"cloud");
        }
    }
}