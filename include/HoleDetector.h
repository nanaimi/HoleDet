//
// Created by hanlonm on 06.04.22.
//

#ifndef HOLEDET_HOLEDETECTOR_H
#define HOLEDET_HOLEDETECTOR_H

#include "include/holedet_utils.h"

class HoleDetector {
public:
    HoleDetector(const std::basic_string<char> &file_name);

    void detectHoles();
    void visualize();

private:
    std::basic_string<char> pointcloud_file;
    pcl::PCDReader reader;

    // Filters
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;


    // Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries;

    pcl::ModelCoefficients::Ptr floor_coefficients;
    std::vector<pcl::Vertices> hull_polygons;
    pcl::ConcaveHull<pcl::PointXYZ> chull;

    // Holes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> holes;
    std::vector<int> hole_sizes;
    int min_size;
    std::vector<pcl::PointXYZ> centers;

    // Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer;



    void init_filters();
    void pre_process();

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

};


#endif //HOLEDET_HOLEDETECTOR_H
