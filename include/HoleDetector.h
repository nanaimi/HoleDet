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
    void setBoundarySearchRadius(const float value);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_hull_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries;

    pcl::ModelCoefficients::Ptr floor_coefficients;
    std::vector<pcl::Vertices> hull_polygons;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    pcl::ConvexHull<pcl::PointXYZ> cvxhull;

    // Holes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> holes;
    std::vector<int> hole_sizes;
    std::vector<pcl::PointXYZ> centers;
    std::vector<double> hole_areas;

    // Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer;

    // Tuning Parameters
    double min_size;
    float boundary_search_radius;



    void init_filters();
    void pre_process();
    void calculate();

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    void keyboard_callback(const pcl::visualization::KeyboardEvent &event, void *viewer_void);
    static void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

};


#endif //HOLEDET_HOLEDETECTOR_H
