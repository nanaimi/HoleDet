//
// Created by hanlonm on 06.04.22.
//

#ifndef HOLEDET_HOLEDETECTOR_H
#define HOLEDET_HOLEDETECTOR_H

//#include "../libs/include/holedet_utils.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
#include <math.h>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ml/kmeans.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/centroid.h>
#include <numeric>
#include <pcl/segmentation/region_growing.h>


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
