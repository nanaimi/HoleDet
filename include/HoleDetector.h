//
// Created by hanlonm on 06.04.22.
//

#ifndef HOLEDET_HOLEDETECTOR_H
#define HOLEDET_HOLEDETECTOR_H

#include "holedet_utils.h"

using namespace pcl;
using namespace Eigen;
using namespace std;

struct MouseParams {
    cv::Mat img;
    vector<cv::Point> points;
};

struct TransformPoints {
    vector<Vector3f> points;
    int cnt;
};

class HoleDetector {
public:
    HoleDetector(const std::basic_string<char> &file_name, const std::basic_string<char> &floorplan_path);

    void detectHoles();
    void getFloorplanCloud(bool debug, string floorplan_path);
    void visualize();
    void setBoundarySearchRadius(const float value);

private:
    // constants TODO Move to yaml file
    const float kImgResolution_ = 0.0694; // [m/px]
    const int kMaxIteration_ = 10000;
    const int kMaxTranslation_ = 5;
    const int kMaxAngle_ = 10; // [deg]

    basic_string<char> pointcloud_file_;
    basic_string<char> floorplan_file_;
    PCDReader reader;

    // Point Picking
    MouseParams mp_;
    TransformPoints tp_;

    // Filters
    RadiusOutlierRemoval<PointXYZ> outrem;
    VoxelGrid<PointXYZ> voxel_filter;


    // Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_hull_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries;
    PointCloud<PointXYZ>::Ptr floor_projected_;
    PointCloud<PointXYZ>::Ptr floorplan_;
    PointCloud<PointXYZ>::Ptr dense_floorplan_;


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
    visualization::PCLVisualizer::Ptr viewer;

    // Tuning Parameters
    double min_size;
    float boundary_search_radius;



    void init_filters();
    void pre_process();
    void calculate();

    void keyboardEventOccurred (const visualization::KeyboardEvent &event, void* viewer_void);
    static void pp_callback(const visualization::PointPickingEvent& event, void* viewer_void);
    static void point_picker_cb(const visualization::PointPickingEvent& event, void* param);
    static void onMouse(int event, int x, int y, int flags, void* param);

};


#endif //HOLEDET_HOLEDETECTOR_H
