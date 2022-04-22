//
// Created by hanlonm on 06.04.22.
//

#ifndef HOLEDET_HOLEDETECTOR_H
#define HOLEDET_HOLEDETECTOR_H

#include "holedet_utils.h"
#include "yaml-cpp/yaml.h"

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
    HoleDetector(const basic_string<char> &path, const basic_string<char> &config_filename);

    void detectHoles();
    void getFloorplanCloud(bool debug, string floorplan_path);
    void visualize();
    void setBoundarySearchRadius(const float value);

private:
    // constants TODO Move to yaml file
    basic_string<char> path_;
    basic_string<char> config_file_;
    basic_string<char> pointcloud_file_;
    basic_string<char> trajectory_file_;
    basic_string<char> floorplan_file_;
    PCDReader reader;

    bool debug_;

    int kPoissonDepth_;
    float kNormalSearchRadius_;

    double kOutlierRadius_;
    int kMinNeighbours_;

    double kPassXLimMin_;
    double kPassXLimMax_;
    double kPassYLimMin_;
    double kPassYLimMax_;

    double kImgResolution_; // [m/px]
    int kMaxIteration_;
    int kMaxTranslation_;
    int kMaxAngle_; // [deg]

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


    void ReadYAML();
    void init_filters();
    void pre_process();
    void calculate();

    void keyboardEventOccurred (const visualization::KeyboardEvent &event, void* viewer_void);
    static void pp_callback(const visualization::PointPickingEvent& event, void* viewer_void);
    static void point_picker_cb(const visualization::PointPickingEvent& event, void* param);
    static void onMouse(int event, int x, int y, int flags, void* param);

};


#endif //HOLEDET_HOLEDETECTOR_H
