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
    PointCloud<PointXYZ>::Ptr raw_cloud;
    PointCloud<PointXYZ>::Ptr filtered_cloud;
    PointCloud<PointXYZ>::Ptr floor;
    PointCloud<PointXYZ>::Ptr floor_projected_;
    PointCloud<PointXYZ>::Ptr hull_cloud;
    PointCloud<PointXYZ>::Ptr interior_boundaries;
    PointCloud<PointXYZ>::Ptr floorplan_;

    ModelCoefficients::Ptr floor_coefficients;
    vector<Vertices> hull_polygons;
    ConcaveHull<PointXYZ> chull;

    // Holes
    vector<PointCloud<PointXYZ>::Ptr> holes;
    vector<int> hole_sizes;
    int min_size;
    vector<PointXYZ> centers;

    // Visualizer
    visualization::PCLVisualizer::Ptr viewer;



    void init_filters();
    void pre_process();

    void keyboardEventOccurred (const visualization::KeyboardEvent &event, void* viewer_void);
    void pp_callback(const visualization::PointPickingEvent& event, void* viewer_void);
    static void point_picker_cb(const visualization::PointPickingEvent& event, void* param);
    static void onMouse(int event, int x, int y, int flags, void* param);

};


#endif //HOLEDET_HOLEDETECTOR_H
