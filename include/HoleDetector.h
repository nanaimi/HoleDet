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

    void DetectHoles();
    void GetFloorplanCloud(bool debug, string floorplan_path);
    void CalculateScores();
    void Visualize();
    void SetBoundarySearchRadius(const float value);

private:
    // constants
    basic_string<char> path_;
    basic_string<char> config_file_;
    basic_string<char> pointcloud_file_;
    basic_string<char> trajectory_file_;
    basic_string<char> gaze_file_;
    basic_string<char> lengths_file_;
    basic_string<char> floorplan_file_;
    PCDReader reader;

    bool debug_;

    float kStartScore_;

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

    // vector of holes
    vector<Hole> holes_;

    // Point Picking
    MouseParams mp_;
    TransformPoints tp_;

    // Filters
    RadiusOutlierRemoval<PointXYZ> outrem_;
    VoxelGrid<PointXYZ> voxel_filter_;

    // Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_hull_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries_;
    PointCloud<PointXYZ>::Ptr floor_projected_;
    PointCloud<PointXYZ>::Ptr floorplan_;
    PointCloud<PointXYZ>::Ptr dense_floorplan_;

    std::vector<PointCloud<PointXYZ>::Ptr> trajectories_;
    std::vector<std::vector<Eigen::Vector3f>> gazes_;


    pcl::ModelCoefficients::Ptr floor_coefficients_;
    std::vector<pcl::Vertices> hull_polygons_;
    pcl::ConcaveHull<pcl::PointXYZ> chull_;
    pcl::ConvexHull<pcl::PointXYZ> cvxhull_;

    // Visualizer
    visualization::PCLVisualizer::Ptr viewer_;

    // Tuning Parameters
    double min_score_;
    float boundary_search_radius_;


    void ReadYAML();
    void InitFilters();
    void PreProcess();
    void CalculateCentroids();

    void KeyboardEventOccurred (const visualization::KeyboardEvent &event, void* viewer_void);
    static void PpCallback(const visualization::PointPickingEvent& event, void* viewer_void);
    static void PointPickerCb(const visualization::PointPickingEvent& event, void* param);
    static void OnMouse(int event, int x, int y, int flags, void* param);

};


#endif //HOLEDET_HOLEDETECTOR_H
