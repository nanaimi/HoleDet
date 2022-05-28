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

    /// Main hole detecting function
    void DetectHoles();

    /// Creates a point cloud of the vertices aligned with original floor plan vertices
    void GetFloorplanCloud(string floorplan_path);

    /// Creates the mesh using an augmented and filtered point cloud
    void GetFullMesh();

    /// Calculates all the score components for each hole
    void CalculateScoresAndPoses();

    /// Create the gaze maps in four different directions
    void GazeMap();

    /// Visualizes everyting
    void Visualize();

private:
    // all paths and files
    basic_string<char> path_;
    basic_string<char> config_file_;
    basic_string<char> pointcloud_file_;
    basic_string<char> trajectory_file_;
    basic_string<char> gaze_file_;
    basic_string<char> lengths_file_;
    basic_string<char> floorplan_file_;

    // PCD Reader
    PCDReader reader_;

    // PCL Visualizer
    visualization::PCLVisualizer::Ptr viewer_;

    // Constants and tuning parameters
    bool kUseGaze_;
    bool kUseExistingFloorplan_;
    int kPoissonDepth_;
    float kNormalSearchRadius_;
    double kOutlierRadius_;
    int kMinNeighbours_;
    double kImgResolution_; // [m/px]
    int kMaxIteration_;
    int kMaxTranslation_;
    int kMaxAngle_; // [deg]
    double kMinScore_;
    float kStepBack_;
    float kBoundarySearchRadius_;
    float kAngleThresh_;
    float kVertScoreThresh_;

    // Hole variables
    vector<Hole> holes_;
    int hole_index_;

    // Gaze variables
    GazeScores gaze_scores_;

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
    pcl::PointCloud<pcl::Normal>::Ptr floor_normals_;
    pcl::PointCloud<pcl::Normal>::Ptr boundary_normals_;
    PointCloud<PointXYZ>::Ptr floorplan_filtered_;

    // Poses and headings
    std::vector<PointCloud<PointXYZ>::Ptr> trajectories_;
    std::vector<std::vector<Eigen::Vector3f>> gazes_;

    // Meshes
    pcl::PolygonMesh::Ptr full_mesh_;

    // Reconstruction variables
    pcl::ModelCoefficients::Ptr floor_coefficients_;
    std::vector<pcl::Vertices> hull_polygons_;
    pcl::ConcaveHull<pcl::PointXYZ> chull_;
    pcl::ConvexHull<pcl::PointXYZ> cvxhull_;

    /// Reads in the config file and initializes respective constants.
    void ReadYAML();

    /// Initializes outlier removal filter and voxel filter.
    void InitFilters();

    /// Applies initial filtering; removes noise
    void PreProcess();

    /// Calculate the centroids of the detected holes
    void CalculateCentroids();

    /// Creates heatmaps and visualizes them
    void CreateAndVisualizeHeatMap();

    /// Keyboard listener for interacting with visualization
    void KeyboardEventOccurred(const visualization::KeyboardEvent &event, void *viewer_void);

    /// Get coordinates of the picked points
    static void PpCallback(const visualization::PointPickingEvent &event, void *viewer_void);

    /// Callback for selecting vertices of floorplan in point cloud
    static void PointPickerCb(const visualization::PointPickingEvent &event, void *param);

    /// Callback for selecting vertices of floorplan in images
    static void OnMouse(int event, int x, int y, int flags, void *param);

};


#endif //HOLEDET_HOLEDETECTOR_H
