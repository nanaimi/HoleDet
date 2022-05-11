//
// Created by hanlonm and Maurice Brunner on 06.04.22.
//

#ifndef HOLEDET_HOLEDET_UTILS_H
#define HOLEDET_HOLEDET_UTILS_H

#include <iostream>
#include <vector>
#include <thread>
#include <math.h>
#include <string>
#include <numeric>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/ml/kmeans.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>

#include "Normal2dEstimation.h"


struct Hole {
    pcl::PointXYZ centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    int size;
    Eigen::Affine3f poses;
    float score;
};

struct GazeScores {
    Eigen::MatrixXf scores[4]; //0:0 1:90 2:180, 3:270
    Eigen::MatrixXf occupancy_grid;
    int offset_x;
    int offset_y;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
};

class Utils {
    public:
    ///
    /// \param file_name string, e.g "hololens.pcd"
    /// \param reader pcl::PCDReader reader
    /// \return shared pointer pcl::PointCloud<pcl::PointXYZ>::Ptr
    static pcl::PointCloud<pcl::PointXYZ>::Ptr ReadCloud(const std::basic_string<char> &file_name,
                                                         pcl::PCDReader &reader);

    static void ReadTrajectoriesAndGaze(const std::basic_string<char> &traj_file_name,
                                        const std::basic_string<char> &gaze_file_name,
                                        const std::basic_string<char> &lenghts_file_name,
                                        pcl::PCDReader &reader,
                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& trajectories,
                                        std::vector<std::vector<Eigen::Vector3f>>& gazes);

    ///
    /// \param cloud input point cloud
    /// \param floor cloud to which unprojected floor_ will be extracted
    /// \param floor_projected floor_ cloud projected to the estimated plane equation
    /// \param coefficients coefficients of the estimated plane equation
    static void ExtractAndProjectFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr floor,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected,
                                       pcl::ModelCoefficients::Ptr coefficients);
    ///
    /// \param input_cloud input point cloud
    /// \param hull_cloud point cloud containing the points on the concave hull
    /// \param polygons the resultant concave hull polygons, as a set of vertices.
    /// The Vertices structure contains an array of point indices.
    /// \param chull PCL concave hull object
    static void CreateConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                  std::vector<pcl::Vertices> polygons, pcl::ConcaveHull<pcl::PointXYZ> chull);
    ///
    /// \param input_cloud input point cloud
    /// \param hull_cloud point cloud containing the points on the concave hull
    /// \param interior_boundaries point cloud containing the interior boundary points
    static void GetInteriorBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                                      pcl::PointCloud<pcl::Normal>::Ptr normals);
    ///
    /// \param interior_boundaries point cloud containing the interior boundary points
    /// \param holes vector containing the point clouds for the individual holes
    /// \param hole_sizes number of points in each hole point cloud
    static void GetHoleClouds(std::vector<Hole> &holes, pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                              const float n_search_radius, pcl::PointCloud<pcl::Normal>::Ptr boundary_normals, const float angle_thresh);
    ///
    /// \param holes vector containing the point clouds for the individual holes
    /// \param min_size minimum number of points in hole cloud required
    /// \param centers vector of points representing hole centers
    static void CalcHoleCenters(std::vector<Hole> &holes);

    static void CalcAreaScore(std::vector<Hole> &holes, pcl::ConvexHull<pcl::PointXYZ> cvxhull);

    /// Creates a Point Cloud from the image points vector
    /// \param cloud
    /// \param img_pts
    static void CreatePointCloudFromImgPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                           std::vector<cv::Point> img_pts,
                                           const float img_resolution);

    /// Draws a line between all points in the cloud
    /// \param cloud
    /// \param viewer
    static void DrawLinesInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 const pcl::visualization::PCLVisualizer::Ptr viewer);

    static void DrawGazesInCloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& trajectories,
                                 const std::vector<std::vector<Eigen::Vector3f>>& gazes,
                                 const pcl::visualization::PCLVisualizer::Ptr viewer);

    /// Computes the rigid transformation from the points in the floorplan and the points in the cloud and
    /// then aplies the transform to the floorplan cloud
    /// \param cloud_in The cloud containing the vertices of the floorplan
    /// \param cloud The cloud of the projected floor_
    /// \param points The points from the floor_ (NOT floorplan) cloud
    /// \param max_iteration Number of max ransac iterations
    /// \param max_angle Max angle for random transformation
    /// \param max_translation Max Translation for random transformation
    static void TransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    std::vector<Eigen::Vector3f> points,
                                    const int max_iteration,
                                    const int max_angle,
                                    const int max_translation);
    /// Takes the vertices of the floorplan as input and creates a dense pointcloud with points along the walls of the map
    /// \param floorplan The cloud containing the vertex points of the floorplan
    /// \param dense_cloud The cloud where a dense set of points are added along the walls
    /// \param z The height of the floor_ in the HoloLens coordinate frame
    static void DenseFloorplanCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& floorplan,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& dense_cloud,
                                    pcl::ModelCoefficients::Ptr coefficients);

    /// Adds all the points from the source_cloud to cloud.
    /// \param source_cloud
    /// \param cloud
    static void CombinePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /// Creates mesh using poisson reconstruction
    /// \param cloud
    /// \param mesh
    static void ConstructMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh & mesh,
                              const double normal_search_radius, const int poisson_depth);

    static void Calc2DNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float search_radius);

    static void CalcPoses(std::vector<Hole> &holes);

    static void Grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    static void CreateGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud,
                           GazeScores &gaze_scores);

    static bool CalculateNextGridPoint(const Eigen::Vector3f& gaze,
                                       GazeScores gaze_scores,
                                       pcl::PointXYZ curr_point,
                                       std::vector<Eigen::Vector3i>& visited,
                                       pcl::PointXYZ& next_point,
                                       float step_size=0.01);

    static float CalculateScoreFromDistance(pcl::PointXYZ grid_point, pcl::PointXYZ gaze_point);

    static void CalcGazeScores(GazeScores &gaze_scores,
                               std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> trajectories,
                               std::vector<std::vector<Eigen::Vector3f>> gazes);
};
#endif //HOLEDET_HOLEDET_UTILS_H

