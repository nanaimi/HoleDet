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
#include <pcl/filters/passthrough.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/features/normal_3d_omp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>

#include <eigen3/Eigen/Eigenvalues>
#include <Eigen/Dense>

#include "Normal2dEstimation.h"

struct Hole {
    pcl::PointXYZ centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    int size;
    std::vector<Eigen::Affine3f> poses;
    float area;
    float score;
    Eigen::Matrix3f cov_matrix;
};

struct GazeScores {
    Eigen::MatrixXf scores;
    Eigen::MatrixXf angle_scores[4]; //0:0 1:90 2:180, 3:270
    Eigen::MatrixXf occupancy_grid;
    int offset_x;
    int offset_y;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
};

class Utils {
public:
    /// Takes a filename of a pcd file and fills the point cloud
    /// \param file_name string, e.g "hololens.pcd"
    /// \param reader pcl::PCDReader reader_
    /// \return shared pointer pcl::PointCloud<pcl::PointXYZ>::Ptr
    static pcl::PointCloud<pcl::PointXYZ>::Ptr ReadCloud(const std::basic_string<char> &file_name,
                                                         pcl::PCDReader &reader);

    /// Creates the trajectory and gaze variables from the files
    /// \param traj_file_name The filepath of the trajectroy file
    /// \param gaze_file_name The filepath of the gaze file
    /// \param lenghts_file_name The filepath of the keyframes length
    /// \param reader The pcd Reader
    /// \param trajectories The trajectory variable
    /// \param gazes The gaze variable
    static void ReadTrajectoriesAndGaze(const std::basic_string<char> &traj_file_name,
                                        const std::basic_string<char> &gaze_file_name,
                                        const std::basic_string<char> &lenghts_file_name,
                                        pcl::PCDReader &reader,
                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &trajectories,
                                        std::vector<std::vector<Eigen::Vector3f>> &gazes);


    /// Fits a floor plane to the pointcloud using RANSAC and then projects it
    /// \param cloud input point cloud
    /// \param floor cloud to which unprojected floor_ will be extracted
    /// \param floor_projected floor_ cloud projected to the estimated plane equation
    /// \param coefficients coefficients of the estimated plane equation
    static void ExtractAndProjectFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr floor,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected,
                                       pcl::ModelCoefficients::Ptr coefficients);

    /// Creates the concave hull of a cloud
    /// \param input_cloud input point cloud
    /// \param hull_cloud point cloud containing the points on the concave hull
    /// \param polygons the resultant concave hull polygons, as a set of vertices.
    /// \param chull PCL concave hull object
    static void CreateConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                  std::vector<pcl::Vertices> polygons, pcl::ConcaveHull<pcl::PointXYZ> chull);

    /// Calculates the boundary points of a cloud and filters them using the hull
    /// \param input_cloud input point cloud
    /// \param hull_cloud point cloud containing the points on the concave hull
    /// \param interior_boundaries point cloud containing the interior boundary points
    static void GetInteriorBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                                      pcl::PointCloud<pcl::Normal>::Ptr normals);

    /// Clusters the boundary points into hole clouds
    /// \param interior_boundaries point cloud containing the interior boundary points
    /// \param holes vector containing the point clouds for the individual holes
    /// \param hole_sizes number of points in each hole point cloud
    static void GetHoleClouds(std::vector<Hole> &holes, pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                              const float n_search_radius, pcl::PointCloud<pcl::Normal>::Ptr boundary_normals,
                              const float angle_thresh);

    /// Calculates the centroid of every hole cloud
    /// \param holes vector containing the point clouds for the individual holes
    /// \param min_size minimum number of points in hole cloud required
    /// \param centers vector of points representing hole centers
    static void CalcHoleCenters(std::vector<Hole> &holes);

    /// Calculates the score of each hole based on the area median
    /// \param holes vector containing the point clouds for the individual holes
    /// \param min_size minimum number of points in hole cloud required
    /// \param centers vector of points representing hole centers
    static void CalcAreaScore(std::vector<Hole> &holes, pcl::ConvexHull<pcl::PointXYZ> cvxhull);

    /// Creates a Point Cloud from the image points vector
    /// \param cloud
    /// \param img_pts
    static void CreatePointCloudFromImgPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                           std::vector<cv::Point> img_pts,
                                           const float img_resolution);

    /// For future work
    /// \param trajectories
    /// \param gazes
    /// \param viewer
    static void DrawGazesInCloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &trajectories,
                                 const std::vector<std::vector<Eigen::Vector3f>> &gazes,
                                 const pcl::visualization::PCLVisualizer::Ptr viewer);

    /// Computes the rigid transformation from the points in the floorplan and the points in the cloud and
    /// then applies the transform to the floor plan cloud
    /// \param cloud_in; The cloud containing the vertices of the floorplan
    /// \param cloud; The cloud of the projected floor_
    /// \param points; The points from the floor_ (NOT floorplan) cloud
    /// \param max_iteration; Number of max ransac iterations
    /// \param max_angle; Max angle for random transformation
    /// \param max_translation; Max Translation for random transformation
    static void TransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                    std::vector<Eigen::Vector3f> points,
                                    const int max_iteration,
                                    const int max_angle,
                                    const int max_translation);

    /// Takes the vertices of the floor plan as input and creates a dense pointcloud with points along the walls of the map
    /// \param floorplan;   The cloud containing the vertex points of the floorplan
    /// \param dense_cloud; The cloud where a dense set of points are added along the walls
    /// \param z;           The height of the floor_ in the HoloLens coordinate frame
    static void DenseFloorplanCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &floorplan,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud,
                                    pcl::ModelCoefficients::Ptr coefficients);

    /// Creates the point cloud necessary for mesh reconstruction
    /// \param filtered_cloud; The preprocessed point cloud
    /// \param floor_projected; the point cloud projected on the floor
    /// \param dense_floorplan_outline; boundary points of floor plan
    /// \param cloud; output cloud
    static void CreateFloorplanFiltered(const pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr dense_floorplan_outline,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// Creates mesh using poisson reconstruction
    /// \param cloud; input point cloud from which to reconstruct mesh
    /// \param mesh; triangle mesh created
    static void ConstructMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              pcl::PolygonMesh::Ptr mesh,
                              const double normal_search_radius,
                              const int poisson_depth);

    /// Calculates the 2d normals using pca
    /// \param cloud The input cloud
    /// \param normals The 3d normals
    /// \param search_radius the search radius for neighbours
    static void Calc2DNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                              float search_radius);

    /// Calculates the poses used to resample the hole
    /// \param holes The holes
    /// \param floor_projected The floor cloud
    static void CalcPoses(std::vector<Hole> &holes, pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected);

    /// Creates an occupancy grid from the floorplan cloud stored in the gaze scores variable
    /// \param dense_cloud The floorplan cloud
    /// \param gaze_scores The gaze scores variable
    static void CreateGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud,
                           GazeScores &gaze_scores);

    /// Calculates the next coordinate in the grid by ray marching
    /// \param gaze The ray direction
    /// \param gaze_scores The gaze scores variable
    /// \param curr_point The point where the ray starts
    /// \param visited All visited grid coordinates
    /// \param next_point The next grid coordinate
    /// \param step_size The step size of the ray marching
    /// \return A bool that is true if a point has been found
    static bool CalculateNextGridPoint(const Eigen::Vector3f &gaze,
                                       GazeScores gaze_scores,
                                       pcl::PointXYZ curr_point,
                                       std::vector<Eigen::Vector3i> &visited,
                                       pcl::PointXYZ &next_point,
                                       float step_size = 0.01);

    /// Implements the distance scoring function
    /// \param grid_point The next calculated point
    /// \param gaze_point the point where the gaze ray starts
    /// \return The score
    static float CalculateScoreFromDistance(pcl::PointXYZ grid_point, pcl::PointXYZ gaze_point);

    /// Main function to create the heat maps
    /// \param gaze_scores gaze scores variable
    /// \param trajectories the trajectory variable
    /// \param gazes the gaze variable
    /// \param num_of_angles num of angles used to sample 140 degrees
    static void CalcHeatMaps(GazeScores &gaze_scores,
                             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> trajectories,
                             std::vector<std::vector<Eigen::Vector3f>> gazes,
                             int num_of_angles = 7);

    /// Get Covariance matrix from point cloud
    /// \param crop_cloud; the 2D point cloud that defines the hole
    /// \param cloud;      the complete preprocessed point cloud
    /// \param cov_matrix; resulting covariance matrix of the isolated point cloud encompassed by the hole
    static bool GetHoleCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        Eigen::Matrix3f &cov_matrix);

    /// Scores all holes based on the information contained in the vertical above the hole
    /// \param holes;      list of all candidate holes
    /// \param cloud;      the complete preprocessed point cloud
    /// \param vert_score_threshold; Threshold for discrete scoring of vertical information contained in holes
    static void ScoreVertical(std::vector<Hole> &holes,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              const float vert_score_threshold);

    /// Calculates the score of each hole based on the median gaze patch
    /// \param holes The holes
    /// \param gaze_scores Gaze scores variable
    /// \param patch_size size of the patch considered for the score
    static void CalcGazeScores(std::vector<Hole> &holes, GazeScores gaze_scores, int patch_size = 2);

    /// Generates output files
    /// \param holes The holes
    static void SaveResults(std::vector<Hole> &holes, std::basic_string<char> path, double min_score);
};

#endif //HOLEDET_HOLEDET_UTILS_H

