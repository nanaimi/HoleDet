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

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Utils {
    public:
    ///
    /// \param file_name string, e.g "hololens.pcd"
    /// \param reader pcl::PCDReader reader
    /// \return shared pointer pcl::PointCloud<pcl::PointXYZ>::Ptr
    static pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud(const std::basic_string<char> &file_name, pcl::PCDReader &reader);
    ///
    /// \param cloud input point cloud
    /// \param floor cloud to which unprojected floor will be extracted
    /// \param floor_projected floor cloud projected to the estimated plane equation
    /// \param coefficients coefficients of the estimated plane equation
    static void extractAndProjectFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr floor, pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected, pcl::ModelCoefficients::Ptr coefficients);
    ///
    /// \param input_cloud input point cloud
    /// \param hull_cloud point cloud containing the points on the concave hull
    /// \param polygons the resultant concave hull polygons, as a set of vertices. The Vertices structure contains an array of point indices.
    /// \param chull PCL concave hull object
    static void createConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud, std::vector<pcl::Vertices> polygons, pcl::ConcaveHull<pcl::PointXYZ> chull);
    ///
    /// \param input_cloud input point cloud
    /// \param hull_cloud point cloud containing the points on the concave hull
    /// \param interior_boundaries point cloud containing the interior boundary points
    static void getInteriorBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries);
    ///
    /// \param interior_boundaries point cloud containing the interior boundary points
    /// \param holes vector containing the point clouds for the individual holes
    /// \param hole_sizes number of points in each hole point cloud
    static void getHoleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries, const float n_search_radius, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes, std::vector<int> &hole_sizes);
    ///
    /// \param holes vector containing the point clouds for the individual holes
    /// \param min_size minimum number of points in hole cloud required
    /// \param centers vector of points representing hole centers
    static void calcHoleCenters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes, const int &min_size, std::vector<pcl::PointXYZ> &centers);

    static void calcHoleAreas(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes, std::vector<double> &hole_areas, pcl::ConvexHull<pcl::PointXYZ> cvxhull, pcl::PointCloud<pcl::PointXYZ>::Ptr hole_hull_cloud);

    static void calcPoses(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes, std::vector<pcl::PointXYZ> &centers, std::vector<Eigen::Affine3f> &poses);

    /// Creates a Point Cloud from the image points vector
    /// \param cloud
    /// \param img_pts
    static void createPointCloudFromImgPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                           std::vector<cv::Point> img_pts,
                                           const float img_resolution);

    /// Draws a line between all points in the cloud
    /// \param cloud
    /// \param viewer
    static void drawLinesInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 const pcl::visualization::PCLVisualizer::Ptr viewer);

    /// Computes the rigid transformation from the points in the floorplan and the points in the cloud and then aplies the transform to the floorplan cloud
    /// \param cloud_in The cloud containing the vertices of the floorplan
    /// \param cloud The cloud of the projected floor
    /// \param points The points from the floor (NOT floorplan) cloud
    /// \param max_iteration Number of max ransac iterations
    /// \param max_angle Max angle for random transformation
    /// \param max_translation Max Translation for random transformation
    static void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    std::vector<Eigen::Vector3f> points,
                                    const int max_iteration,
                                    const int max_angle,
                                    const int max_translation);
};
#endif //HOLEDET_HOLEDET_UTILS_H

