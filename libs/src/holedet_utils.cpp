//
// Created by hanlonm on 06.04.22.
//

#include "../include/holedet_utils.h"
pcl::PointCloud<pcl::PointXYZ>::Ptr Utils::readCloud(const std::basic_string<char> &file_name, pcl::PCDReader &reader) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(file_name, *cloud);
    return cloud;
}

void Utils::extractAndProjectFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr floor,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected,
                                              pcl::ModelCoefficients::Ptr coefficients) {
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(10000);

    // segment floor
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *floor);

    // extract floor from input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(floor);
    proj.setModelCoefficients(coefficients);
    proj.filter(*floor_projected);

}

void Utils::createConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                         std::vector<pcl::Vertices> polygons,
                                         pcl::ConcaveHull<pcl::PointXYZ> chull) {
    chull.setInputCloud(input_cloud);
    chull.setAlpha(1);
    chull.reconstruct(*hull_cloud, polygons);
}

void Utils::getInteriorBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // estimate normals and fill in \a normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.3);
    ne.compute(*normals);

    // extract boundary points
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud(input_cloud);
    est.setInputNormals(normals);
    est.setRadiusSearch(0.5);   // 50cm radius
    est.setSearchMethod(typename pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute(boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < input_cloud->points.size(); ++i) {
        if (boundaries[i].boundary_point > 0) {
            boundary_points->push_back(input_cloud->points[i]);
        }
    }

    // filter out points that are close to the concave hull
    for (auto point: *boundary_points) {
        float min_dist = 100;
        for (int i = 0; i < hull_cloud->size(); ++i) {
            float dist = pcl::euclideanDistance(point, hull_cloud->points[i]);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist > 0.5) {
            interior_boundaries->push_back(point);
        }
    }
}

void Utils::getHoleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                                     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> holes,
                                     std::vector<int> hole_sizes) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(interior_boundaries);

    std::vector<int> visited;
    int point_idx;
    float search_radius = 0.6;

    for (int i = 0; i < interior_boundaries->points.size(); ++i) {
        int start_point = i;
        if (std::count(visited.begin(), visited.end(), i)) { continue; }

        pcl::PointXYZ search_point = interior_boundaries->points[start_point];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        visited.push_back(start_point);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hole(new pcl::PointCloud<pcl::PointXYZ>);
        hole->push_back(interior_boundaries->points[start_point]);

        std::deque<int> to_visit;
        do {
            kdtree.radiusSearch(search_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            for (auto p: pointIdxRadiusSearch) {
                if (!std::count(visited.begin(), visited.end(), p) &&
                    std::find(to_visit.begin(), to_visit.end(), p) == to_visit.end()) {
                    if (p < interior_boundaries->points.size() && p > 0) {
                        to_visit.push_back(p);
                    }
                }
            }
            point_idx = to_visit.front();
            to_visit.pop_front();
            if (point_idx < interior_boundaries->points.size() && point_idx > 0) {
                hole->push_back(interior_boundaries->points[point_idx]);
                search_point = interior_boundaries->points[point_idx];
                visited.push_back(point_idx);
            } else { break; }

        } while (!to_visit.empty());
        holes.push_back(hole);
    }

    for (auto hole: holes) {
        hole_sizes.push_back(hole->points.size());
    }

}

void Utils::calcHoleCenters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> holes,
                                       const int &min_size, std::vector<pcl::PointXYZ> centers) {
    for (int i = 0; i < holes.size(); ++i) {
        if (holes[i]->points.size() < min_size) { continue; }

        Eigen::Matrix<float, 4, 1> hole_center;
        pcl::compute3DCentroid(*holes[i], hole_center);
        pcl::PointXYZ center(hole_center.x(), hole_center.y(), hole_center.z());

        centers.push_back(center);
    }
}




