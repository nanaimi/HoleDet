//
// Created by hanlonm on 06.04.22.
//

#include "holedet_utils.h"
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

void Utils::getHoleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries, const float n_search_radius,
                                     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes,
                                     std::vector<int> &hole_sizes) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(interior_boundaries);

    std::vector<int> visited;
    int point_idx;

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
            kdtree.radiusSearch(search_point, n_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

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

void Utils::calcHoleCenters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes,
                                       const int &min_size, std::vector<pcl::PointXYZ> &centers) {
    for (int i = 0; i < holes.size(); ++i) {

        Eigen::Matrix<float, 4, 1> hole_center;
        pcl::compute3DCentroid(*holes[i], hole_center);
        pcl::PointXYZ center(hole_center.x(), hole_center.y(), hole_center.z());

        centers.push_back(center);
    }
}

void Utils::createPointCloudFromImgPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                       std::vector<cv::Point> img_pts,
                                       const float img_resolution) {
    pcl::PointXYZ last_pt (0, 0, 0);
    cloud->push_back(last_pt);
    for(int i = 1; i < img_pts.size(); i++) {
        float x_diff = static_cast<float>(img_pts[i].x - img_pts[i - 1].x) * img_resolution;
        float y_diff = static_cast<float>(img_pts[i].y - img_pts[i - 1].y) * img_resolution;
        float x = last_pt.x + x_diff;
        float y = last_pt.y + y_diff;
        last_pt  = pcl::PointXYZ(x, y, 0);
        cloud->push_back(last_pt);
    }
}

void Utils::drawLinesInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 const pcl::visualization::PCLVisualizer::Ptr viewer) {
    bool first = true;
    int cnt =  0;
    pcl::PointXYZ last_point = cloud->points[0];
    for (auto point : *cloud) {
        if (first) {
            first = false;
            continue;
        }
        viewer->addLine(point, last_point, 1.0, 0.0, 0.0, "line" + std::to_string(cnt));
        last_point = point;
        cnt++;
    }
    viewer->addLine(cloud->points[0], last_point, 1.0, 0.0, 0.0, "line" + std::to_string(cnt));
}

void Utils::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                std::vector<Eigen::Vector3f> points,
                                const int max_iteration,
                                const int max_angle,
                                const int max_translation) {
    const uint n = cloud_in->width;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::MatrixXf floor(3, n);
    Eigen::MatrixXf end(3, n);

    for (int i = 0; i < n; i++) {
        floor.col(i) = cloud_in->points[i].getVector3fMap();
        end.col(i) = points[i];
    }

    Eigen::Matrix4f trans = Eigen::umeyama(floor, end, true);
    Eigen::Affine3f t(trans);
    pcl::transformPointCloud(*cloud_in, *cloud_out, t, false);

    // Project the floor and the floorplan to create a true 2d problem z = 0
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    coefficients->values.resize (4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 1;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_out);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_out);

    proj.setInputCloud (cloud);
    proj.filter (*cloud);

    srand(static_cast<unsigned int>(std::time(nullptr)));
    Eigen::Vector3f axis (coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    pcl::Vertices::Ptr verticesPtr (new pcl::Vertices);
    std::vector<uint> vert_vec(n);
    iota(vert_vec.begin(), vert_vec.end(), 0);
    verticesPtr->vertices = vert_vec;
    std::vector<pcl::Vertices> polygon;
    polygon.push_back(*verticesPtr);

    pcl::CropHull<pcl::PointXYZ> crop;
    crop.setHullIndices(polygon);
    crop.setDim(2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    crop.setHullCloud(cloud_out);
    crop.setInputCloud(cloud);
    crop.filter(*cropped_cloud);
    int max_inlier = static_cast<int>(cropped_cloud->width);
    Eigen::Affine3f best_transform = t;

    cout << "Number of inliers before ransacing:\t" << max_inlier << endl;

    for(int it = 0; it < max_iteration; it++) {
        Eigen::Affine3f rand_t;
        float angle = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0) * max_angle / 180.0 * M_PI;
        float x = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0) * max_translation;
        float y = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0) * max_translation;

        rand_t = Eigen::AngleAxis<float>(angle, axis);
        rand_t.translation() = Eigen::Vector3f(x, y, 0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr rand_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_out, *rand_cloud, rand_t);

        crop.setHullCloud(rand_cloud);
        crop.filter(*cropped_cloud2);

        int inliers = cropped_cloud2->width;
        if(inliers > max_inlier) {
            max_inlier = inliers;
            cout << it << ":\t" << max_inlier << endl;
            best_transform = rand_t;
        }
    }

    pcl::transformPointCloud(*cloud_out, *cloud_in, best_transform);
}

void Utils::calcHoleAreas(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes,
                          std::vector<double> &hole_areas, pcl::ConvexHull<pcl::PointXYZ> cvxhull, pcl::PointCloud<pcl::PointXYZ>::Ptr hole_hull_cloud) {
    double area;
    for (int i = 0; i < holes.size(); ++i) {
        if(holes[i]->size() < 3){
            area = 0;
        }
        else {
            cvxhull.setComputeAreaVolume(true);
            cvxhull.setInputCloud(holes[i]);
            cvxhull.reconstruct(*hole_hull_cloud);

            area = cvxhull.getTotalArea();
        }

        hole_areas.push_back(area);
    }

}

void Utils::denseFloorplanCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &floorplan,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud, pcl::ModelCoefficients::Ptr coefficients) {
    bool first = true;
    pcl::PointXYZ last_point = floorplan->points[0];
    for (auto point : *floorplan) {
        if (first) {
            first = false;
            continue;
        }
        // do stuff
        float dx = last_point.x - point.x;
        float dy = last_point.y - point.y;

        for (float i = 0.0; i <= 1; i+=0.001) {
            pcl::PointXYZ new_point;
            new_point.x = point.x + i * dx;
            new_point.y = point.y + i * dy;
            new_point.z = point.z;
            dense_cloud->points.push_back(new_point);
        }



        last_point = point;
    }
    pcl::PointXYZ point = floorplan->points[0];
    for (float i = 0.0; i <= 1; i+=0.001) {
        float dx = last_point.x - point.x;
        float dy = last_point.y - point.y;
        pcl::PointXYZ new_point;
        new_point.x = point.x + i * dx;
        new_point.y = point.y + i * dy;
        new_point.z = point.z;
        dense_cloud->points.push_back(new_point);
    }
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(dense_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*dense_cloud);

}

void Utils::combinePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    for (auto point:source_cloud->points) {
        cloud->points.push_back(point);
    }
}



