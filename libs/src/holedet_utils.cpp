//
// Created by hanlonm on 06.04.22.
//

#include "holedet_utils.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr Utils::ReadCloud(const std::basic_string<char> &file_name, pcl::PCDReader &reader) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(file_name, *cloud);
    return cloud;
}

void Utils::ReadTrajectoriesAndGaze(const std::basic_string<char> &traj_file_name,
                                    const std::basic_string<char> &gaze_file_name,
                                    const std::basic_string<char> &lenghts_file_name,
                                    pcl::PCDReader &reader,
                                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& trajectories,
                                    std::vector<std::vector<Eigen::Vector3f>>& gazes) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory = Utils::ReadCloud(traj_file_name, reader);

    int max = 0;
    int min = 0;
    std::ifstream length_file(lenghts_file_name);
    std::ifstream gaze_file(gaze_file_name);
    std::string gaze;
    std::string length;
    std::vector<Eigen::Vector3f> all_gazes;

    // decypher .obj file
    while(std::getline(gaze_file, gaze)) {
        if(gaze[0] == 'v') {
            std::stringstream line(gaze.erase(0,2));
            std::string segment;
            std::vector<std::string> values;
            while(std::getline(line, segment, ' ')) {
                values.push_back(segment);
            }
            Eigen::Vector3f gaze_vec(stof(values[0]), stof(values[1]), stof(values[2]));
            all_gazes.push_back(gaze_vec);
        }
    }

    while(std::getline(length_file, length)) {
        max += stoi(length);
        pcl::PointCloud<pcl::PointXYZ>::Ptr traj(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<Eigen::Vector3f> curr_gazes;
        for(int i = min; i < max; i++) {
            traj->points.push_back(trajectory->points[i]);
            curr_gazes.push_back(all_gazes[i]);
        }
        min = max;
        trajectories.push_back(traj);
        gazes.push_back(curr_gazes);
    }
}

void Utils::ExtractAndProjectFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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

    // segment floor_
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *floor);

    // extract floor_ from input cloud
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

void Utils::CreateConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                         std::vector<pcl::Vertices> polygons,
                                         pcl::ConcaveHull<pcl::PointXYZ> chull) {
    chull.setInputCloud(input_cloud);
    chull.setAlpha(1);
    chull.reconstruct(*hull_cloud, polygons);
}

void Utils::GetInteriorBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                                             pcl::PointCloud<pcl::Normal>::Ptr normals) {
    // estimate normals and fill in \a normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius 30cm
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

void Utils::GetHoleClouds(std::vector<Hole> &holes, pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries,
                          const float n_search_radius, pcl::PointCloud<pcl::Normal>::Ptr boundary_normals, const float angle_thresh) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(interior_boundaries);

    std::vector<int> visited;
    int point_idx;

    for (int i = 0; i < interior_boundaries->points.size(); ++i) {
        int start_point = i;
        if (std::count(visited.begin(), visited.end(), i)) { continue; }

        Hole hole;
        pcl::PointXYZ search_point = interior_boundaries->points[start_point];
        pcl::Normal search_normal = boundary_normals->points[start_point];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        visited.push_back(start_point);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        hole_cloud->push_back(interior_boundaries->points[start_point]);

        std::deque<int> to_visit;
        do {
            kdtree.radiusSearch(search_point, n_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            // for all points within the search radius
            for (auto p: pointIdxRadiusSearch) {
                // check if the point has already been visited and if it has already been added to the queue
                if (!std::count(visited.begin(), visited.end(), p) &&
                    std::find(to_visit.begin(), to_visit.end(), p) == to_visit.end()) {
                    // check if the point has a valid index
                    if (p < interior_boundaries->points.size() && p > 0) {
                        // add point to queue
                        pcl::Normal p_normal = boundary_normals->points[p];
                        float dot_product = search_normal.normal_x * p_normal.normal_x + search_normal.normal_y * p_normal.normal_y;
                        if (dot_product<0){
                            p_normal.normal_x = p_normal.normal_x * (-1);
                            p_normal.normal_y = p_normal.normal_y * (-1);
                        }
                        dot_product = search_normal.normal_x * p_normal.normal_x + search_normal.normal_y * p_normal.normal_y;
                        float mag = sqrt(pow(search_normal.normal_x, 2) + pow(search_normal.normal_y, 2)) * sqrt(pow(p_normal.normal_x, 2) + pow(p_normal.normal_y, 2));
                        float angle = acos(dot_product / mag);
                        if (angle<angle_thresh) {
                            to_visit.push_back(p);
                        }
                    }
                }
            }
            point_idx = to_visit.front();
            to_visit.pop_front();
            if (point_idx < interior_boundaries->points.size() && point_idx > 0) {
                hole_cloud->push_back(interior_boundaries->points[point_idx]);
                search_point = interior_boundaries->points[point_idx];
                search_normal = boundary_normals->points[point_idx];
                visited.push_back(point_idx);
            } else { break; }

        } while (!to_visit.empty());
        hole.points = hole_cloud;
        hole.size = hole_cloud->points.size();
        if (hole.size>5) {
            holes.push_back(hole);
        }
    }
}

void Utils::CalcHoleCenters(std::vector<Hole> &holes) {
    for (auto& hole : holes) {
        Eigen::Matrix<float, 4, 1> hole_center;
        pcl::compute3DCentroid(*hole.points, hole_center);
        hole.centroid = pcl::PointXYZ(hole_center.x(), hole_center.y(), hole_center.z());
    }
}

void Utils::CreatePointCloudFromImgPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
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

void Utils::DrawLinesInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
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

void Utils::DrawGazesInCloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& trajectories,
                             const std::vector<std::vector<Eigen::Vector3f>>& gazes,
                             const pcl::visualization::PCLVisualizer::Ptr viewer) {
    int n = trajectories.size();
    for(int i = 0; i < n; i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_traj = trajectories[i];
        std::vector<Eigen::Vector3f> curr_gazes = gazes[i];
        float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        std::string id_base = "arrow" + std::to_string(i);
        for(int it = 0; it < curr_gazes.size(); it++) {
            std::string id = id_base +  std::to_string(it);
            std::cout << id << "\n";
            pcl::PointXYZ p1 = curr_traj->points[it];
            pcl::PointXYZ p2(p1.x + curr_gazes[it].x(), p1.y + curr_gazes[it].y(), p1.z + curr_gazes[it].z());
            viewer->addArrow(p2, p1, r1, 1-r1, r3, false, id);
        }
    }
}

void Utils::TransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
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

    // Project the floor_ and the floorplan to create a true 2d problem z = 0
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

void Utils::CalcAreaScore(std::vector<Hole> &holes, pcl::ConvexHull<pcl::PointXYZ> cvxhull) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_reconstruct (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<float> areas;
    float max_area = 0;
    for (auto& hole : holes) {
        if (hole.points->size() < 3) {
            hole.score = 0;
            areas.push_back(0.0);
        }
        else {
            cvxhull.setComputeAreaVolume(true);
            cvxhull.setInputCloud(hole.points);
            cvxhull.reconstruct(*convex_hull_reconstruct);
            float area = cvxhull.getTotalArea();
            areas.push_back(area);
            if (area > max_area) {
                max_area = area;
            }
        }
    }
    for(int i = 0; i < holes.size(); i++) {
        if(holes[i].score == 0) {
            continue;
        }
        holes[i].score -= (1 - areas[i] / max_area);
    }
}

void Utils::DenseFloorplanCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &floorplan,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud,
                                  pcl::ModelCoefficients::Ptr coefficients) {
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

void Utils::CombinePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    for (auto point:source_cloud->points) {
        cloud->points.push_back(point);
    }
}

void Utils::ConstructMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          pcl::PolygonMesh & mesh,
                          const double normal_search_radius,
                          const int poisson_depth) {
    /* normal estimation using OMP */
    Eigen::Vector4f centroid;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimate;
    normal_estimate.setNumberOfThreads(8);
    normal_estimate.setInputCloud(cloud);
    normal_estimate.setRadiusSearch(normal_search_radius);
    pcl::compute3DCentroid(*cloud, centroid);
    normal_estimate.setViewPoint(centroid[0], centroid[1], centroid[2]);
    normal_estimate.compute(*cloud_normals);

    // reverse normals' direction
//    #pragma omp parallel for
    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    /* End normal estimation */

    // combine points and normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

    /* poisson reconstruction */
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(poisson_depth);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(mesh);
    /* end poisson reconstruction */
}

void Utils::Calc2DNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float search_radius) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    Normal2dEstimation norm_estim;
    norm_estim.setInputCloud(cloud);
    norm_estim.setSearchMethod (tree);

    norm_estim.setRadiusSearch (search_radius);

    norm_estim.compute(normals);
}

void Utils::CalcPoses(std::vector<Hole> &holes) {
    for (int i = 0; i < holes.size(); ++i) {

        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        Eigen::Vector3f centroid_vec = holes[i].centroid.getVector3fMap();
        Eigen::Vector3f point_vec = holes[i].points->points[0].getVector3fMap();

        Eigen::Vector3f vec = centroid_vec - point_vec;

        pose.translation() = point_vec;
        pose.rotate(Eigen::AngleAxisf(atan2(vec(1),vec(0)), Eigen::Vector3f(0,0,1)));
        pose.rotate(Eigen::AngleAxisf(atan2(vec(2),sqrt(vec(0)*vec(0)+vec(1)*vec(1))), Eigen::Vector3f(0,1,0)));

        holes[i].poses = pose;
    }
}

void Utils::Grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> grid;
//    grid.setInputCloud(cloud);
    grid.setLeafSize(0.1f, 0.1f, 0.1f);
    auto coords = grid.getGridCoordinates(0,0,0);
    coords = grid.getGridCoordinates(1,3,0);
    coords = grid.getGridCoordinates(1,2,0);
}

void Utils::CreateGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud, pcl::VoxelGrid<pcl::PointXYZ> grid, Eigen::MatrixXf &grid_matrix) {
    for (auto point : dense_cloud->points) {
        auto coords = grid.getGridCoordinates(point.x, point.y, point.z);
        grid_matrix(coords.x()+500, coords.y()+500) = 1;
    }
}

bool Utils::CalculateNextGridPoint(const Eigen::Vector3f& gaze, const pcl::VoxelGrid<pcl::PointXYZ>& grid,
                                              pcl::PointXYZ curr_point,
                                              std::vector<Eigen::Vector3i>& visited,
                                              pcl::PointXYZ& next_point,
                                              Eigen::MatrixXf grid_matrix,
                                              int offset, float step_size) {
    for(int i = 0; i < 10 / step_size; i++) {
        float x = curr_point.x + i * step_size * gaze.x();
        float y = curr_point.y + i * step_size * gaze.y();
        Eigen::Vector3i next_coord = grid.getGridCoordinates(x, y, curr_point.z);
        bool in_grid = grid_matrix(next_coord.x() + offset, next_coord.y() + offset);
        bool not_visited = std::count(visited.begin(), visited.end(), next_coord) == 0;
        if (not_visited && in_grid) {
            next_point = pcl::PointXYZ(x, y, curr_point.z);
            visited.push_back(next_coord);
            return true;
        }
    }
    return false;
}

float Utils::CalculateScoreFromDistance(pcl::PointXYZ grid_point, pcl::PointXYZ gaze_point){
    float dist = std::sqrt(std::pow(grid_point.x - gaze_point.x, 2) +
                                std::pow(grid_point.y - gaze_point.y, 2));
    return dist > 10 ? 0 : std::exp(-0.43 * dist);
}