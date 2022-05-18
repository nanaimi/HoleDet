//
// Created by hanlonm on 06.04.22.
//

#include "holedet_utils.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr Utils::ReadCloud(const std::basic_string<char> &file_name, pcl::PCDReader &reader) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(file_name, *cloud);
    return cloud;
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
        holes[i].area = areas[i];
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

void Utils::CalcPoses(std::vector<Hole> &holes, pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected) {
    float step_back; //Describes wanted distance between pose and hole boundary
    float filter_size; //sqrt of hole area, size of box filter to determine boundary normal direction

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_front(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_back(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passY;
    passY.setFilterFieldName ("y");
    pcl::PassThrough<pcl::PointXYZ> passFront;
    passFront.setFilterFieldName ("x");
    pcl::PassThrough<pcl::PointXYZ> passBack;
    passBack.setFilterFieldName ("x");

    for (int i = 0; i < holes.size(); ++i) {
        //Get boundary normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        Calc2DNormals(holes[i].points, normals, 0.6);

        //Define filter size for normal direction defintion
        filter_size = sqrt(holes[i].area);
        //std::cout << "Filter size of hole " << i << ": " << filter_size << "\n";
        step_back = 0.3;

        passY.setFilterLimits (-filter_size, filter_size);
        passFront.setFilterLimits (0, 2*filter_size);
        passBack.setFilterLimits (-2*filter_size,0);

        pcl::PointXYZ keypoint = holes[i].points->points[0]; //Temp variable for keypoints
        for (int j = 0; j < normals->size(); ++j){
            pcl::PointXYZ boundary_point = holes[i].points->points[j];

            //Handle keypoint choice
            bool skip = true;
            if(j==0 || pcl::euclideanDistance(boundary_point, keypoint) > step_back){
                skip = false;
                keypoint = boundary_point;
            }

            //Skip if not keypoint
            if(!skip) {
                //Get translation
                Eigen::Affine3f pose = Eigen::Affine3f::Identity();
                pose.translation() = boundary_point.getVector3fMap();

                //Rotate towards normal
                Eigen::Vector3f dir(normals->points[j].normal_x, normals->points[j].normal_y,
                                    normals->points[j].normal_z);
                pose.rotate(Eigen::AngleAxisf(atan2(dir(1), dir(0)), Eigen::Vector3f(0, 0, 1)));
                pose.rotate(Eigen::AngleAxisf(atan2(dir(2), sqrt(dir(0) * dir(0) + dir(1) * dir(1))),Eigen::Vector3f(0, 1, 0)));

                ///Handle wrong direction normals
                pcl::transformPointCloud(*floor_projected, *points_front, pose.inverse(), true); //Transform points to current pose
                pcl::transformPointCloud(*floor_projected, *points_back, pose.inverse(), true);

                //pass.setFilterLimitsNegative (true);
                passY.setInputCloud (points_front);
                passY.filter(*points_front);
                passY.filter(*points_back);

                //pass.setFilterLimitsNegative (true);
                passFront.setInputCloud (points_front);
                passFront.filter(*points_front);

                //pass.setFilterLimitsNegative (true);
                passBack.setInputCloud (points_back);
                passBack.filter(*points_back);

                //Switch dirention if nessecary
                if(points_front->size() > points_back->size()) {
                    pose.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1)));
                    //std::cout << "Rotated pose!\n";
                    holes[i].rotateds.push_back(true);
                }else{
                    holes[i].rotateds.push_back(false);
                }

                //Take a step back
                pose.translate(Eigen::Vector3f(-step_back,0,0));

                holes[i].poses.push_back(pose);
            }
        }

    }
}