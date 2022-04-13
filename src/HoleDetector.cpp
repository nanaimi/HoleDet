//
// Created by hanlonm on 06.04.22.
//

#include "../include/HoleDetector.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud(const std::basic_string<char> &file_name, pcl::PCDReader &reader){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(file_name, *cloud);
    return cloud;
}

void extractAndProjectFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &floor, pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected, pcl::ModelCoefficients::Ptr coefficients){
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setMaxIterations(10000);

    // segment floor
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *floor);

    // extract floor from input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (floor);
    proj.setModelCoefficients (coefficients);
    proj.filter (*floor_projected);

}

void createConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud, std::vector<pcl::Vertices> polygons, pcl::ConcaveHull<pcl::PointXYZ> chull){
    chull.setInputCloud (input_cloud);
    chull.setAlpha (1);
    chull.reconstruct (*hull_cloud, polygons);
}

void getInteriorBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries){
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // estimate normals and fill in \a normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.3);
    ne.compute (*normals);

    // extract boundary points
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (input_cloud);
    est.setInputNormals (normals);
    est.setRadiusSearch (0.5);   // 50cm radius
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute (boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i <input_cloud->points.size(); ++i) {
        if(boundaries[i].boundary_point> 0) {
            boundary_points->push_back(input_cloud->points[i]);
        }
    }

    // filter out points that are close to the concave hull
    for (auto point: *boundary_points) {
        float min_dist = 100;
        for (int i = 0; i < hull_cloud->size(); ++i) {
            float dist = pcl::euclideanDistance(point, hull_cloud->points[i]);
            if (dist<min_dist){
                min_dist = dist;
            }
        }
        if(min_dist>0.5){
            interior_boundaries->push_back(point);
        }
    }
}

void getHoleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &holes, std::vector<int> &hole_sizes){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (interior_boundaries);

    std::vector<int> visited;
    int point_idx;
    float search_radius = 0.6;

    for (int i = 0; i < interior_boundaries->points.size(); ++i) {
        int start_point = i;
        if (std::count(visited.begin(), visited.end(), i)){continue;}

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
                    if(p<interior_boundaries->points.size() && p > 0){
                        to_visit.push_back(p);
                    }
                }
            }
            point_idx = to_visit.front();
            to_visit.pop_front();
            if(point_idx<interior_boundaries->points.size() && point_idx > 0){
                hole->push_back(interior_boundaries->points[point_idx]);
                search_point = interior_boundaries->points[point_idx];
                visited.push_back(point_idx);
            } else{break;}

        } while (!to_visit.empty());
        holes.push_back(hole);
    }

    for (auto hole: holes) {
        hole_sizes.push_back(hole->points.size());
    }

}

void calcHoleCenters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> holes, const int &min_size, std::vector<pcl::PointXYZ> &centers){
    for (int i = 0; i < holes.size(); ++i) {

        Eigen::Matrix<float, 4, 1> hole_center;
        pcl::compute3DCentroid(*holes[i],hole_center);
        pcl::PointXYZ center(hole_center.x(),hole_center.y(),hole_center.z());

        centers.push_back(center);
    }
}

HoleDetector::HoleDetector(const std::basic_string<char> &file_name):
raw_cloud(new pcl::PointCloud<pcl::PointXYZ>),
filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>),
floor(new pcl::PointCloud<pcl::PointXYZ>),
floor_projected(new pcl::PointCloud<pcl::PointXYZ>),
hull_cloud(new pcl::PointCloud<pcl::PointXYZ>),
interior_boundaries(new pcl::PointCloud<pcl::PointXYZ>),
viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")),
floor_coefficients (new pcl::ModelCoefficients)
{
    pointcloud_file = file_name;
    min_size = 50;
}

void HoleDetector::init_filters() {
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);

    voxel_filter.setMinimumPointsNumberPerVoxel(2);
    voxel_filter.setLeafSize (0.1, 0.1, 0.1);
}

void HoleDetector::pre_process() {
    outrem.setInputCloud(raw_cloud);
    outrem.filter (*filtered_cloud);

//    voxel_filter.setInputCloud (filtered_cloud);
//    voxel_filter.filter(*filtered_cloud);

}

void HoleDetector::detectHoles() {
    init_filters();
    raw_cloud = readCloud(pointcloud_file, reader);
    pre_process();
    extractAndProjectFloor(filtered_cloud, floor, floor_projected, floor_coefficients);
    createConcaveHull(floor_projected, hull_cloud, hull_polygons, chull);
    getInteriorBoundaries(floor_projected, hull_cloud, interior_boundaries);
    getHoleClouds(interior_boundaries, holes, hole_sizes);
    calcHoleCenters(holes, min_size, centers);

}

void HoleDetector::visualize() {
    viewer->addPointCloud(hull_cloud,"hull");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"hull");

    viewer ->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

    for (int i = 0; i < holes.size(); ++i) {
        if (holes[i]->points.size() < min_size) { continue; }
        auto name = "hole_" + std::to_string(i);
        float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        viewer->addPointCloud(holes[i], name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r1, 1 - r1, r3, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

        auto center_name = "hole_center_" + std::to_string(i);
        viewer->addSphere(centers[i],0.3,r1,1 - r1,r3, center_name);


    }
//    viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
//    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    using namespace std::chrono_literals;
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(10ms);
    }

}

void HoleDetector::pp_callback(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
    std::cout << "Picking event active" << std::endl;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << x << "; " << y << "; " << z << std::endl;
    }
}

void HoleDetector::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    pcl::visualization::PCLVisualizer::Ptr event_viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (event_viewer->contains("cloud")) {
            cout << "c was pressed => removing preprocessed PointCloud" << endl;
            event_viewer->removePointCloud("cloud");
        } else {
            cout << "c was pressed => showing preprocessed PointCloud" << endl;
            event_viewer->addPointCloud(filtered_cloud,"cloud");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
            event_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"cloud");
        }
    }
}