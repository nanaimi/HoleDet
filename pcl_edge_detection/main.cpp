//
// Created by hanlonm on 21.03.22.
//

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <math.h>
#include <string>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ml/kmeans.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/centroid.h>
#include <numeric>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>


void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    std::cout << "Picking event active" << std::endl;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << x << "; " << y << "; " << z << std::endl;
    }
}
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    pcl::visualization::PCLVisualizer::Ptr event_viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (event_viewer->contains("cloud")) {
            cout << "c was pressed => removing preprocessed PointCloud" << endl;
            event_viewer->removePointCloud("cloud");
        } else {
            cout << "c was pressed => showing preprocessed PointCloud" << endl;
            event_viewer->addPointCloud(cloud,"cloud");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
            event_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"cloud");
        }
    }
}

// Function for calculating
// the median
double findMedian(std::vector<int> a,
                  int n)
{

    // If size of the arr[] is even
    if (n % 2 == 0) {

        // Applying nth_element
        // on n/2th index
        nth_element(a.begin(),
                    a.begin() + n / 2,
                    a.end());

        // Applying nth_element
        // on (n-1)/2 th index
        nth_element(a.begin(),
                    a.begin() + (n - 1) / 2,
                    a.end());

        // Find the average of value at
        // index N/2 and (N-1)/2
        return (double)(a[(n - 1) / 2]
                        + a[n / 2])
               / 2.0;
    }

        // If size of the arr[] is odd
    else {

        // Applying nth_element
        // on n/2
        nth_element(a.begin(),
                    a.begin() + n / 2,
                    a.end());

        // Value at index (N/2)th
        // is the median
        return (double)a[n / 2];
    }
}



int main (int argc, char** argv)
{
    using namespace std::chrono_literals;

    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    //reader.read ("../data/table_scene_lms400.pcd", *cloud);
//    reader.read ("/home/hanlonm/HoleDet/Data/projected.pcd", *cloud);
    reader.read ("/home/hanlonm/HoleDet/Data/hololens.pcd", *cloud);
    reader.read ("/home/hanlonm/HoleDet/Data/ba_keyframes.pcd", *trajectory);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // Filter out conference room
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    float conf_min_x = -10;
    float conf_max_x = 10;
    float conf_min_y = -10;
    float conf_max_y = 10;
    boxFilter.setMin(Eigen::Vector4f(conf_min_x, conf_min_y, -5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(conf_max_x, conf_max_y, 5, 1.0));

//    boxFilter.setMin(Eigen::Vector4f(-30, 10, -5, 1.0));
//    boxFilter.setMax(Eigen::Vector4f(6, -40, 5, 1.0));
    boxFilter.setInputCloud(cloud);
//    boxFilter.filter(*cloud);
    boxFilter.setInputCloud(trajectory);
//    boxFilter.filter(*trajectory);



    // radius outlier removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud);

//    viewer->addPointCloud(cloud,"cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");

    // Voxel filter
    std::cerr << *cloud << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setLeafSize (0.1, 0.1, 0.1);
//    sor.filter (* cloud);




    // extract floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setMaxIterations(10000);
    // Segment dominant plane
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *floor);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud (floor);
    proj.setModelCoefficients (coefficients);
    proj.filter (*floor_projected);


//    pcl::PointCloud<pcl::PointXYZ>::Ptr wall (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::SACSegmentation<pcl::PointXYZ> wall_seg;
//    wall_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//    wall_seg.setModelType(pcl::SAC_RANSAC);
//    wall_seg.setOptimizeCoefficients(true);
//    wall_seg.setDistanceThreshold(0.1);
//    wall_seg.setMaxIterations(10000);
//    wall_seg.setInputCloud(cloud);
//    wall_seg.segment(*inliers, *coefficients);
//    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *wall);
//    viewer->addPointCloud(wall, "wall");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "wall");

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> polygons;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (floor_projected);
    chull.setAlpha (1);
    chull.reconstruct (*cloud_hull,polygons);
    viewer->addPointCloud(cloud_hull,"hull");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"hull");

    // boundary
    // fill in the cloud data here
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // estimate normals and fill in \a normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (floor_projected);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.3);
    ne.compute (*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (floor_projected);
    est.setInputNormals (normals);
    est.setRadiusSearch (0.5);   // 50cm radius
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute (boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i <floor_projected->points.size(); ++i) {
        if(boundaries[i].boundary_point> 0) {
            floor_boundary->push_back(floor_projected->points[i]);
        }
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr interior_boundaries(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto point: *floor_boundary) {
        float min_dist = 100;
        for (int i = 0; i < cloud_hull->size(); ++i) {
            float dist = pcl::euclideanDistance(point, cloud_hull->points[i]);
            if (dist<min_dist){
                min_dist = dist;
            }
        }
        if(min_dist>0.5){
         interior_boundaries->push_back(point);
        }
    }

    ////// KMEANS //////////
    pcl::Kmeans kmeans(interior_boundaries->size(),3);
    kmeans.setClusterSize(100);
    for (size_t i = 0; i < interior_boundaries->points.size(); i++)
    {
        std::vector<float> data(3);
        data[0] = interior_boundaries->points[i].x;
        data[1] = interior_boundaries->points[i].y;
        data[2] = interior_boundaries->points[i].z;
        kmeans.addDataPoint(data);
    }
    kmeans.kMeans();
    auto centroids = kmeans.get_centroids();


//    std::cout << interior_boundaries;
//    viewer->addPointCloud(interior_boundaries,"bound");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "bound");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"bound");




    viewer->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

//    viewer->addCoordinateSystem(2.0);



//    viewer->addPointCloud(trajectory,"trajectory");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "trajectory");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"trajectory");


//    for (int i = 0; i<centroids.size(); i++)
//    {
//        pcl::PointXYZ center;
//        center.x =centroids[i][0];
//        center.y =centroids[i][1];
//        center.z =centroids[i][2];
//        if (isinf(center.x)) continue;
//        auto name = "center_" + std::to_string(i);
//        viewer->addSphere(center,0.1,1.0,1.0,0.0, name);
//        std::cout << center;
//    }


    /////////////REGION GROWING SEG/////////

//    pcl::search::Search<pcl::PointXYZ>::Ptr reg_tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::PointCloud <pcl::Normal>::Ptr reg_normals (new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod (reg_tree);
//    normal_estimator.setInputCloud (cloud);
//    normal_estimator.setKSearch (50);
//    normal_estimator.compute (*reg_normals);
//
//    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//    reg.setMinClusterSize (50);
//    reg.setMaxClusterSize (10000);
//    reg.setSearchMethod (reg_tree);
//    reg.setNumberOfNeighbours (50);
//    reg.setInputCloud (cloud);
//    //reg.setIndices (indices);
//    reg.setInputNormals (reg_normals);
//    reg.setSmoothnessThreshold (20.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold (1.0);
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer cluster_viewer ("Cluster viewer");
//    cluster_viewer.showCloud(colored_cloud);
//    while (!cluster_viewer.wasStopped ())
//    {
//    }
    /////// MESH ////////////
    // Normal estimation*
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr mesh_normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr mesh_tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    mesh_tree->setInputCloud (floor_projected);
//    n.setInputCloud (floor_projected);
//    n.setSearchMethod (mesh_tree);
//    n.setRadiusSearch (1);
//    n.compute (*mesh_normals);
//    //* normals should not contain the point normals + surface curvatures
//
//    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*floor_projected, *mesh_normals, *cloud_with_normals);
//    //* cloud_with_normals = cloud + normals
//
//    // Create search tree*
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud (cloud_with_normals);
//
//    // Initialize objects
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//
//    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (2);
//
//    // Set typical values for the parameters
//    gp3.setMu (2.5);
//    gp3.setMaximumNearestNeighbors (100);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);
//
//    // Get result
//    gp3.setInputCloud (cloud_with_normals);
//    gp3.setSearchMethod (tree2);
//    gp3.reconstruct (triangles);

//    viewer->addPolygonMesh(triangles);

//
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (interior_boundaries);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> holes;
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
    std::vector<int> hole_sizes;
    for (auto hole: holes) {
        hole_sizes.push_back(hole->points.size());

    }
    double sum = std::accumulate(hole_sizes.begin(), hole_sizes.end(), 0.0);
    double mean = sum / hole_sizes.size();

    double sq_sum = std::inner_product(hole_sizes.begin(), hole_sizes.end(), hole_sizes.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / hole_sizes.size() - mean * mean);
    double med = findMedian(hole_sizes, hole_sizes.size());
    for (int i = 0; i < holes.size(); ++i) {
//        if (holes[i]->points.size()<mean){continue;}
        if (holes[i]->points.size()<10){continue;}
        auto name = "hole_" + std::to_string(i);
        float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        viewer->addPointCloud(holes[i],name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r1, 1-r1, r3, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,name);

        Eigen::Matrix<float, 4, 1> hole_center;
        pcl::compute3DCentroid(*holes[i],hole_center);
        pcl::PointXYZ center;
        center.x =hole_center.x();
        center.y =hole_center.y();
        center.z =hole_center.z();
        auto center_name = "hole_center_" + std::to_string(i);
        viewer->addSphere(center,0.3,1.0,1.0,0.0, center_name);


    }

//    viewer->addPointCloud(hole,"hole");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "hole");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"hole");


//    viewer->setBackgroundColor (1, 1, 1);
    viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}