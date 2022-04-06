// created by Maurice Brunner 23.03

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/boundary.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ml/kmeans.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


using namespace std::chrono_literals;
using namespace pcl;
using namespace std;
using namespace cv;
using namespace Eigen;

// constants
bool VISUALS = true;
bool DEBUG = true;
const  float LEAFX = 0.1;
const float LEAFY = 0.1;
const float LEAFZ = 0.1;
const int RADREMRADIUS = 1;
const int RADREMMINPTS = 5;
const float IMGRESOLUTION = 0.0694; // [m/px]
const float MAXTRANS = 5;
const float MAXANGLE = 10;

// MEMBERS
PointCloud<PointXYZ>::Ptr cloud_ (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr floor_projected_ (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr floor_ (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloud_hull_ (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr floorplan_ (new PointCloud<PointXYZ>);

struct MouseParams {
    Mat img;
    vector<Point> points;
};

struct TransformPoints {
    vector<Vector3f> points;
    int cnt = 0;
};


/// Callback for cv image viewer used to draw points in image and extracting floorplan
/// \param event The event
/// \param x X coordinate of the mouse
/// \param y Y Coordinate of the mouse
/// \param flags -
/// \param param struct with Image and point vector
void onMouse(int event, int x, int y, int flags, void* param) {
    if(event != EVENT_LBUTTONDOWN) {
        return;
    }
    auto* mp = (MouseParams*)param;
    Mat & img = mp->img;
    Point point (x, y);
    mp->points.push_back(point);
    circle(img, point, 5, Scalar(255, 0, 0), -1);
}


/// prints 3d coordinates of points selected
/// \param event
/// \param viewer_void
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* param)
{
    cout << "Picking points" << endl;
    auto* tp = (TransformPoints*)param;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        Vector3f point (x, y, z);
        tp->points.push_back(point);
        tp->cnt++;
        cout << tp->cnt << endl;
    }
}


/// Adds pointclouds to viewer depending on keystroke
/// \param event
/// \param viewer_void
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    visualization::PCLVisualizer::Ptr viewer = *static_cast<visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "r" && event.keyDown ())
    {

        if (viewer->contains("cloud")) {
            cout << "r was pressed => removing preprocessed PointCloud" << endl;
            viewer->removePointCloud("cloud");
        } else {
            cout << "r was pressed => showing preprocessed PointCloud" << endl;
            viewer->addPointCloud(cloud_,"cloud");
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "cloud");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");
        }

    } else if (event.getKeySym () == "o" && event.keyDown()) {

        if (viewer->contains("floor")) {
            cout << "o was pressed => Removing Floor" << endl;
            viewer->removePointCloud("floor");
        } else {
            cout << "o was pressed => Showing Floor" << endl;
            viewer->addPointCloud(floor_,"floor");
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

        }

    } else if (event.getKeySym () == "l" && event.keyDown()) {

        if (viewer->contains("floor_projected")) {
            cout << "l was pressed => Removing projected floor" << endl;
            viewer->removePointCloud("floor_projected");
        } else {
            cout << "l was pressed => Showing projected floor" << endl;
            viewer->addPointCloud(floor_projected_,"floor_projected");
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                                      0.0f, 0.0f, 1.0f, "floor_projected");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor_projected");
        }

    } else if (event.getKeySym () == "p" && event.keyDown()) {

        if (viewer->contains("floorplan")) {
            cout << "p was pressed => Removing floorplan" << endl;
            viewer->removePointCloud("floorplan");
        } else {
            cout << "p was pressed => Showing floorplan" << endl;
            viewer->addPointCloud(floorplan_,"floorplan");
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                    0.0f, 0.5f, 0.5f, "floorplan");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor_projected");
        }
    } else if (event.getKeySym () == "b" && event.keyDown()) {

        if (viewer->contains("hull")) {
            cout << "b was pressed => Removing boundary" << endl;
            viewer->removePointCloud("hull");
        } else {
            cout << "b was pressed => Showing boundary" << endl;
            viewer->addPointCloud(cloud_hull_,"hull");
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "hull");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"hull");
        }
    }
}

/// Filters the pointcloud with a radius outlier removal and creates a Voxel Grid
/// \param cloud The pointcloud
/// \param leaf_x Voxelgrid resolution in x-direction
/// \param leaf_y Voxelgrid resolution in y-direction
/// \param leaf_z Voxelgrid resolution in z-direction
/// \param radius Radius of outlier removal
/// \param min_points min points parameter of outlier removal
void FilterPointCLoud(PointCloud<PointXYZ>::Ptr cloud, float leaf_x, float leaf_y,
                      float leaf_z, int radius, int min_points) {
    // outlier removal
    RadiusOutlierRemoval<PointXYZ> outrem;

    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius (min_points);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud);

    VoxelGrid<PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leaf_x, leaf_y, leaf_z);
    filter.filter(*cloud);
}


/// Extracts the floor from a pointcloud and projects all inliers on the floor plane
/// \param cloud Input cloud
/// \param floor The cloud of the floor
/// \param floor_projected The cloud of all projected points
ModelCoefficients::Ptr ExtractFloor(const PointCloud<PointXYZ>::Ptr& cloud,
                  const PointCloud<PointXYZ>::Ptr& floor,
                  const PointCloud<PointXYZ>::Ptr& floor_projected,
                  const PointCloud<PointXYZ>::Ptr& cloud_hull) {
    // extract floor
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    // Create the segmentation object
    SACSegmentation<PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setMaxIterations(10000);
    // Segment dominant plane
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    copyPointCloud<PointXYZ>(*cloud, *inliers, *floor);

    ExtractIndices<PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud);

    // Project the model inliers
    ProjectInliers<PointXYZ> proj;
    proj.setModelType (SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud (floor);
    proj.setModelCoefficients (coefficients);
    proj.filter (*floor_projected);

    // Create a Concave Hull representation of the projected inliers
    ConcaveHull<PointXYZ> chull;
    vector<Vertices> polygons;
    PointCloud<PointXYZ>::Ptr convex_hull (new PointCloud<PointXYZ>);
    chull.setInputCloud(floor_projected);
    chull.setAlpha(1);
    chull.reconstruct(*convex_hull, polygons);

    int max_num_of_points = 0;
    int idx_polygon;
    // get index of largest polygon
    for (int i = 0; i < polygons.size(); i++) {
        Vertices vertices = polygons[i];
        int num_of_points = static_cast<int>(vertices.vertices.size());
        if (num_of_points > max_num_of_points) {
            max_num_of_points = num_of_points;
            idx_polygon = i;
        }
    }

    for (uint idx: polygons[idx_polygon].vertices) {
        PointXYZ pt = convex_hull->points[idx];
        cloud_hull->push_back(pt);
    }
    return coefficients;
}


/// Creates a Point Cloud from the image points vector
/// \param cloud
/// \param img_pts
void CreatePointCloudFromImgPts(const PointCloud<PointXYZ>::Ptr& cloud, vector<Point> img_pts) {
    PointXYZ last_pt (0, 0, 0);
    cloud->push_back(last_pt);
    for(int i = 1; i < img_pts.size(); i++) {
        float x_diff = static_cast<float>(img_pts[i].x - img_pts[i - 1].x) * IMGRESOLUTION;
        float y_diff = static_cast<float>(img_pts[i].y - img_pts[i - 1].y) * IMGRESOLUTION;
        float x = last_pt.x + x_diff;
        float y = last_pt.y + y_diff;
        last_pt  = PointXYZ(x, y, 0);
        cloud->push_back(last_pt);
    }
}


/// Draws a line between all points in the cloud
/// \param cloud
/// \param viewer
void DrawLinesInCloud(const PointCloud<PointXYZ>::Ptr& cloud, const visualization::PCLVisualizer::Ptr viewer) {
    bool first = true;
    int cnt =  0;
    PointXYZ last_point = cloud->points[0];
    for (auto point : *cloud) {
        if (first) {
            first = false;
            continue;
        }
        viewer->addLine(point, last_point, 1.0, 0.0, 0.0, "line" + to_string(cnt));
        last_point = point;
        cnt++;
    }
    viewer->addLine(cloud->points[0], last_point, 1.0, 0.0, 0.0, "line" + to_string(cnt));
}


///Computes the rigid transformation from the points in the floorplan and the points in the cloud and then aplies the transform to the floorplan cloud
/// \param cloud_in The cloud containing the vertices of the floorplan
/// \param cloud_out The transformed cloud of the floorplan
/// \param cloud
/// \param tp The points from the floor (NOT floorplan) cloud
/// \param max_iteration
void TransformPointCloud(const PointCloud<PointXYZ>::Ptr& cloud_in,
                         const PointCloud<PointXYZ>::Ptr& cloud_out,
                         const PointCloud<PointXYZ>::Ptr& cloud,
                         TransformPoints tp,
                         const int max_iteration) {
    const uint n = cloud_in->width;
    MatrixXf floor(3, n);
    MatrixXf end(3, n);

    for (int i = 0; i < n; i++) {
        floor.col(i) = cloud_in->points[i].getVector3fMap();
        end.col(i) = tp.points[i];
    }

    Eigen::Matrix4f trans = Eigen::umeyama(floor, end, true);
    Eigen::Affine3f t(trans);
    transformPointCloud(*cloud_in, *cloud_out, t, false);

    // Project the floor and the floorplan to create a true 2d problem z = 0
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
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
    Vector3f axis (coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Vertices::Ptr verticesPtr (new Vertices);
    vector<uint> vert_vec(n);
    iota(vert_vec.begin(), vert_vec.end(), 0);
    verticesPtr->vertices = vert_vec;
    vector<Vertices> polygon;
    polygon.push_back(*verticesPtr);

    CropHull<PointXYZ> crop;
    crop.setHullIndices(polygon);
    crop.setDim(2);

    PointCloud<PointXYZ>::Ptr cropped_cloud (new PointCloud<PointXYZ>);
    crop.setHullCloud(cloud_out);
    crop.setInputCloud(cloud);
    crop.filter(*cropped_cloud);
    int max_inlier = cropped_cloud->width;
    Eigen::Affine3f best_transform = t;

    cout << "Number of inliers before ransacing:\t" << max_inlier << endl;

    for(int it = 0; it < max_iteration; it++) {
        Eigen::Affine3f rand_t;
        float angle = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0) * MAXANGLE / 180.0 * M_PI;
        float x = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0) * MAXTRANS;
        float y = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0) * MAXTRANS;

        rand_t = AngleAxis<float>(angle, axis);
        rand_t.translation() = Vector3f(x, y, 0);

        PointCloud<PointXYZ>::Ptr rand_cloud (new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr cropped_cloud (new PointCloud<PointXYZ>);
        transformPointCloud(*cloud_out, *rand_cloud, rand_t);

        crop.setHullCloud(rand_cloud);
        crop.filter(*cropped_cloud);

        int inliers = cropped_cloud->width;
        if(inliers > max_inlier) {
            max_inlier = inliers;
            cout << it << ":\t" << max_inlier << endl;
            best_transform = rand_t;
        }
    }

    transformPointCloud(*cloud_out, *cloud_out, best_transform);


}

int main() {
    string base_path = "/home/maurice/ETH/HoleDet/prototype/data/";
    string dataset = base_path + "hololens.pcd";
    string floorplan_path = base_path + "floorplan.jpg";
    string floorplan_cloud = base_path + "floorplan.pcd";


//region Floorplan
    Mat floorplan = imread(floorplan_path);
    MouseParams mp;
    mp.img = floorplan;
    namedWindow("floorplan", 0);
    setMouseCallback("floorplan", onMouse, (void *) &mp);
    while (!DEBUG) {
        imshow("floorplan", floorplan);
        if (waitKey(10) == 27) {
            CreatePointCloudFromImgPts(floorplan_, mp.points);
            destroyAllWindows();
            break;
        }
    }
//endregion

//region Cloud
    /* VISUALS */
    TransformPoints tp;
    visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("Cloud Viewer"));
    //viewer->addCoordinateSystem(2.0);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) &viewer);
    viewer->registerPointPickingCallback(pp_callback, (void *) &tp);


    /* READ FILE */
    if (io::loadPCDFile<PointXYZ>(dataset, *cloud_) == -1) {
        PCL_ERROR ("Couldn't read file\n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud_->width * cloud_->height
              << " data points"
              << std::endl;


    // adding floorplan cloud to viewer
    //viewer->addPointCloud(floorplan_, "floorplan");
    //viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.0f, 0.5f, "floorplan");
    //viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floorplan");

    /* PREPROCESSING */
    FilterPointCLoud(cloud_, LEAFX, LEAFY, LEAFZ, RADREMRADIUS, RADREMMINPTS);
    std::cout << "Points after first filtering:\t" << cloud_->width << std::endl;
    PointCloud<PointXYZ>::Ptr convex_hull (new PointCloud<PointXYZ>);
    vector<Vertices> polygons;
    ModelCoefficients::Ptr coefficients = ExtractFloor(cloud_, floor_, floor_projected_, cloud_hull_);

    if(DEBUG) {
        /* LOAD FLOORPLAN PC FROM FILE */
        if (io::loadPCDFile<PointXYZ>(floorplan_cloud, *floorplan_) == -1) {
            PCL_ERROR ("Couldn't read file\n");
            return (-1);
        }
        std::cout << "Loaded floorplan cloud successfully" << std::endl;
    }

    // DrawLinesInCloud(floorplan_, viewer);
    // adding floor on zero z cloud to viewer
    viewer->addPointCloud(floor_projected_, "floor_projected");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor_projected");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 8, "floor_projected");


    //VISUALS
    if (VISUALS) {
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(10ms);
            if (tp.cnt > floorplan_->width) {
                break;
            }
        }
    }

    if(false) {
        tp.points.push_back(Vector3f(0.873, 3.945, -1.659));
        tp.points.push_back(Vector3f(-21.82 , -7.706, -1.676));
        tp.points.push_back(Vector3f(-29.849, -15.862, -1.686));
        tp.points.push_back(Vector3f(-6.039, -39.379, -1.707));
        tp.points.push_back(Vector3f(-2.963, -35.696, -1.702));
        tp.points.push_back(Vector3f(-6.352, -31.969, -1.699));
        tp.points.push_back(Vector3f(-1.835, -27.082, -1.693));
        tp.points.push_back(Vector3f(-0, -0, 0));
        tp.points.push_back(Vector3f(-9.614, -25.12, -1.692));
        tp.points.push_back(Vector3f(-18.149, -16.528, -1.685));
        tp.points.push_back(Vector3f(-15.862, -14.056, -1.682));
        tp.points.push_back(Vector3f(-10.258, -10.966, -1.678));
        tp.points.push_back(Vector3f(-8.472, -14.57, -1.681));
        tp.points.push_back(Vector3f(-4.688, -12.144, -1.678));
        tp.points.push_back(Vector3f(-6.861, -9.588, -1.675));
        tp.points.push_back(Vector3f(-0.471, -5.8, -1.67));
        tp.points.push_back(Vector3f(7.815, -0.623, -1.664));

    }

    PointCloud<PointXYZ>::Ptr transformed_floorplan (new PointCloud<PointXYZ>);
    TransformPointCloud(floorplan_, transformed_floorplan, floor_projected_,tp, 10000);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // adding floorplan cloud to viewer
    viewer->addPointCloud(transformed_floorplan, "floorplan");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.0f, 0.5f, "floorplan");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floorplan");
    DrawLinesInCloud(transformed_floorplan, viewer);

    // adding floor on zero z cloud to viewer
    viewer->addPointCloud(floor_projected_, "floor_projected");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.2f, 0.8f, "floor_projected");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floor_projected");
    //io::savePCDFileASCII(base_path + "floorplan_cloud.pcd", *transformed_floorplan);
    // VISUALS
    if (VISUALS) {
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(10ms);
        }
    }
    //endregion
    return 0;
}