// created by Maurice Brunner 23.03

#include <iostream>
#include <vector>
#include <thread>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
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

// constants
bool VISUALS = true;
const  float LEAFX = 0.1;
const float LEAFY = 0.1;
const float LEAFZ = 0.1;
const int RADREMRADIUS = 1;
const int RADREMMINPTS = 5;
const float IMGRESOLUTION = 0.0694; // [m/px]

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
    Eigen::Vector3f floorplan[3];
    Eigen::Vector3f cloud[3];
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
    MouseParams* mp = (MouseParams*)param;
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
        Eigen::Vector3f point (x, y, z);
        switch(tp->cnt) {
            case 0:
                tp->floorplan[0] = point;
                break;
            case 1:
                tp->floorplan[1] = point;
                break;
            case 2:
                tp->floorplan[2] = point;
                break;
            case 3:
                tp->cloud[0] = point;
                break;
            case 4:
                tp->cloud[1] = point;
                break;
            case 5:
                tp->cloud[2] = point;
                break;
            default:
                cout << "All points clicked" << endl;
        }
        tp->cnt++;
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
void ExtractFloor(PointCloud<PointXYZ>::Ptr cloud,
                  PointCloud<PointXYZ>::Ptr floor,
                  PointCloud<PointXYZ>::Ptr floor_projected) {
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
}


/// Creates a Point Cloud from the image points vector
/// \param cloud
/// \param img_pts
void CreatePointCloudFromImgPts(PointCloud<PointXYZ>::Ptr cloud, vector<Point> img_pts) {
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
void DrawLinesInCloud(PointCloud<PointXYZ>::Ptr cloud, visualization::PCLVisualizer::Ptr viewer) {
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

void CalculateTransform(vector<Eigen::Vector3f> floorplan, vector<Eigen::Vector3f> cloud) {
    Eigen::Vector3f cog_floorplan;
    cog_floorplan = 1 / 3 * (floorplan[0] + floorplan[1] + floorplan[2]);

    Eigen::Vector3f cog_cloud;
    cog_cloud = 1 / 3 * (cloud[0] + cloud[1] + cloud[2]);

    for(int i = 0; i < 3; i++) {
        
    }


}

int main() {

    string dataset = "/home/maurice/ETH/HoleDet/prototype/data/hololens.pcd";
    string floorplan_path = "/home/maurice/ETH/HoleDet/prototype/data/floorplan.jpg";


//region Floorplan
    Mat floorplan = imread(floorplan_path);
    MouseParams mp;
    mp.img = floorplan;
    namedWindow("floorplan", 0);
    setMouseCallback("floorplan", onMouse, (void*)&mp);
    while(true) {
        imshow("floorplan", floorplan);
        if(waitKey(10) == 27) {
            destroyAllWindows();
            break;
        }
    }

    CreatePointCloudFromImgPts(floorplan_, mp.points);
//endregion

//region Cloud
    /* VISUALS */
    TransformPoints tp;
    visualization::PCLVisualizer::Ptr viewer (new visualization::PCLVisualizer("Cloud Viewer"));
    viewer->addCoordinateSystem(2.0);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&tp);


    /* READ FILE */
    if (io::loadPCDFile<PointXYZ> (dataset, *cloud_) == -1) {
        PCL_ERROR ("Couldn't read file\n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud_->width * cloud_->height
              << " data points"
              << std::endl;


    // adding floorplan cloud to viewer
    viewer->addPointCloud(floorplan_,"floorplan");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.0f, 0.5f, "floorplan");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floorplan");

    /* PREPROCESSING */
    FilterPointCLoud(cloud_, LEAFX, LEAFY, LEAFZ, RADREMRADIUS, RADREMMINPTS);
    std::cout << "Points after first filtering:\t" << cloud_->width << std::endl;

    ExtractFloor(cloud_, floor_, floor_projected_);

    // Create a Concave Hull representation of the projected inliers
    PointCloud<PointXYZ>::Ptr convex_hull (new PointCloud<PointXYZ>);
    std::vector<Vertices> polygons;
    ConcaveHull<PointXYZ> chull;
    chull.setInputCloud (floor_projected_);
    chull.setAlpha (1);
    chull.reconstruct (*convex_hull, polygons);

    int max_num_of_points = 0;
    int idx_polygon;
    // get index of largest polygon
    for(int i = 0; i < polygons.size(); i++) {
        Vertices vertices = polygons[i];
        int num_of_points = vertices.vertices.size();
        if(num_of_points > max_num_of_points) {
            max_num_of_points = num_of_points;
            idx_polygon = i;
        }
    }


    for(int idx : polygons[idx_polygon].vertices) {
        PointXYZ pt = convex_hull->points[idx];
        cloud_hull_->push_back(pt);
    }

    DrawLinesInCloud(floorplan_, viewer);

    //VISUALS
    if (VISUALS) {
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(10ms);
            if(tp.cnt == 5) {
                break;
            }
        }
    }



    //endregion
    return 0;
}
