//
// Created by hanlonm on 06.04.22.
//

#include "HoleDetector.h"

using namespace pcl;
using namespace Eigen;
using namespace std;
using namespace std::chrono_literals;

HoleDetector::HoleDetector(const basic_string<char> &file_name, const std::basic_string<char> &floorplan_path):
        raw_cloud(new PointCloud<PointXYZ>),
        filtered_cloud(new PointCloud<PointXYZ>),
        floor(new PointCloud<PointXYZ>),
        floor_projected_(new PointCloud<PointXYZ>),
        hull_cloud(new PointCloud<PointXYZ>),
        hole_hull_cloud(new PointCloud<pcl::PointXYZ>),
        interior_boundaries(new PointCloud<PointXYZ>),
        floorplan_(new PointCloud<PointXYZ>),
        viewer (new visualization::PCLVisualizer ("3D Viewer")),
        floor_coefficients (new ModelCoefficients)

{
    pointcloud_file_ = file_name;
    floorplan_file_ = floorplan_path;
    min_size = 50;
    tp_.cnt = 0;
    min_size = 0.2;
    boundary_search_radius = 0.6;
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
    raw_cloud = Utils::readCloud(pointcloud_file_, reader);
    pre_process();
    Utils::extractAndProjectFloor(filtered_cloud, floor, floor_projected_, floor_coefficients);
    Utils::createConcaveHull(floor_projected_, hull_cloud, hull_polygons, chull);
    Utils::getInteriorBoundaries(floor_projected_, hull_cloud, interior_boundaries);
    calculate();

}

void HoleDetector::calculate() {
    hole_areas.clear();
    centers.clear();
    hole_sizes.clear();
    holes.clear();
    Utils::getHoleClouds(interior_boundaries, boundary_search_radius, holes, hole_sizes);
    Utils::calcHoleCenters(holes, min_size, centers);
    Utils::calcHoleAreas(holes, hole_areas, cvxhull, hole_hull_cloud);

}

void HoleDetector::visualize() {
    viewer->addPointCloud(hull_cloud,"hull");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                              0.0f, 1.0f, 1.0f, "hull");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"hull");

    viewer ->addPointCloud(floor,"floor");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                              0.0f, 0.2f, 0.8f, "floor");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floor");

    viewer->addPointCloud(floorplan_, "floorplan");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                              0.5f, 0.0f, 0.5f, "floorplan");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"floorplan");

    Utils::drawLinesInCloud(floorplan_, viewer);

    for (int i = 0; i < holes.size(); ++i) {
        if (hole_areas[i] < min_size) { continue; }
        auto name = "hole_" + std::to_string(i);
        float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        viewer->addPointCloud(holes[i], name);
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, r1, 1 - r1, r3, name);
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

        auto center_name = "hole_center_" + std::to_string(i);
        viewer->addSphere(centers[i],0.3,r1,1 - r1,r3, center_name);

        //Handle poses visualisation
        pcl::PointXYZ p(poses[i].translation().x(), poses[i].translation().y(), poses[i].translation().z());
        viewer->addSphere(p,0.1,r1,1 - r1,r3, center_name + "_pose"); //add colour sphere to pose
        viewer->addCoordinateSystem(0.5, poses[i]); //display pose

    }

    viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
    viewer->registerKeyboardCallback (&HoleDetector::keyboardEventOccurred, *this, (void*)&viewer);
    using namespace std::chrono_literals;
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        this_thread::sleep_for(10ms);
    }

}

void HoleDetector::getFloorplanCloud(bool debug, string floorplan_path) {
    if(debug) {
        if (io::loadPCDFile<PointXYZ>(floorplan_path, *floorplan_) == -1) {
            PCL_ERROR ("Couldn't read file\n");
        }
    } else {
        mp_.img = cv::imread(floorplan_file_);
        cv::namedWindow("floorplan", 0);
        cv::setMouseCallback("floorplan", onMouse, (void*) &mp_);
        while(true) {
            cv::imshow("floorplan", mp_.img);
            if (cv::waitKey(10) == 27) {
                Utils::createPointCloudFromImgPts(floorplan_, mp_.points, kImgResolution_);
                cv::destroyAllWindows();
                break;
            }
        }

        visualization::PCLVisualizer::Ptr pp_viewer(new visualization::PCLVisualizer("Cloud Viewer"));
        pp_viewer->registerPointPickingCallback(point_picker_cb, (void*) &tp_);

        pp_viewer->addPointCloud(floor_projected_, "floor_projected_");
        pp_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR,
                                                    0.0f, 0.2f, 0.8f, "floor_projected_");
        pp_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    8, "floor_projected_");
        while (!pp_viewer->wasStopped()) {
            pp_viewer->spinOnce(100);
            this_thread::sleep_for(10ms);
            if (tp_.cnt > floorplan_->width) {
                pp_viewer->close();
                break;
            }
        }
        Utils::transformPointCloud(floorplan_, floor_projected_, tp_.points,
                                   kMaxIteration_, kMaxAngle_, kMaxTranslation_);
    }


}

void HoleDetector::setBoundarySearchRadius(const float value) {
    boundary_search_radius = value;
}

void HoleDetector::pp_callback(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
    std::cout << "Picking event active" << std::endl;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        cout << x << "; " << y << "; " << z << endl;
    }
}

void HoleDetector::keyboardEventOccurred(const visualization::KeyboardEvent &event, void *viewer_void) {
    visualization::PCLVisualizer::Ptr event_viewer = *static_cast<visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (event_viewer->contains("cloud")) {
            cout << "c was pressed => removing preprocessed PointCloud" << endl;
            event_viewer->removePointCloud("cloud");
        } else {
            cout << "c was pressed => showing preprocessed PointCloud" << endl;
            event_viewer->addPointCloud(filtered_cloud,"cloud");
            event_viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"cloud");
        }
    }

    if (event.getKeySym () == "i" && event.keyDown ()){
        event_viewer->removeAllShapes();
        event_viewer->removeAllPointClouds();
        boundary_search_radius += 0.1;
        cout << "i was pressed => increasing the boundary point search radius by 0.1" << endl;
        calculate();
        visualize();

    }
    if (event.getKeySym () == "r" && event.keyDown ()){
        event_viewer->removeAllShapes();
        event_viewer->removeAllPointClouds();
        boundary_search_radius -= 0.1;
        cout << "i was pressed => reducing the boundary point search radius by 0.1" << endl;
        calculate();
        visualize();

    }
}

void HoleDetector::point_picker_cb(const visualization::PointPickingEvent& event, void* param) {
    cout << "Picking points" << endl;
    auto* tp_ptr = (TransformPoints*) param;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        Vector3f point (x, y, z);
        tp_ptr->points.push_back(point);
        tp_ptr->cnt++;
        cout << tp_ptr->cnt << endl;
    }
}

void HoleDetector::onMouse(int event, int x, int y, int flags, void* param) {
    if(event != cv::EVENT_LBUTTONDOWN) {
        return;
    }
    auto* mp_ptr = (MouseParams*)param;
    cv::Mat & img = mp_ptr->img;
    cv::Point point (x, y);
    mp_ptr->points.push_back(point);
    circle(img, point, 5, cv::Scalar(255, 0, 0), -1);
}
