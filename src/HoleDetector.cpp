//
// Created by hanlonm on 06.04.22.
//

#include "HoleDetector.h"

using namespace pcl;
using namespace Eigen;
using namespace std;
using namespace std::chrono_literals;

HoleDetector::HoleDetector(const basic_string<char> &path, const basic_string<char> &config_filename):
        raw_cloud_(new PointCloud<PointXYZ>),
        filtered_cloud_(new PointCloud<PointXYZ>),
        floor_(new PointCloud<PointXYZ>),
        floor_projected_(new PointCloud<PointXYZ>),
        hull_cloud_(new PointCloud<PointXYZ>),
        hole_hull_cloud_(new PointCloud<pcl::PointXYZ>),
        interior_boundaries_(new PointCloud<PointXYZ>),
        floorplan_(new PointCloud<PointXYZ>),
        dense_floorplan_(new PointCloud<PointXYZ>),
        viewer_ (new visualization::PCLVisualizer ("3D Viewer")),
        floor_coefficients_ (new ModelCoefficients)
{
    path_ = path;
    config_file_ = config_filename;
    ReadYAML();
    min_score_ = 4;
    tp_.cnt = 0;

    InitFilters();
    raw_cloud_ = Utils::ReadCloud(pointcloud_file_, reader);
    PreProcess();
    Utils::ExtractAndProjectFloor(filtered_cloud_, floor_, floor_projected_,
                                        floor_coefficients_);
}

void HoleDetector::ReadYAML() {
    try {
        YAML::Node config = YAML::LoadFile(config_file_);
        debug_ = config["debug"].as<bool>();
        std::cout << "loading" << std::endl;
        pointcloud_file_ = path_ + config["input"]["cloud_file"].as<std::string>();
        trajectory_file_ = path_ + config["input"]["trajectory_file"].as<std::string>();
        floorplan_file_ = path_ + config["input"]["floorplan_file"].as<std::string>();

        kStartScore_ = config["parameters"]["start_score"].as<float>();
        kPoissonDepth_ = config["parameters"]["poisson_depth"].as<int>();
        kNormalSearchRadius_ = config["parameters"]["normal_search_radius"].as<double>();

        kOutlierRadius_ = config["parameters"]["outlier_radius"].as<double>();
        kMinNeighbours_ = config["parameters"]["outlier_min_neighbours"].as<int>();

        kImgResolution_ = config["parameters"]["image_resolution"].as<double>();
        kMaxIteration_ = config["parameters"]["max_iteration"].as<int>();
        kMaxTranslation_ = config["parameters"]["max_translation"].as<int>();
        kMaxAngle_ = config["parameters"]["max_angle"].as<int>(); // [deg]

        kPassXLimMin_ = config["parameters"]["pass_xlim_min"].as<double>();
        kPassXLimMax_ = config["parameters"]["pass_xlim_max"].as<double>();
        kPassYLimMin_ = config["parameters"]["pass_ylim_min"].as<double>();
        kPassYLimMax_ = config["parameters"]["pass_ylim_max"].as<double>();

        boundary_search_radius_ = config["parameters"]["boundary_search_radius"].as<float>();

    } catch(const YAML::ParserException& ex) {
        std::cout << ex.what() << std::endl;
    }
}

void HoleDetector::InitFilters() {
    outrem_.setRadiusSearch(1);
    outrem_.setMinNeighborsInRadius (5);
    outrem_.setKeepOrganized(true);

    voxel_filter_.setMinimumPointsNumberPerVoxel(2);
    voxel_filter_.setLeafSize (0.1, 0.1, 0.1);
}

void HoleDetector::PreProcess() {
    outrem_.setInputCloud(raw_cloud_);
    outrem_.filter (*filtered_cloud_);

    // voxel_filter_.setInputCloud (filtered_cloud_);
    // voxel_filter_.filter(*filtered_cloud_);

}

void HoleDetector::DetectHoles() {
    Utils::DenseFloorplanCloud(floorplan_, dense_floorplan_, floor_coefficients_);
    Utils::CreateConcaveHull(floor_projected_, hull_cloud_, hull_polygons_, chull_);
    Utils::CombinePointClouds(hull_cloud_, dense_floorplan_);
    Utils::GetInteriorBoundaries(floor_projected_, dense_floorplan_, interior_boundaries_);
    CalculateCentroids();
}

void HoleDetector::CalculateCentroids() {
    Utils::GetHoleClouds(holes_, interior_boundaries_, boundary_search_radius_);
    Utils::CalcHoleCenters(holes_);
    std::cout << holes_.size() << "\n";
    for(auto& hole : holes_) {
        hole.score = kStartScore_;
    }
}

void HoleDetector::CalculateScores() {
    /* Calculate A score based on the Area */
    Utils::CalcAreaScore(holes_, cvxhull_);
    for(int i = 0; i < holes_.size(); i++) {
        float score = holes_[i].score;
        std::string str_nmb = to_string(i);
        if(str_nmb.length() < 2) {
            str_nmb = "0" + str_nmb;
        }
        cout << "Hole " << str_nmb << ":\tScore :" << to_string(score) << "\n";
    }
}

void HoleDetector::Visualize() {

    viewer_ ->addPointCloud(floor_, "floor_");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                               0.0f, 0.2f, 0.8f, "floor_");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floor_");

    viewer_->addPointCloud(floorplan_, "floorplan");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                               0.5f, 0.0f, 0.5f, "floorplan");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floorplan");

    viewer_ ->addPointCloud(dense_floorplan_, "dense_floor");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                               1.0f, 0.0f, 0.0f, "dense_floor");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dense_floor");

//    Utils::drawLinesInCloud(floorplan_, viewer_);

    for (int i = 0; i < holes_.size(); i++) {
        if (holes_[i].score < min_score_) { continue; }
        auto name = "hole_" + std::to_string(i);
        float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        viewer_->addPointCloud(holes_[i].points, name);
        viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, r1, 1 - r1, r3, name);
        viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

        auto center_name = "hole_center_" + std::to_string(i);
        viewer_->addSphere(holes_[i].centroid, 0.3, r1, 1 - r1, r3, center_name);


    }

    viewer_->registerPointPickingCallback(PpCallback, (void*)&viewer_);
    viewer_->registerKeyboardCallback (&HoleDetector::KeyboardEventOccurred, *this, (void*)&viewer_);
    using namespace std::chrono_literals;
    while (!viewer_->wasStopped ())
    {
        viewer_->spinOnce (100);
        this_thread::sleep_for(10ms);
    }

}

void HoleDetector::GetFloorplanCloud(bool debug, string floorplan_path) {
    if(debug) {
        if (io::loadPCDFile<PointXYZ>(floorplan_path, *floorplan_) == -1) {
            PCL_ERROR ("Couldn't read file\n");
        }
    } else {
        mp_.img = cv::imread(floorplan_file_);
        cv::namedWindow("floorplan", 0);
        cv::setMouseCallback("floorplan", OnMouse, (void*) &mp_);
        while(true) {
            cv::imshow("floorplan", mp_.img);
            if (cv::waitKey(10) == 27) {
                Utils::CreatePointCloudFromImgPts(floorplan_, mp_.points, kImgResolution_);
                cv::destroyAllWindows();
                break;
            }
        }

        visualization::PCLVisualizer::Ptr pp_viewer(new visualization::PCLVisualizer("Cloud Viewer"));
        pp_viewer->registerPointPickingCallback(PointPickerCb, (void*) &tp_);

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
        Utils::TransformPointCloud(floorplan_, floor_projected_, tp_.points,
                                   kMaxIteration_, kMaxAngle_, kMaxTranslation_);
    }


}

void HoleDetector::SetBoundarySearchRadius(const float value) {
    boundary_search_radius_ = value;
}

void HoleDetector::PpCallback(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
    std::cout << "Picking event active" << std::endl;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        cout << x << "; " << y << "; " << z << endl;
    }
}

void HoleDetector::KeyboardEventOccurred(const visualization::KeyboardEvent &event, void *viewer_void) {
    visualization::PCLVisualizer::Ptr event_viewer = *static_cast<visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
    {

        if (event_viewer->contains("cloud")) {
            cout << "c was pressed => removing preprocessed PointCloud" << endl;
            event_viewer->removePointCloud("cloud");
        } else {
            cout << "c was pressed => showing preprocessed PointCloud" << endl;
            event_viewer->addPointCloud(filtered_cloud_, "cloud");
            event_viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud");
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"cloud");
        }
    }

    if (event.getKeySym () == "i" && event.keyDown ()){
        event_viewer->removeAllShapes();
        event_viewer->removeAllPointClouds();
        boundary_search_radius_ += 0.1;
        cout << "i was pressed => increasing the boundary point search radius by 0.1" << endl;
        CalculateCentroids();
        Visualize();

    }
    if (event.getKeySym () == "r" && event.keyDown ()){
        event_viewer->removeAllShapes();
        event_viewer->removeAllPointClouds();
        boundary_search_radius_ -= 0.1;
        cout << "i was pressed => reducing the boundary point search radius by 0.1" << endl;
        CalculateCentroids();
        Visualize();

    }
}

void HoleDetector::PointPickerCb(const visualization::PointPickingEvent &event, void *param) {
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

void HoleDetector::OnMouse(int event, int x, int y, int flags, void* param) {
    if(event != cv::EVENT_LBUTTONDOWN) {
        return;
    }
    auto* mp_ptr = (MouseParams*)param;
    cv::Mat & img = mp_ptr->img;
    cv::Point point (x, y);
    mp_ptr->points.push_back(point);
    circle(img, point, 5, cv::Scalar(255, 0, 0), -1);
}
