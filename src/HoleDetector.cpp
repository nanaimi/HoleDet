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
        floorplan_filtered_(new PointCloud<PointXYZ>),
        hull_cloud_(new PointCloud<PointXYZ>),
        hole_hull_cloud_(new PointCloud<pcl::PointXYZ>),
        interior_boundaries_(new PointCloud<PointXYZ>),
        floorplan_(new PointCloud<PointXYZ>),
        dense_floorplan_(new PointCloud<PointXYZ>),
        viewer_ (new visualization::PCLVisualizer ("3D Viewer")),
        floor_coefficients_ (new ModelCoefficients),
        floor_normals_(new pcl::PointCloud<pcl::Normal>),
        boundary_normals_(new pcl::PointCloud<pcl::Normal>),
        hole_index_(0)
{
    path_ = path;
    config_file_ = config_filename;
    ReadYAML();
    min_score_ = 4;
    tp_.cnt = 0;


    InitFilters();
    raw_cloud_ = Utils::ReadCloud(pointcloud_file_, reader);
    Utils::ReadTrajectoriesAndGaze(trajectory_file_, gaze_file_,
                                        lengths_file_, reader, trajectories_, gazes_);
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
        gaze_file_ = path_ + config["input"]["gaze_file"].as<std::string>();
        lengths_file_ = path_ + config["input"]["lengths_file"].as<std::string>();
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

        boundary_search_radius_ = config["parameters"]["boundary_search_radius"].as<float>();
        angle_thresh_ = config["parameters"]["angle_thresh"].as<float>();

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

    gaze_scores_.grid.setLeafSize(0.1f, 0.1f, 0.1f);
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
     Utils::GetInteriorBoundaries(floor_projected_, dense_floorplan_, interior_boundaries_, floor_normals_);
     Utils::Calc2DNormals(interior_boundaries_, boundary_normals_, boundary_search_radius_);
     CalculateCentroids();
     Utils::CalcPoses(holes_);
}

void HoleDetector::GazeMap() {
    Utils::CreateGrid(dense_floorplan_, gaze_scores_);
    Utils::CalcGazeScores(gaze_scores_,trajectories_, gazes_);
    Eigen::MatrixXf all = gaze_scores_.scores[0] + gaze_scores_.scores[1] +gaze_scores_.scores[2]+gaze_scores_.scores[3];
    for(int i = 0; i < 5; i++) {
        cv::Mat scores_img;
        cv::Mat heatmap;
        if(i == 0) {
            cv::eigen2cv(all, scores_img);
        } else {
            cv::eigen2cv(gaze_scores_.scores[i-1], scores_img);
        }


        double max;
        cv::minMaxLoc(scores_img, NULL, &max, NULL, NULL);


        scores_img = scores_img / max;
        scores_img.convertTo(scores_img, CV_8U, 255);
        cv::applyColorMap(scores_img, heatmap, cv::COLORMAP_JET);

        for (int j = 0; j < holes_.size(); j++) {
            if (holes_[i].score < min_score_) { continue; }
            for (auto point: *holes_[j].points) {
                auto coords = gaze_scores_.grid.getGridCoordinates(point.x, point.y, point.z);
                auto px = gaze_scores_.offset_x + coords.x();
                auto py = gaze_scores_.offset_y + coords.y();
                cv::Vec3b & color = heatmap.at<cv::Vec3b>(px,py);
                color[0] = 0;
                color[1] = 0;
                color[2] = 255;
            }
            auto centroid = holes_[j].centroid;
            auto coords = gaze_scores_.grid.getGridCoordinates(centroid.x, centroid.y, centroid.z);
            auto px = gaze_scores_.offset_x + coords.x();
            auto py = gaze_scores_.offset_y + coords.y();
            cv::circle(heatmap, cv::Point(py, px), 2, cv::Scalar( 255, 0, 255 ),cv::FILLED);

        }

        cv::Mat resized;
        cv::resize(heatmap, resized, cv::Size(heatmap.cols*2, heatmap.rows*2));
        std::string fname = std::to_string(i) + ".jpg";
        imwrite(fname, resized);
        while(true) {
            cv::imshow("scores", resized);
            if (cv::waitKey(10) == 27) {
                cv::destroyAllWindows();
                break;
            }
        }
    }

}

void HoleDetector::CalculateCentroids() {
    Utils::GetHoleClouds(holes_, interior_boundaries_, boundary_search_radius_, boundary_normals_, angle_thresh_);
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

    viewer_ ->addPointCloud(floor_projected_, "floor_projected_");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                      0.0f, 0.2f, 0.8f, "floor_projected_");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floor_projected_");

    viewer_->addPointCloud(floorplan_, "floorplan");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                               0.5f, 0.0f, 0.5f, "floorplan");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floorplan");

    viewer_ ->addPointCloud(dense_floorplan_, "dense_floor");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                               1.0f, 0.0f, 0.0f, "dense_floor");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "dense_floor");

//    viewer_ ->addPointCloud(floorplan_filtered_, "floorplan_filtered");
//    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
//                                               0.5f, 0.0f, 0.5f, "floorplan_filtered");
//    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "floorplan_filtered");

//    viewer_ ->addPolygonMesh(full_mesh_, "full_mesh",0);

    viewer_ ->addPointCloud(interior_boundaries_, "interior_boundaries_");
    viewer_->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR,
                                               0.0f, 0.2f, 0.8f, "interior_boundaries_");
    viewer_->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "interior_boundaries_");

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

        //Handle poses visualisation
        for (int j = 0; j < holes_[i].poses.size(); j++) {
            pcl::PointXYZ p(holes_[i].poses[j].translation().x(),
                            holes_[i].poses[j].translation().y(),
                            holes_[i].poses[j].translation().z());
            viewer_->addSphere(p, 0.1, r1, 1 - r1, r3, center_name + "_pose"); //add colour sphere to pose
            viewer_->addCoordinateSystem(0.5, holes_[i].poses[j]); //display pose
        }

    }

    // Utils::DrawGazesInCloud(trajectories_, gazes_, viewer_);

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

void HoleDetector::GetFullMesh() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (floor_projected_);

    float ceiling_height = 4;

    float avg_nearest_dist = 0;
    int count = 0;
    // nr of nearest neighbours
    int K = 2;
    for (auto search_point : floor_projected_->points) {
        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);

        if ( kdtree.nearestKSearch (search_point, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
        {
            count++;
            avg_nearest_dist += pointKNNSquaredDistance[1];
        }
    }

    // size of voxel cell
    avg_nearest_dist = avg_nearest_dist / count;

    std::cout << "avg nearest dist: " << avg_nearest_dist << std::endl;

    // subsample the dense floorplan point-cloud and add to the full cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (dense_floorplan_);
    sor.setLeafSize (avg_nearest_dist, avg_nearest_dist, avg_nearest_dist);
    sor.filter (*tmp_cloud);

    *full_cloud += *tmp_cloud;

    // subsample the projected floorplan pointcloud
    sor.setInputCloud (floor_projected_);
    sor.setLeafSize (avg_nearest_dist, avg_nearest_dist, avg_nearest_dist);
    sor.filter (*tmp_cloud);

    *full_cloud += *tmp_cloud;
    *full_cloud += *filtered_cloud_;

    // remove all points that are higher than ceiling and lower than floor
    tmp_cloud->clear();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (full_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (dense_floorplan_->points[0].z, ceiling_height);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*tmp_cloud);

    *full_cloud += *tmp_cloud;

    const uint n = dense_floorplan_->width;
    pcl::Vertices::Ptr verticesPtr (new pcl::Vertices);
    std::vector<uint> vert_vec(n);
    iota(vert_vec.begin(), vert_vec.end(), 0);
    verticesPtr->vertices = vert_vec;
    std::vector<pcl::Vertices> polygon;
    polygon.push_back(*verticesPtr);

    pcl::CropHull<pcl::PointXYZ> crop;
    crop.setHullIndices(polygon);
    crop.setDim(2);

    crop.setHullCloud(dense_floorplan_);
    crop.setInputCloud(full_cloud);
    crop.filter(*full_cloud);

    floorplan_filtered_ = full_cloud;

    // Generate extra constraining points on the wall
    tmp_cloud->clear();
    for (auto point : dense_floorplan_->points) {
        float res = 1;
        int nr_iterations = ceiling_height / res;
        for(int i = 1; i <= nr_iterations; i++) {
            pcl::PointXYZ new_point;
            new_point.x = point.x;
            new_point.y = point.y;
            new_point.z = point.z + i*res;
            tmp_cloud->points.push_back(new_point);
        }
    }

    *full_cloud += *tmp_cloud;

    Utils::ConstructMesh(full_cloud, full_mesh_, kNormalSearchRadius_, kPoissonDepth_);

    std::cout << "constructed mesh" << std::endl;

}

void HoleDetector::CalculateVerticalScores() {
    Utils::ScoreVertical(holes_,floorplan_filtered_);
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
    if (event.getKeySym () == "n" && event.keyDown ())
    {

        if (event_viewer->contains("boundary_normals")) {
            cout << "n was pressed => removing boundary_normals" << endl;
            event_viewer->removePointCloud("boundary_normals");
        } else {
            cout << "n was pressed => showing boundary_normals" << endl;
            event_viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(interior_boundaries_, boundary_normals_, 1,0.2, "boundary_normals");
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

    if (event.getKeySym () == "m" && event.keyDown ()){
        event_viewer->removeAllShapes();
        event_viewer->removeAllPointClouds();
        cout << "m was pressed => showing mesh" << endl;
        Visualize();
        viewer_ ->addPolygonMesh(full_mesh_, "full_mesh",0);
    }

    if (event.getKeySym () == "z" && event.keyDown ())
    {


        if (event_viewer->contains("hole")) {
            cout << "n was pressed => cycling through to next hole" << endl;
            event_viewer->removePointCloud("hole");

            auto name = "hole_" + std::to_string(hole_index_);
            float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            float r3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            event_viewer->addPointCloud(holes_.at(hole_index_).points, name);
            event_viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, r1, 1 - r1, r3, name);
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5,name);

            do {
                if (hole_index_ < holes_.size() - 1) {
                    hole_index_++;
                } else {
                    hole_index_=0;
                }
            } while (holes_.at(hole_index_).score < 4);

            name = "hole_" + std::to_string(hole_index_);

            event_viewer->removePointCloud(name);

            event_viewer->addPointCloud(holes_.at(hole_index_).points, "hole");
            event_viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 1.0f, "hole");
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5,"hole");
            std::cout << "with score: " << holes_.at(hole_index_).score << " and variances x: " << holes_.at(hole_index_).cov_matrix(0,0) << " y: " << holes_.at(hole_index_).cov_matrix(1,1) << " z: " << holes_.at(hole_index_).cov_matrix(2,2) << std::endl;
        } else {

            auto name = "hole_" + std::to_string(hole_index_);
            event_viewer->removePointCloud(name);

            event_viewer->addPointCloud(holes_.at(hole_index_).points, "hole");
            event_viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 1.0f, "hole");
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5,"hole");
            std::cout << "with score: " << holes_.at(hole_index_).score << " and variances x: " << holes_.at(hole_index_).cov_matrix(0,0) << " y: " << holes_.at(hole_index_).cov_matrix(1,1) << " z: " << holes_.at(hole_index_).cov_matrix(2,2) << std::endl;
        }
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
