//
// Created by Nasib Naimi on 23.03.22.
//
// std imports
#include <iostream>
#include <thread>
#include <string>

// PCL imports
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h> // for concatenateFields
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/transforms.h>

// utilities
#include <yaml-cpp/yaml.h>
//#include <yaml-cpp/node.h>

//#include <pcl/features/boundary.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/point_types.h>
//#include <pcl/conversions.h>
//#include <CGAL/Point_3.h>
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/Simple_cartesian.h>

using namespace std;
using namespace pcl;
using namespace std::literals::chrono_literals;

//typedef CGAL::Simple_cartesian<double> Kernel;
//typedef Kernel::Point_3 Point;
//typedef CGAL::Surface_mesh<CGAL::Point_3> CGAL_Mesh;

//int MeshPCL2CGAL(pcl::PolygonMesh::Ptr PCL_mesh, CGAL::Surface_mesh<CGAL::Point_3> & CGAL_mesh)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2( PCL_mesh->cloud, *mesh_cloud );
//
//    // clear and reserve the
//    CGAL_mesh.clear();
//    int n = mesh_cloud->size();
//    int f = PCL_mesh->polygons.size();
//    int e = 0;
//    CGAL_mesh.reserve(n, 2*f, e);
//
//    //copy the vertices
//    double x, y, z;
//    for (int i=0; i<mesh_cloud->size(); i++) {
//        Point p;
//        x = mesh_cloud->points[i].x;
//        y = mesh_cloud->points[i].y;
//        z = mesh_cloud->points[i].z;
//        p = Point(x, y, z);
//        CGAL_mesh.add_vertex(p);
//    }
//
//    // copy the faces
//    std::vector <int> vertices;
//    for(int i=0; i<PCL_mesh->polygons.size(); i++) {
//        vertices.resize(3);
//        vertices[0] = PCL_mesh->polygons[i].vertices[0];
//        vertices[1] = PCL_mesh->polygons[i].vertices[1];
//        vertices[2] = PCL_mesh->polygons[i].vertices[2];
//        CGAL_mesh.add_face(CGAL_Mesh::Vertex_index (vertices[0]),
//                           CGAL_Mesh::Vertex_index (vertices[1]),
//                           CGAL_Mesh::Vertex_index (vertices[2]));
//    }
//
//    return 0;
//}


//int MeshCGAL2PCL(CGAL::CGAL_Mesh CGAL_mesh, pcl::PolygonMesh::Ptr old_PCL_mesh, pcl::PolygonMesh::Ptr PCL_mesh)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2( old_PCL_mesh->cloud, *mesh_cloud );
//
//    int i=0;
//    BOOST_FOREACH(CGAL::CGAL_vertex v, vertices(CGAL_mesh))
//    {
//        mesh_cloud->points[i].x = CGAL_mesh[v].point.x();
//        mesh_cloud->points[i].y = CGAL_mesh[v].point.y();
//        mesh_cloud->points[i].z = CGAL_mesh[v].point.z();
//        i++;
//    }
//
//    //BOOST_FOREACH(CGAL_vertex v, vertices(CGAL_mesh))
//    //BOOST_FOREACH(CGAL_face f, faces(CGAL_mesh))
//
//    pcl::toPCLPointCloud2( *mesh_cloud, PCL_mesh->cloud );
//
//    return 0;
//}

/* CONST VARIABLES */
bool debug_;

std::string cloud_file_;
std::string trajectory_file_;
std::string floorplan_file_;

int PoissonDepth_;
float NormalSearchRadius_;

double OutlierRadius_;
int MinNeighbours_;

double ImageResolution_;

double PassXLimMin_;
double PassXLimMax_;
double PassYLimMin_;
double PassYLimMax_;

/* MEMBER VARIABLES */
Eigen::Affine3f center_translation = Eigen::Affine3f::Identity();
Eigen::Affine3f center_rotation = Eigen::Affine3f::Identity();
Eigen::Vector4f centroid;
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr floor_projected (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr ceiling_projected (new PointCloud<PointXYZ>);
PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
PointCloud<PointXYZ>::Ptr intermediate(new PointCloud<PointXYZ>);
PolygonMesh mesh;

void pp_callback(const pcl::visualization::PointPickingEvent& event,
                 void* viewer_void) {
    std::cout << "Picking event active" << std::endl;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << x << "; " << y << "; " << z << std::endl;
    }
}

void ReadYaml() {
    try {
        YAML::Node config = YAML::LoadFile("../cfg/config.yaml");
        debug_ = config["debug"].as<bool>();

        cloud_file_ = config["input"]["cloud_file"].as<std::string>();
        trajectory_file_ = config["input"]["trajectory_file"].as<std::string>();
        floorplan_file_ = config["input"]["floorplan_file"].as<std::string>();

        PoissonDepth_ = config["parameters"]["poisson_depth"].as<int>();
        NormalSearchRadius_ = config["parameters"]["normal_search_radius"].as<double>();

        OutlierRadius_ = config["parameters"]["outlier_radius"].as<double>();
        MinNeighbours_ = config["parameters"]["outlier_min_neighbours"].as<int>();

        ImageResolution_ = config["parameters"]["image_resolution"].as<double>();

        PassXLimMin_ = config["parameters"]["pass_xlim_min"].as<double>();
        PassXLimMax_ = config["parameters"]["pass_xlim_max"].as<double>();
        PassYLimMin_ = config["parameters"]["pass_ylim_min"].as<double>();
        PassYLimMax_ = config["parameters"]["pass_ylim_max"].as<double>();

    } catch(const YAML::ParserException& ex) {
        std::cout << ex.what() << std::endl;
    }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    visualization::PCLVisualizer::Ptr event_viewer = *static_cast<visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "r" && event.keyDown ())
    {
        ReadYaml();
        if (event_viewer->contains("raw")) {
            cout << "r was pressed => removing preprocessed PointCloud" << endl;
            event_viewer->removePointCloud("raw");
        } else {
            cout << "r was pressed => showing preprocessed PointCloud" << endl;
            event_viewer->addPointCloud(cloud,"raw");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "raw");
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1,"raw");
        }

    } else if (event.getKeySym () == "b" && event.keyDown()) {

        if (event_viewer->contains("floor")) {
            cout << "b was pressed => Removing Floor" << endl;
            event_viewer->removePointCloud("floor");
        } else {
            cout << "b was pressed => Showing Floor" << endl;
//            event_viewer->addPointCloud(floor_projected,"floor");
            event_viewer->addPointCloud(ceiling_projected,"ceiling");
//            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "floor");
//            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1,"floor");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.0f, 0.5f, "ceiling");
            event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1,"ceiling");

        }

    } else if (event.getKeySym () == "m" && event.keyDown()) {

        if (event_viewer->contains("mesh")) {
            cout << "m was pressed => Removing mesh" << endl;
            event_viewer->removePolygonMesh("mesh");
        } else {
            cout << "m was pressed => Showing mesh" << endl;
            event_viewer->addPolygonMesh(mesh,"meshes",0);
            event_viewer->addCoordinateSystem (1.0);
        }

    } else if (event.getKeySym () == "n" && event.keyDown()) {

        if (event_viewer->contains("normals")) {
            cout << "n was pressed => Removing normals" << endl;
            event_viewer->removePointCloud("normals");
        } else {
            cout << "n was pressed => Showing normals" << endl;
            event_viewer->addPointCloudNormals<PointXYZ, Normal>(intermediate, cloud_normals, 1, 0.08, "normals");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "normals");
        }

    } else if (event.getKeySym () == "i" && event.keyDown()) {

        if (event_viewer->contains("intermediate")) {
            cout << "i was pressed => Removing normals" << endl;
            event_viewer->removePointCloud("normals");
        } else {
            cout << "i was pressed => Showing normals" << endl;
            event_viewer->addPointCloud(intermediate,"intermediate");
            event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "normals");
        }

    } else if (event.getKeySym () == "c" && event.keyDown()) {
        cout << "c was pressed => clearing all" << endl;
        event_viewer->removePolygonMesh("mesh");
        event_viewer->removeAllPointClouds();
    }
}

void preprocess4HoleDet(PointCloud<PointXYZ>::Ptr cloud) {

    /* Filtering and Preprocessing */
    // radius outlier removal
    RadiusOutlierRemoval<PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(3);
    outrem.setMinNeighborsInRadius (10);
    outrem.setKeepOrganized(true);
    outrem.filter(*cloud);

    // Voxel filter
    VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (* cloud);

    // Transform to center
    compute3DCentroid(*cloud, centroid);
    center_translation.translation() << -centroid[0], -centroid[1], -centroid[2];

    transformPointCloud (*cloud, *cloud, center_translation);

    float theta = 1.0;
    center_rotation.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    transformPointCloud (*cloud, *cloud, center_rotation);

    // Passthrough filtering
    PassThrough<PointXYZ> pass_x;
    pass_x.setInputCloud (cloud);
    pass_x.setFilterFieldName ("x");
    pass_x.setFilterLimits (PassXLimMin_, PassXLimMax_);
    pass_x.filter (* cloud);
    PassThrough<PointXYZ> pass_y;
    pass_y.setInputCloud (cloud);
    pass_y.setFilterFieldName ("y");
    pass_y.setFilterLimits (PassYLimMin_, PassYLimMax_);
    //pass.setFilterLimitsNegative (true);
    pass_y.filter (* cloud);

    /* End Filtering and Preprocessing */
}

void constructMesh(PointCloud<PointXYZ>::Ptr cloud, PolygonMesh & mesh) {
    /* normal estimation using OMP */
    NormalEstimationOMP<PointXYZ, Normal> normal_estimate;
    normal_estimate.setNumberOfThreads(8);
    normal_estimate.setInputCloud(cloud);
    normal_estimate.setRadiusSearch(NormalSearchRadius_);
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
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
    concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

    /* poisson reconstruction */
    Poisson<PointNormal> poisson;
    poisson.setDepth(PoissonDepth_);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(mesh);
    /* end poisson reconstruction */
}

void getProjectedFloor(const PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr cloud_out, PointCloud<PointXYZ>::Ptr intermediate) {
    /* Floor Extraction */
    PointCloud<pcl::PointXYZ>::Ptr floor (new PointCloud<pcl::PointXYZ>);
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
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);
    copyPointCloud<PointXYZ>(*cloud, *inliers, *floor);


    ExtractIndices<PointXYZ> extract;
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_in);

    // Project the model inliers
    ProjectInliers<PointXYZ> proj;
    proj.setModelType (SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud (floor);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_out);
    /* End Floor Extraction */

    copyPointCloud<PointXYZ>(*cloud, *intermediate);
    ExtractIndices<pcl::PointXYZ> extract_ceiling;

    extract_ceiling.setInputCloud(intermediate);
    extract_ceiling.setIndices(inliers);
    extract_ceiling.setNegative(true);
    extract_ceiling.filter(*intermediate);

}

int main(int argc, char *argv[])
{
    PCDReader reader;
    visualization::PCLVisualizer::Ptr viewer (new visualization::PCLVisualizer ("3D Viewer"));

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 1000, 0, 0, 0, 1, 0, 0);
//    viewer->setCameraFieldOfView(0.523599);
    /* Read in PointCloud */
    reader.read ("/Users/nasib/code/HoleDet/data/talstrasse/hololens.pcd", *cloud);

    /* Filtering and Preprocessing */
    preprocess4HoleDet(cloud);

    /* Floor and ceiling Construction */
    getProjectedFloor(cloud, floor_projected, intermediate);
    getProjectedFloor(intermediate, ceiling_projected, intermediate);

    *intermediate += *floor_projected;
    *intermediate += *ceiling_projected;

    /* Mesh Construction */
    constructMesh(intermediate, mesh);

    /* Visualization */
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(10ms);
    }
    return 0;
}

//void getProjectedCeiling(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr cloud_out) {
//    /* Floor Extraction */
//    PointCloud<pcl::PointXYZ>::Ptr floor (new PointCloud<pcl::PointXYZ>);
//    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
//    PointIndices::Ptr inliers (new PointIndices);
//    // Create the segmentation object
//    SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (SACMODEL_PLANE);
//    seg.setMethodType (SAC_RANSAC);
//    seg.setDistanceThreshold (0.1);
//    seg.setMaxIterations(10000);
//    // Segment dominant plane
//    seg.setInputCloud (cloud_in);
//    seg.segment (*inliers, *coefficients);
//    copyPointCloud<PointXYZ>(*cloud, *inliers, *floor);
//
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    extract.setInputCloud (cloud_in);
//    extract.setIndices (inliers);
//    extract.setNegative (true);
//    extract.filter (*cloud_in);
//
//    // Project the model inliers
//    ProjectInliers<pcl::PointXYZ> proj;
//    proj.setModelType (SACMODEL_PLANE);
//    // proj.setIndices (inliers);
//    proj.setInputCloud (floor);
//    proj.setModelCoefficients (coefficients);
//    proj.filter (*cloud_out);
//    /* End Floor Extraction */
//}