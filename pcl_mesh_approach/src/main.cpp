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
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

// Poisson stuff imports
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include <CGAL/Point_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>

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
const int    POISSONDEPTH = 7;
const float  NORMALSEARCHRADIUS = 0.2;
const double PASSXLIMMIN = -13.0;
const double PASSXLIMMAX = 24.0;
const double PASSYLIMMIN = -15.0;
const double PASSYLIMMAX = 25.0;

/* MEMBER VARIABLES */
Eigen::Affine3f center_translation = Eigen::Affine3f::Identity();
Eigen::Affine3f center_rotation = Eigen::Affine3f::Identity();
Eigen::Vector4f centroid;
PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr floor_projected (new PointCloud<PointXYZ>);
PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
PolygonMesh mesh;

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

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    visualization::PCLVisualizer::Ptr event_viewer = *static_cast<visualization::PCLVisualizer::Ptr *> (viewer_void);

    if (event.getKeySym () == "r" && event.keyDown ())
    {
        cout << "r was pressed => Showing preprocessed PointCloud" << endl;

        event_viewer->removePolygonMesh("mesh");
        event_viewer->removeAllPointClouds();

        event_viewer->addPointCloud(cloud,"raw");
        event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "raw");
        event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2,"raw");

    } else if (event.getKeySym () == "b" && event.keyDown()) {
        cout << "b was pressed => Showing Floor" << endl;

        event_viewer->removePolygonMesh("mesh");
        event_viewer->removeAllPointClouds();

        event_viewer->removeAllPointClouds();
        event_viewer->addPointCloud(floor_projected,"floor");
        event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "floor");
        event_viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1,"floor");

    } else if (event.getKeySym () == "m" && event.keyDown()) {
        cout << "m was pressed => Showing reconstructed mesh" << endl;

        event_viewer->removePolygonMesh("mesh");
        event_viewer->removeAllPointClouds();

        event_viewer->addPolygonMesh(mesh,"meshes",0);
        event_viewer->addCoordinateSystem (1.0);

    } else if (event.getKeySym () == "n" && event.keyDown()) {
        cout << "n was pressed => Showing normals and point cloud" << endl;

        event_viewer->removePolygonMesh("mesh");
        event_viewer->removeAllPointClouds();

        event_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 1, 0.08, "normals");
        event_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "normals");

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
    pass_x.setFilterLimits (PASSXLIMMIN, PASSXLIMMAX);
    pass_x.filter (* cloud);
    PassThrough<PointXYZ> pass_y;
    pass_y.setInputCloud (cloud);
    pass_y.setFilterFieldName ("y");
    pass_y.setFilterLimits (PASSYLIMMIN, PASSYLIMMAX);
    //pass.setFilterLimitsNegative (true);
    pass_y.filter (* cloud);

    /* End Filtering and Preprocessing */
}

void constructMesh(PointCloud<PointXYZ>::Ptr cloud) {
    /* normal estimation using OMP */
    NormalEstimationOMP<PointXYZ, Normal> normal_estimate;
    normal_estimate.setNumberOfThreads(8);
    normal_estimate.setInputCloud(cloud);
    normal_estimate.setRadiusSearch(NORMALSEARCHRADIUS);
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
    poisson.setDepth(POISSONDEPTH);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(mesh);
    /* end poisson reconstruction */
}

int main(int argc, char *argv[])
{
    PCDReader reader;
    visualization::PCLVisualizer::Ptr viewer (new visualization::PCLVisualizer ("3D Viewer"));

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 30, 0,    0, 0, 0,   0, 0, 1);
    viewer->setCameraFieldOfView(0.523599);
//    viewer->setCameraClipDistances(0.00522511, 50);

    /* Read in PointCloud */
    reader.read ("/Users/nasib/code/HoleDet/data/talstrasse/hololens.pcd", *cloud);

    /* Filtering and Preprocessing */
    preprocess4HoleDet(cloud);

    /* Floor Extraction */
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor (new pcl::PointCloud<pcl::PointXYZ>);

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
    /* End Floor Extraction */

    /* Mesh Construction */
    constructMesh(cloud);

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