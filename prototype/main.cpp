#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <vector>
#include <Eigen/Core>
#include <boost/multi_array.hpp>


int main() {
    std::string dataset = "/home/maurice/CLionProjects/prototype/hololens.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    bool visuals = true;

    float leaf_x = 0.2;
    float leaf_y = 0.2;
    float leaf_z = 0.2;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (dataset, *cloud) == -1) {
          PCL_ERROR ("Couldn't read file\n");
          return (-1);
    }
    std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points"
                << std::endl;
    //filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leaf_x, leaf_y, leaf_z);
    filter.filter(*cloud_filtered);

    std::cout << "After filtering: "
                << cloud_filtered->height * cloud_filtered->width
                << " points in the cloud."
                << std::endl;

    // find min max points
    if(cloud_filtered->isOrganized()) {
        std::cout << "The cloud is organized" << std::endl;
        return(-1);
    }

    // pcl::people::HeightMap2D<pcl::PointXYZ>::HeightMap2D height_map;

    /*
    pcl::PointXYZ pt_min;
    pcl::PointXYZ pt_max;

    pcl::getMinMax3D(*cloud_filtered,pt_min, pt_max);

    std::cout << "Min \t X \t Y \t Z:\n"
                << "\t" << pt_min.x << "\t" << pt_min.y << "\t" << pt_min.z << "\n"
                << "Max \t X \t Y \t Z:\n"
                << "\t" << pt_max.x << "\t" << pt_max.y << "\t" << pt_max.z << "\n";

    std::vector<std::vector<std::vector<float>>> z_min;
    std::vector<std::vector<std::vector<float>>> z_max;

    const int len_x = abs(pt_max.x - pt_min.x) / leaf_x;
    const int len_y = abs(pt_max.y - pt_min.y) / leaf_y;
    const int len_z = abs(pt_max.z - pt_min.z) / leaf_z;

    boost::multi_array<float, 3> z_min(boost::extents[len_x][len_y][len_z]);
    boost::multi_array<float, 3> z_max(boost::extents[len_x][len_y][len_z]);
    */




    //visuals
    if (visuals) {
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloud_filtered);
        while (!viewer.wasStopped()) {}
    }
    return 0;
}
