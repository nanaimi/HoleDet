//
// Created by Nasib Naimi on 23.03.22.
//
#include "helpers.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>

MeshPCL2CGAL(pcl::PolygonMesh::Ptr PCL_mesh, CGAL_Mesh& CGAL_mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2( PCL_mesh->cloud, *mesh_cloud );

    // clear and reserve the
    CGAL_mesh.clear();
    int n = mesh_cloud->size();
    int f = PCL_mesh->polygons.size();
    int e = 0;
    CGAL_mesh.reserve(n, 2*f, e);

    //copy the vertices
    double x, y, z;
    for (int i=0; i<mesh_cloud->size(); i++)
    {
        Point p;
        x = mesh_cloud->points[i].x;
        y = mesh_cloud->points[i].y;
        z = mesh_cloud->points[i].z;
        p = Point(x, y, z);
        CGAL_mesh.add_vertex(p);
    }

    // copy the faces
    std::vector <int> vertices;
    for(int i=0; i<PCL_mesh->polygons.size(); i++)
    {
        vertices.resize(3);
        vertices[0] = PCL_mesh->polygons[i].vertices[0];
        vertices[1] = PCL_mesh->polygons[i].vertices[1];
        vertices[2] = PCL_mesh->polygons[i].vertices[2];
        CGAL_mesh.add_face(CGAL_Mesh::Vertex_index (vertices[0]),
                           CGAL_Mesh::Vertex_index (vertices[1]),
                           CGAL_Mesh::Vertex_index (vertices[2]));
    }

    return 0;
}


MeshCGAL2PCL(CGAL_Mesh CGAL_mesh, pcl::PolygonMesh::Ptr old_PCL_mesh, pcl::PolygonMesh::Ptr PCL_mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2( old_PCL_mesh->cloud, *mesh_cloud );

    int i=0;
    BOOST_FOREACH(CGAL_vertex v, vertices(CGAL_mesh))
    {
        mesh_cloud->points[i].x = CGAL_mesh[v].point.x();
        mesh_cloud->points[i].y = CGAL_mesh[v].point.y();
        mesh_cloud->points[i].z = CGAL_mesh[v].point.z();
        i++;
    }

    //BOOST_FOREACH(CGAL_vertex v, vertices(CGAL_mesh))
    //BOOST_FOREACH(CGAL_face f, faces(CGAL_mesh))

    pcl::toPCLPointCloud2( *mesh_cloud, PCL_mesh->cloud );

    return 0;
}