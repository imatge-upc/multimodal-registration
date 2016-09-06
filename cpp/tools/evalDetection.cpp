#include <iostream>

#include <limits>
#include <vector>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <SimppleLogger.hpp>

typedef pcl::PointXYZI PointTypeIn;

int
main (int argc, char* argv[])
{
    std::string path_in_cloud, path_in_lines;
    float x0, x1, y0, y1, z0, z1;
 
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input-cloud", boost::program_options::value<std::string>(&path_in_cloud)->required())
        ("input-lines", boost::program_options::value<std::string>(&path_in_lines)->required())
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);


    pcl::PointCloud<PointTypeIn>::Ptr cloud(new pcl::PointCloud<PointTypeIn>);
    if(path_in_cloud.substr( path_in_cloud.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path_in_cloud, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in_cloud);
            return (-1);
        }
    }
    else if(path_in_cloud.substr( path_in_cloud.length() - 3 ) == "ply")
    {
        if (pcl::io::loadPLYFile<PointTypeIn>(path_in_cloud, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in_cloud);
            return (-1);
        }
    } else {
        PCL_ERROR("File extension for file %s not recognized\n", path_in_cloud);
        return (-1);
    }
    std::cout << "Loaded " << cloud->width << " * " << cloud->height
            << " data points from " << path_in_cloud << std::endl;

    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY (path_in_lines, mesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mesh(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(path_in_lines, *cloud_mesh) == -1) {
        PCL_ERROR("Couldn't read the file %s \n", path_in_lines);
        return (-1);
    }
    std::vector<pcl::PointXYZ> p1s, p2s;

    for (int i=0; i<mesh.polygons.size(); i++){
        for (int j=0; j<mesh.polygons[i].vertices.size(); j++){
            p1s.push_back(cloud_mesh->points[mesh.polygons[i].vertices[j]]);
            int idx2 = j+1;
            if (idx2 >= mesh.polygons[i].vertices.size()) idx2 = 0;
            p2s.push_back(cloud_mesh->points[mesh.polygons[i].vertices[idx2]]);
        }
    }
    std::cout << "Constructed lines: " << p1s.size() << std::endl;

    float error = 0;
    float intens = 0;
    for (int i=0; i<cloud->points.size(); i++){
        float error_point =0; 
        for (int j=0; j<p1s.size(); j++){
            float error_step = 0;
            Eigen::Vector3f p1, p2, p, projection;
            p1 << p1s[j].x, p1s[j].y, p1s[j].z;
            p2 << p2s[j].x, p2s[j].y, p2s[j].z;
            p << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z; 
            float l2 = (p1-p2).squaredNorm();
            if (l2 == 0.0) error_step = (p1-p).norm();
            else {
                float val = ((p-p2).dot(p1-p2))/l2;
                float t = std::max(float(0), std::min(float(1),val));
                projection = p2+(p1-p2)*t;
                error_step = (p-projection).norm();
            }
            if (j==0) error_point = error_step;
            else error_point = std::min(error_point, error_step);
        }
        error += error_point*cloud->points[i].intensity;
        intens += cloud->points[i].intensity;
    }
    error /= intens;

    std::cout << "ERROR: " << error << std::endl;

    return (0);
}
