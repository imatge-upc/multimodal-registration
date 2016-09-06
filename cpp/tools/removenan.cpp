#include <iostream>

#include <limits>
#include <vector>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

#include <SimppleLogger.hpp>

typedef pcl::PointXYZRGB PointTypeIn;

int
main (int argc, char* argv[])
{
    std::string path_in, path_out;
    float x0, x1, y0, y1, z0, z1;
 
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&path_in)->required())
        ("output", boost::program_options::value<std::string>(&path_out)->required())

    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);


    pcl::PointCloud<PointTypeIn>::Ptr cloud(new pcl::PointCloud<PointTypeIn>);
    if(path_in.substr( path_in.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path_in, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in);
            return (-1);
        }
    }
    else if(path_in.substr( path_in.length() - 3 ) == "ply")
    {
        if (pcl::io::loadPLYFile<PointTypeIn>(path_in, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in);
            return (-1);
        }
    } else {
        PCL_ERROR("File extension for file %s not recognized\n", path_in);
        return (-1);
    }
    std::cout << "Loaded " << cloud->width << " * " << cloud->height
            << " data points from " << path_in << std::endl;


    std::vector<int> hh;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, hh);

    pcl::io::savePLYFileASCII (path_out, *cloud);

    return (0);
}
