#include <iostream>

#include <limits>
#include <vector>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <SimppleLogger.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


typedef pcl::PointXYZRGB PointTypeIn;

int
main (int argc, char* argv[])
{
    std::string path_in, path_out;
    float noise_xy, noise_z;
 
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&path_in)->required())
        ("noise_xy", boost::program_options::value<float>(&noise_xy)->default_value(0.0))
        ("noise_z", boost::program_options::value<float>(&noise_z)->default_value(0.0))
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

  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));

  boost::normal_distribution<> distribution_n(0, noise_xy);
  boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dist_n(*rng, distribution_n);
  boost::normal_distribution<> distribution_nz(0, noise_z);
  boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dist_nz(*rng, distribution_nz);

    pcl::PointCloud<PointTypeIn>::Ptr cloud_2(new pcl::PointCloud<PointTypeIn>);

    for (int j = 0; j < cloud->points.size(); j++) {
        PointTypeIn p;
        pcl::copyPoint(cloud->points[j], p);
        p.x = cloud->points[j].x + dist_n();
        p.y = cloud->points[j].y + dist_n();
        p.z = cloud->points[j].z + dist_nz();
        cloud_2->push_back(p);
    }
    pcl::io::savePLYFileASCII (path_out, *cloud_2);

    return (0);
}
