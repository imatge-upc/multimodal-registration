#include <iostream>

#include <limits>
#include <vector>
#include <ctime>

#include <pcl/pcl_base.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

#include <pcl/visualization/cloud_viewer.h>
#include <boost/program_options.hpp>
#include <pcl/features/edge_multiscale.h>
//#include <armadillo>
#include <math.h>


typedef pcl::PointXYZRGB PointTypeIn;
// typedef pcl::PointXYZRGB PointTypeOut;
typedef pcl::PointXYZRGB PointTypeOut;
typedef pcl::PointXYZI PointTypeOutPly;


void setViewerCamera( pcl::visualization::PCLVisualizer &viewer, std::string cam_params_str) {
    if (cam_params_str.compare("") != 0) {
        pcl::visualization::Camera cam;
        std::stringstream str(cam_params_str);
        std::vector<double> cam_params((std::istream_iterator<double>(str)), std::istream_iterator<double>());
        cam.focal[0] = cam_params[0];
        cam.focal[1] = cam_params[1];
        cam.focal[2] = cam_params[2];
        cam.pos[0] = cam_params[3];
        cam.pos[1] = cam_params[4];
        cam.pos[2] = cam_params[5];
        cam.view[0] = cam_params[6];
        cam.view[1] = cam_params[7];
        cam.view[2] = cam_params[8];
        cam.clip[0] = cam_params[9];
        cam.clip[1] = cam_params[10];
        cam.fovy = cam_params[11];
        cam.window_size[0] = cam_params[12];
        cam.window_size[1] = cam_params[13];
        cam.window_pos[0] = cam_params[14];
        cam.window_pos[1] = cam_params[15];
        viewer.setCameraParameters(cam, 0);
    }
}


int
main (int argc, char* argv[])
{
    std::string path_in, path_out, path_out_orig, path_out_pcd, path_out_ply;
    bool x, single_k;
    float weight_low_thresh, weight_high_thresh, weight_level_thresh, weight_level_color_thresh, color_factor;
    int min_K, max_K, step_K, max_K_color;
    std::string cam_params;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("input,i", boost::program_options::value<std::string>(&path_in)->required(),
     "Input point cloud in PCD format")

    ("weight_level_thresh", boost::program_options::value<float>(&weight_level_thresh)->default_value(40.0),
     "Threshold on the neighbourhood")
    ("weight_level_color_thresh", boost::program_options::value<float>(&weight_level_color_thresh)->default_value(40.0),
     "Threshold on the neighbourhood")

    ("min_K", boost::program_options::value<int>(&min_K)->default_value(15),
     "Number of neighbours to be considered initially")
    ("max_K", boost::program_options::value<int>(&max_K)->default_value(250),
     "Number of neighbours to be considered initially")
    ("step_K", boost::program_options::value<int>(&step_K)->default_value(5),
     "Number of neighbours to be considered initially")
    ("max_K_color", boost::program_options::value<int>(&max_K_color)->default_value(250),
     "Number of neighbours to be considered initially")
    ("single_k", boost::program_options::value<bool>(&single_k)->default_value(false),
     "Number of neighbours to be considered initially")

    ("screenshot", boost::program_options::value<std::string>(&path_out)->default_value(""),
     "capture screenshot of the output")
    ("screenshot-orig", boost::program_options::value<std::string>(&path_out_orig)->default_value(""),
     "Capture screenshot of the input")
    ("output-pcd", boost::program_options::value<std::string>(&path_out_pcd)->default_value(""),
     "Save the output to a pcd file")
    ("output-ply", boost::program_options::value<std::string>(&path_out_ply)->default_value(""),
     "Save the output to a ply file")
    ("x", boost::program_options::value<bool>(&x)->default_value(false),
     "Show graphic data (unset when working with no X)")
    ("camera", boost::program_options::value<std::string>(&cam_params)->default_value(""),
     "Vector with the 16 camera parameters:  'focal[3], pos[3], view[3], clip[2], fovy, window_size[2], window_pos[2]'")
    ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    // Working with point clouds
    // Define initial cloud and keypoints
    pcl::PointCloud<PointTypeIn>::Ptr cloud(new pcl::PointCloud<PointTypeIn>);
    if (path_in.substr( path_in.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path_in, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in);
            return (-1);
        }
    }
    else if (path_in.substr( path_in.length() - 3 ) == "ply")
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

    if (path_out_orig.compare("") != 0) {
        pcl::visualization::PCLVisualizer viewer2 ("3D Edge Viewer");
        viewer2.addPointCloud (cloud, "original point cloud");
        setViewerCamera(viewer2, cam_params);
        viewer2.saveScreenshot(path_out_orig);
    }

    // Computation
    pcl::PointCloud<pcl::PointXYZI> weights;
    pcl::PointCloud<PointTypeOut>::Ptr weights_ply;

    std::vector<int> hh;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, hh);

    pcl::MultiscaleEdgeDetector<PointTypeIn> detector = pcl::MultiscaleEdgeDetector<PointTypeIn>();
    // detector.setItersParameters(min_K, max_K, max_K_color, step_K);
    // detector.setItersStopParameters();
    // detector.setSingleK(false);
    // detector.setThresholds(weight_level_thresh, weight_level_color_thresh);
      detector.setItersParameters(3,100,10,1);
  detector.setItersStopParameters();
  detector.setSingleK(false);
  detector.setThresholds (0.1, 0.05);
    detector.setInputCloud(*cloud);
    detector.estimate(weights);


    // max_ins = 0;
    // for (int i=0; i<weights_ply->points.size(); i++){
    //     if (weights_ply->points[i].intensity>max_ins) max_ins = weights_ply->points[i].intensity;
    // }
    // std::cout << "Maxins" << max_ins<<  std::endl;
    // for (int i=0; i<weights_ply->points.size(); i++){
    //     weights_ply->points[i].intensity /= max_ins;
    // }

    if (path_out_pcd.compare("") != 0) {
        pcl::io::savePCDFileASCII (path_out_pcd, weights);
    }

    if (path_out_ply.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_out_ply, weights);
    }
    // Display edges in PCLVisualizer
    pcl::visualization::PCLVisualizer viewer ("3D Edge Vieweer");
    viewer.setSize (640, 480);
    viewer.addCoordinateSystem (0.2f, "global");
    // viewer.addPointCloud<PointTypeOut>(weights, "weight point cloud on level K");

    const int point_size = 2;
    if (path_out.compare("") != 0) {
        setViewerCamera(viewer, cam_params);
        viewer.saveScreenshot(path_out);
    }

    if (x) {
        pcl::visualization::Camera cam;
        setViewerCamera(viewer, cam_params);
        while (!viewer.wasStopped ())
        {
            viewer.getCameraParameters(cam);
            // std::cout << "Focal: " << cam.focal[0] << " " << cam.focal[1] << " " << cam.focal[2] << std::endl;
            // std::cout << "Pos: " << cam.pos[0] << " " << cam.pos[1] << " " << cam.pos[2] << std::endl;
            // std::cout << "View: " << cam.view[0] << " " << cam.view[1] << " " << cam.view[2] << std::endl;
            // std::cout << "Clip: " << cam.clip[0] << " " << cam.clip[1] << std::endl;
            // std::cout << "Fovy: " << cam.fovy << std::endl;
            // std::cout << "Window_size: " << cam.window_size[0] << " " << cam.window_size[1] << std::endl;
            // std::cout << "Window_pos: " << cam.window_pos[0] << " " << cam.window_pos[1]  << std::endl;
            // std::cout << "String: " << cam.focal[0] << " " << cam.focal[1] << " " << cam.focal[2] << " "
            //           << cam.pos[0] << " " << cam.pos[1] << " " << cam.pos[2] << " "
            //           << cam.view[0] << " " << cam.view[1] << " " << cam.view[2] << " "
            //           << cam.clip[0] << " " << cam.clip[1] << " "
            //           << cam.fovy << " "
            //           << cam.window_size[0] << " " << cam.window_size[1] << " "
            //           << cam.window_pos[0] << " " << cam.window_pos[1]  << std::endl;
            // cam.window_pos[1] = 109;
            // viewer.setCameraParameters(cam);
            viewer.spinOnce ();
            std::cout << "here?" << std::endl;
            boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }
    }



//    pcl::visualization::CloudViewer viewer1 ("All cloud");
//    viewer1.showCloud (cloud);
//    while (!viewer1.wasStopped ()){}

    return (0);
}
