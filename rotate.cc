#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>  //  pcl::transformPointCloud 用到这个头文件
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

using namespace std;

int main (int argc, char **argv) {
    std::vector<int> filenames;

    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
        return -1;
    } else {
        cout << "Loading map successful." << endl;
    }
    cout << "file name is: " << argv[filenames[0]] << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
    }

    string pcd_name = argv[filenames[0]];
    pcd_name = "rotated_" + pcd_name;
    cout << "rotated_pcd_name: " << pcd_name << endl;

    float theta = M_PI /2;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

    pcl::io::savePCDFile(pcd_name, *transformed_cloud, true);

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_cloud_color_handler (source_cloud, 255, 255, 255);
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // 红
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    cout << "raw point_cloud is white, rotate point_cloud is red." << endl;

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景为深灰
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // 设置窗口位置

    while (!viewer.wasStopped ()) { // 在按下 "q" 键之前一直会显示窗口
        viewer.spinOnce ();
    }

}