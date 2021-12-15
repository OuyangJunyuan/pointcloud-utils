#include <iostream>
#include <thread>
#include <chrono>

#include <gflags/gflags.h>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = boost::filesystem;
namespace viz = pcl::visualization;


DEFINE_string(DIR, "", "the dir/path .pcd files are stored");
DEFINE_double(POINT_SIZE, 1.0, "size of point");
DEFINE_bool(GRID_ON, true, "to show grid");
DEFINE_double(GRID_SIZE, 50, "to show grid");
DEFINE_double(GRID_CELL, 3, "to show grid");
DEFINE_double(GRID_Z, 0, "to show grid");


std::vector<std::string> pcd_files;
static int cnt = 0;

void show_pcd(pcl::visualization::PCLVisualizer *visualizer, const std::string &file) {
    visualizer->removeAllPointClouds();
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file, *points);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(points, "z");
    visualizer->addPointCloud(points, rgb, file);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, FLAGS_POINT_SIZE,
                                                 file);
};

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    auto *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym() == "s" && event.keyDown()) {
        if(cnt == pcd_files.size()){
            fprintf(stderr,"all .pcd files had been showed\n");
            exit(-1);
        }
        auto file = (fs::path(FLAGS_DIR) / pcd_files[cnt]);
        std::cout << "load " << file << std::endl;
        show_pcd(viewer, file.string());
        cnt++;
    }
};

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "pcd dir: " << FLAGS_DIR << std::endl;
    fs::path path(FLAGS_DIR);
    if (not exists(path)) {
        fprintf(stderr, "no file or dir %s exist\n", path.string().c_str());
        exit(-1);
    }


    viz::PCLVisualizer viz("simple viewer");
    viz.setBackgroundColor(0.5, 0.5, 0.5);
    viz.addCoordinateSystem(1);
    viz.initCameraParameters();
    viz.setCameraPosition(0,0,30,0,1,0);
    if (FLAGS_GRID_ON) {
        for (int i = -FLAGS_GRID_SIZE / FLAGS_GRID_CELL; i < FLAGS_GRID_SIZE / FLAGS_GRID_CELL; i++) {
            viz.addLine<pcl::PointXYZ>(pcl::PointXYZ(i * FLAGS_GRID_CELL, FLAGS_GRID_SIZE, FLAGS_GRID_Z),
                                       pcl::PointXYZ(i * FLAGS_GRID_CELL, -FLAGS_GRID_SIZE, FLAGS_GRID_Z),
                                       0, 0, 0, "line_x" + std::to_string(i));

            viz.addLine<pcl::PointXYZ>(pcl::PointXYZ(FLAGS_GRID_SIZE, i * FLAGS_GRID_CELL, FLAGS_GRID_Z),
                                       pcl::PointXYZ(-FLAGS_GRID_SIZE, i * FLAGS_GRID_CELL, FLAGS_GRID_Z),
                                       0, 0, 0, "line_y" + std::to_string(i));
        }
    }


    if (not is_directory(path)) {
        show_pcd(&viz, path.string());
    } else {
        fs::directory_iterator end_iter;
        for (fs::directory_iterator iter(path); iter != end_iter; ++iter) {
            if (!fs::is_directory(*iter) && (fs::extension(*iter) == ".pcd")) {
                pcd_files.push_back(iter->path().leaf().string());    //获取文件名
            }
        }
        if(pcd_files.empty()){
            fprintf(stderr,"no .pcd file was found in this directory\n");
            exit(-1);
        }
        viz.registerKeyboardCallback(keyboardEventOccurred, (void *) &viz);
        std::sort(pcd_files.begin(),pcd_files.end());
        cnt %= pcd_files.size();
        auto file = (fs::path(FLAGS_DIR) / pcd_files[cnt]);
        std::cout << "load " << file << std::endl;
        show_pcd(&viz, file.string());
        cnt++;
        std::cout << "press 's' to show next cloud" << std::endl;
    }

    while (!viz.wasStopped()) {
        viz.spinOnce(10);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    google::ShutDownCommandLineFlags();
    return 0;
}
