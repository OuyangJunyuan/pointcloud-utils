//
// Created by ou on 2021/11/16.
//

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <dbg.h>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pc_utils/common/detail/factory.h>
#include <pc_utils/common/parameter.h>
#include <pc_utils/seg/cluster.h>
#include <pc_utils/seg/ground_estimator.h>
#include <pc_utils/bound/box_extract.h>
#include <pc_utils/filter/cloud_filter.h>


using namespace pc_utils;

template<class T>
void display_filter_result(const std::string &name, T &&input, T &&output);

template<class T>
void display_ground_seg_result(const std::string &name, T &&input, T &&output);

struct Timer {
    std::string title;
    std::chrono::steady_clock::time_point t1;

    Timer(const char *_title) {
        title = std::string(_title);
        t1 = std::chrono::steady_clock::now();
    }

    ~ Timer() {
        auto t2 = std::chrono::steady_clock::now();
        std::cout << title << " : " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000
                  << " ms\n";
    };
};


TEST(pc_utils, parameters) {
    struct TestParam {
        TestParam(const Params &params) {
            xMax = any_lexical_cast<double>(params.at("xMax"));
            yMax = any_lexical_cast<double>(params.at("yMax"));
            threshold = any_lexical_cast<std::vector<double>>(params.at("threshold"));
            printf("%lf\n", xMax);
            printf("%lf\n", yMax);
            printf("%lf\n", threshold[0]);
        }

        double xMax{0};
        double yMax{0};
        std::vector<double> threshold;
    };
    TestParam test(
            {
                    {"xMax",      5.0},
                    {"yMax",      "inf"},
                    {"threshold", std::vector<double>{1}}
            });
}

TEST(pc_utils, filter_chain) {
    using FilterFromYaml = Factory<pc_utils::CloudFilter<PXYZ>, const YAML::Node &>;


    PCXYZPtr input(new PCXYZ);
    pcl::io::loadPCDFile(ROOT_PATH "/resource/000000.pcd", *input);
    YAML::Node config = YAML::LoadFile(ROOT_PATH  "/config/config.yaml")["CloudFilter"]["Chain"];
    auto filters = FilterFromYaml::BuildT(pc_utils::ns("Filters"), config);
    PCXYZPtr output(new PCXYZ);
    filters->filter(input, output);
    display_filter_result(filters->class_name(), input, output);
}

TEST(pc_utils, filter_from_params) {
    using FilterFactoryParams = Factory<pc_utils::CloudFilter<PXYZ>, const Params &>;

    PCXYZPtr input(new PCXYZ);
    pcl::io::loadPCDFile(ROOT_PATH "/resource/000000.pcd", *input);
    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("PassThroughFilter"),
                                                  Params{
                                                          {"max_x",    5.0f},
                                                          {"max_y",    5.0f},
                                                          {"max_z",    5.0f},
                                                          {"negative", false}});filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }

    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("CropAABoxFilter"),
                                                  Params{
                                                          {"max_x",    "inf"},
                                                          {"min_x",    "-10.0"},
                                                          {"max_y",    0.0f},
                                                          {"min_y",    -10.0f},
                                                          {"max_z",    0.0f},
                                                          {"min_z",    -10.0f},
                                                          {"negative", false}});filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }


    auto pose = Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) * Eigen::Isometry3f::Identity();
    auto dxyz = Eigen::Vector3f{15, 15, 15};
    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("CropOBoxFilter"),
                                                  Params{
                                                          {"pose",     pose},
                                                          {"dxyz",     dxyz},
                                                          {"negative", false}
                                                  });filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }

    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("RegionOfInterestFilter"),
                                                  Params{
                                                          {"max_r",    10.0f},
                                                          {"min_r",    "0.0"},
                                                          {"negative", true}
                                                  });filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }

    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("ApproximateVoxelFilter"),
                                                  Params{
                                                          {"leaf_x", "1.0"},
                                                          {"leaf_y", "1.0"},
                                                          {"leaf_z", "1.0"}
                                                  });filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }


    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("MaxPointCountFilter"),
                                                  Params{{"count", "10000"}});filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }


    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("RandomSamplingFilter"),
                                                  Params{{"prob",   "0.25"},
                                                         {"method", "1"}});filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }

    if (auto filter = FilterFactoryParams::BuildT(pc_utils::ns("RemoveNaNFilter"), Params{});filter) {
        PCXYZPtr output(new PCXYZ);
        filter->filter(input, output);
        display_filter_result(filter->class_name(), input, output);
    }
}

TEST(pc_utils, boundingbox3d) {
    using BoundFactory = Factory<pc_utils::BoundingExtractor<PXYZ>>;
    PCXYZPtr input(new PCXYZ);
    pcl::io::loadPCDFile(ROOT_PATH "/resource/000000.pcd", *input);
    if (auto box_extractor = BoundFactory::BuildT<std::shared_ptr>(
                pc_utils::ns("OrientedBBox2p5DExtractor"));box_extractor) {

        BoundingBox box;
        box_extractor->extract(input, box);
        std::cout << box.pose.translation().transpose() << ", " << box.dxyz.transpose() << std::endl;
    }
}

TEST(pc_utils, ground_seg) {
    auto config = YAML::LoadFile(ROOT_PATH  "/config/config.yaml")["GroundEstimator"];
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            points(new pcl::PointCloud<pcl::PointXYZ>),
            ground(new pcl::PointCloud<pcl::PointXYZ>),
            non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(ROOT_PATH "/resource/000000.pcd", *points);
    using GroundSegFactory = Factory<pc_utils::GroundEstimator<PXYZ>, const YAML::Node &>;

    if (auto filter = GroundSegFactory::BuildT<std::shared_ptr>(pc_utils::ns("RansacGroundEstimator"),
                                                                config["RansacGroundEstimator"]);filter) {
        {
            Timer timer("RansacGroundEstimator");
            filter->estimate(points, ground, non_ground);
        }
        display_ground_seg_result("RansacGroundEstimator", points, ground);
    }

    if (auto filter = GroundSegFactory::BuildT<std::shared_ptr>(pc_utils::ns("PatchWorkGroundEstimator"),
                                                                config["PatchWorkGroundEstimator"]);filter) {
        {
            Timer timer("PatchWorkGroundEstimator");
            filter->estimate(points, ground, non_ground);
        }
        display_ground_seg_result("PatchWorkGroundEstimator", points, ground);
    }

    if (auto filter = GroundSegFactory::BuildT<std::shared_ptr>(pc_utils::ns("RingShapedElevationConjunctionMap"),
                                                                config["RingShapedElevationConjunctionMap"]);filter) {
        {
            Timer timer("RingShapedElevationConjunctionMap");
            filter->estimate(points, ground, non_ground);
        }
        display_ground_seg_result("RingShapedElevationConjunctionMap", points, ground);
    }
}


#include <random>

TEST(pc_utils, cluster) {
    auto config = YAML::LoadFile(ROOT_PATH  "/config/config.yaml")["Cluster"];
    using ClusterFactory = Factory<pc_utils::Cluster<PXYZ>, const YAML::Node &>;

    PCXYZPtr points(new PCXYZ);

    for (int i = 0; i < 10; i++) {
        float rand = 1.0 * std::rand() / RAND_MAX;
        points->push_back({5.f + rand, 5.f + rand, 1});
        points->push_back({5.f + rand, -5.f + rand, 1});
        points->push_back({-5.f + rand, 5.f + rand, 1});
        points->push_back({-5.f + rand, -5.f + rand, 1});
    }

    if (auto filter = ClusterFactory::BuildT<std::shared_ptr>(pc_utils::ns("EuclideanCluster"),
                                                              config["EuclideanCluster"]);filter) {
        std::vector<int> id, label;
        filter->extract(points, label, id);
        std::cout << points->size() << ", label: " << label.size() << ", id: " << id.size() << std::endl;
    }

    if (auto filter = ClusterFactory::BuildT<std::shared_ptr>(pc_utils::ns("CurvedVoxelCluster"),
                                                              config["CurvedVoxelCluster"]);filter) {
        std::vector<int> id, label;
        filter->extract(points, label, id);
        std::cout << points->size() << ", label: " << label.size() << ", id: " << id.size() << std::endl;
    }
}

template<class T>
void display_filter_result(const std::string &name, T &&input, T &&output) {
    std::cout << "[" + name + "]: " << input->size() << " pts -> " << output->size() << " pts" << std::endl;
    pcl::visualization::PCLVisualizer viewer(name);
    viewer.resetCamera();
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.addCoordinateSystem();
    viewer.addPointCloud(input, "input");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(input, 0, 255, 0);
    viewer.addPointCloud(output, output_color, "output");
    viewer.spin();
    viewer.close();
}

template<class T>
void display_ground_seg_result(const std::string &name, T &&input, T &&output) {
    std::cout << "[" + name + "]: " << input->size() << " pts -> " << output->size() << " pts" << std::endl;
    pcl::visualization::PCLVisualizer viewer(name);
    viewer.resetCamera();
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.addCoordinateSystem();
    viewer.addPointCloud(input, "input");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(input, 0, 255, 0);
    viewer.addPointCloud(output, output_color, "output");
    viewer.spin();
    viewer.close();
}