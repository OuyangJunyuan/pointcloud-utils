# PointCloud Utils

A package for convenient pointcloud processing, implemented by abstract factory pattern. And it's supported to build from a variety of ways

# Install

cmake install has not been implemented yet, only as cmake subdirectory was supported.

note: c++17 standard needed.

# Introduction

* Filter:
  * PassThroughFilter 
  * CropAABoxFilter[pcl wrapper]
  * CropOBoxFilter[pcl wrapper]
  * RegionOfInterestFilter 
  * ApproximateVoxelFilter[pcl wrapper]
  * SelfFilter 
  * MaxPointCountFilter 
  * RandomSamplingFilter 
  * RemoveNaNFilter[pcl wrapper]
  * Filters
* Cluster:
  * EuclideanCluster[pcl wrapper]
  * CurvedVoxelCluster[[IROS2019: Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance]](https://ieeexplore.ieee.org/abstract/document/8968026)
* GroundSegmentation:
  * RansacGroundEstimator[pcl wrapper]
  * PatchWorkGroundEstimator[[2021 RAL: Patchwork: Concentric Zone-Based Region-Wise Ground Segmentation With Ground Likelihood Estimation Using a 3D LiDAR Sensor]](https://ieeexplore.ieee.org/abstract/document/9466396)
* bounding:
  * bounding box:[PCA]

todo:

* cluster
  * JCP
* GroundSegmentation
  * range-based segmentation
  * vertical analysis groud remove
* bounding:
  * Polygon bounding  
  * Capsule bounding 
* ...

# Simple example

usage of all these methods can be found in test cases `test/test_pc.cpp`.

* filter

  you wanna specific filter, and can type as following:

  ```c++
   // A [CloudFilter<PXYZ>] factory function and   is overloaded [const Params &]
  using FilterFactoryParams = Factory<pc_utils::CloudFilter<PXYZ>, const Params &>; 
  
  // set params [max_x,max_y,max_z,negative] by using [Params] interface
  auto filter1 = FilterFactoryParams::BuildT(pc_utils::ns("PassThroughFilter"),
                                                    Params{
                                                            {"max_x",    5.0f},
                                                            {"max_y",    5.0f},
                                                            {"max_z",    5.0f},
                                                            {"negative", false}})
                                                       )
      
  // or you can use yaml to config params                                            
  /**
  *config.yaml:
  *---------------------
  *CloudFilter:
  *	PassThroughFilter: { max_x: 5, max_y: 5, max_z: 5,negative: false }
  *	CropAABoxFilter: { min_x: -5, min_y: -5, min_z: -5, max_x: 5, max_y: 5, max_z: 5,negative: false }
  *	RegionOfInterestFilter: { min_r: 1, max_r: 5,negative: false }
  *	CropOBoxFilter: { pose: [ 0, 0, 0, 1, 0, 0, 0 ], dxyz: [ 15, 15, 15 ], negative: false }
  *	ApproximateVoxelFilter: { leaf_x: 1, leaf_y: 1, leaf_z: 1 }
  */
  using FilterFactoryYAML = Factory<pc_utils::CloudFilter<PXYZ>, const YAML::Node &>; 
  auto cfg = YAML::LoadFIle("config.yaml")["CloudFilter"];
  auto filter2 = FilterFactoryYAML::BuildT(pc_utils::ns("PassThroughFilter"),cfg["PassThroughFilter"]);
  
  // how to use filter?
  PCXYZPtr output(new PCXYZ);
  filter->filter(input, output);
  ```

* filter chains

  ```c++
  /**
   * config.yaml
   * -----
   * Chain:
   *  - RemoveNaNFilter: { }
   *  - RandomSamplingFilter: { prob: 0.75, method: 0 }
   *  - PassThroughFilter: { max_x: 25, max_y: 25, max_z: 10, negative: false }
   *  - CropAABoxFilter: { min_x: -15, min_y: -15, min_z: -5, max_x: 15, max_y: 15, max_z: 5,negative: false }
   *  - CropOBoxFilter: { pose: [ 0, 0, 0, 0, 0, 0.78 ], dxyz: [ 30, 30, 30 ], negative: false }
   *  - RegionOfInterestFilter: { min_r: 7, max_r: 10, negative: true }
   *  - ApproximateVoxelFilter: { leaf_x: 0.1, leaf_y: 0.1, leaf_z: 0.1 }
   *  - MaxPointCountFilter: { count: 10000 }
   */
  PCXYZPtr input(new PCXYZ), output(new PCXYZ);
  pcl::io::loadPCDFile(ROOT_PATH "/resource/000000.pcd", *input);
  
  YAML::Node config = YAML::LoadFile(ROOT_PATH  "/config/config.yaml")["CloudFilter"]["Chain"];
  auto filters = Factory<pc_utils::CloudFilter<PXYZ>, const YAML::Node &>::BuildT(pc_utils::ns("Filters"), config);
  // points was filtered in yaml declare order
  filters->filter(input, output);
  ```

* bounding box

  ```c++
  auto box_extractor = BoundFactory::BuildT<std::shared_ptr>(pc_utils::ns("OrientedBBox2p5DExtractor"));
  BoundingBox box;
  box_extractor->extract(input, box);
  ```

* ...

