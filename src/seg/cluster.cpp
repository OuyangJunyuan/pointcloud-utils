//
// Created by nrsl on 2021/10/11.
//

#include "pc_utils/seg/cluster.h"

#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pc_utils/common/detail/factory.h"
#include "pc_utils/common/parameter.h"


#define PC_UTILS_CLASS_BASE_TYPE                Cluster
#define PC_UTILS_CLASS_CONSTRUCTION             (const YAML::Node &)(const Params &)
#define PC_UTILS_TEMPLATE_SPECIALIZATION_LIST   PC_UTILS_CLUSTER_TYPE

using namespace YAML;
using namespace std;

namespace pc_utils {
/**
 * @breif pcl 提供的欧氏距离聚类
 * @tparam PointT: point type
 */
template<class PointT>
class EuclideanCluster final : PC_UTILS_BASE_LIST(EuclideanCluster) {
#define PC_UTILS_CLASS                  \
EuclideanCluster

#define PC_UTILS_MEMBER_VARIABLE        \
define(float, min_dis   , {1.0f})       \
define(float, min_points, {10})

#include "detail/member_define.h"

public:
    void extract(const typename PC<PointT>::Ptr &cloud_in,
                 std::vector<int> &cluster_indices, std::vector<int> &cluster_id) override {
        if (min_dis < 0 and min_points < 0) {
            throw std::invalid_argument("[min_dis] or [min_points] must more than zero");
            return;
        }

        typename pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
        std::vector<pcl::PointIndices> indices;
        pcl::EuclideanClusterExtraction<PointT> ec;

        kdTree->setInputCloud(cloud_in);
        ec.setSearchMethod(kdTree);
        ec.setInputCloud(cloud_in);
        ec.setClusterTolerance(min_dis);
        ec.setMinClusterSize(min_points);
        ec.extract(indices);

        cluster_indices.resize(cloud_in->size(), 0);
        cluster_id.resize(indices.size(), 0);

        int label = 0;
        for (auto &cluster: indices) {
            cluster_id.push_back(label);
            for (auto &idx: cluster.indices) {
                cluster_indices[idx] = label;
            }
            label++;
        }
    }
};

/**
 * @breif 曲面体素聚类
 * @tparam PointT: point type
 */
template<typename PointT>
class CurvedVoxelCluster final : PC_UTILS_BASE_LIST(CurvedVoxelCluster) {
#define PC_UTILS_CLASS  CurvedVoxelCluster

#define PC_UTILS_MEMBER_VARIABLE        \
define(float, deltaA     , {2.0f}   )   \
define(float, deltaR     , {0.35}   )   \
define(float, deltaP     , {1.2f}   )   \
define(int  , min_points , {10}     )   \
define(float, max_azimuth, {24.0f}  )   \
define(float, min_azimuth, {-24.0f} )

#include "detail/member_define.h"

    struct PointAPR {
        float azimuth;
        float polar_angle;
        float range;
    };

    struct Voxel {
        bool haspoint = false;
        int cluster = -1;
        std::vector<int> index;
    };

    float min_range_ = std::numeric_limits<float>::max();
    float max_range_ = std::numeric_limits<float>::min();
    int length_ = 0;
    int width_ = 0;
    int height_ = 0;

    //upper sort
    static inline bool compare_cluster(std::pair<int, int> a, std::pair<int, int> b) {
        return a.second > b.second;
    }

    static inline float Polar_angle_cal(float x, float y) {
        float temp_tangle = 0;
        if (x == 0 && y == 0) {
            temp_tangle = 0;
        } else if (y >= 0) {
            temp_tangle = (float) atan2(y, x);
        } else if (y < 0) {
            temp_tangle = (float) atan2(y, x) + 2 * M_PI;
        }
        return temp_tangle;
    }

    void calculateAPR(const pcl::PointCloud<PointT> &cloud_IN, std::vector<PointAPR> &vapr) {
        for (int i = 0; i < cloud_IN.points.size(); ++i) {
            PointAPR par;
            par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
            par.range = sqrt(cloud_IN.points[i].x * cloud_IN.points[i].x + cloud_IN.points[i].y * cloud_IN.points[i].y);
            par.azimuth = (float) atan2(cloud_IN.points[i].z, par.range);
            if (par.range < min_range_) {
                min_range_ = par.range;
            }
            if (par.range > max_range_) {
                max_range_ = par.range;
            }
            vapr.push_back(par);
        }
        length_ = int((max_range_ - min_range_) / deltaR) + 1;
        width_ = round(360 / deltaP);
        height_ = (max_azimuth - min_azimuth) / deltaA + 1;
    }

    void build_hash_table(const std::vector<PointAPR> &vapr, std::unordered_map<int, Voxel> &map_out) {

        std::vector<int> ri;
        std::vector<int> pi;
        std::vector<int> ai;
        for (int i = 0; i < vapr.size(); ++i) {
            int azimuth_index = (vapr[i].azimuth - min_azimuth) / deltaA;
            int polar_index = int(vapr[i].polar_angle * 180 / M_PI / deltaP);
            int range_index = int((vapr[i].range - min_range_) / deltaR);

            int voxel_index = (polar_index * (length_) + range_index) + azimuth_index * (length_) * (width_);
            ri.push_back(range_index);
            pi.push_back(polar_index);
            ai.push_back(azimuth_index);
            typename std::unordered_map<int, Voxel>::iterator it_find;
            it_find = map_out.find(voxel_index);
            if (it_find != map_out.end()) {
                it_find->second.index.push_back(i);

            } else {
                Voxel vox;
                vox.haspoint = true;
                vox.index.push_back(i);
                vox.index.swap(vox.index);
                map_out.insert(std::make_pair(voxel_index, vox));
            }

        }
        auto maxPosition = max_element(ai.begin(), ai.end());
        auto maxPosition1 = max_element(ri.begin(), ri.end());
        auto maxPosition2 = max_element(pi.begin(), pi.end());
    }

    void find_neighbors(int polar, int range, int azimuth, std::vector<int> &neighborindex) {

        for (int z = azimuth - 1; z <= azimuth + 1; z++) {
            if (z < 0 || z > (height_ - 1)) {
                continue;
            }

            for (int y = range - 1; y <= range + 1; y++) {
                if (y < 0 || y > (length_ - 1)) {
                    continue;
                }

                for (int x = polar - 1; x <= polar + 1; x++) {
                    int px = x;
                    if (x < 0) {
                        px = width_ - 1;
                    }
                    if (x > (width_ - 1)) {
                        px = 0;

                    }
                    neighborindex.push_back((px * (length_) + y) + z * (length_) * (width_));
                }
            }
        }
    }

    bool most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index) {
        std::unordered_map<int, int> histcounts;
        for (int i = 0; i < values.size(); i++) {
            if (histcounts.find(values[i]) == histcounts.end()) {
                histcounts[values[i]] = 1;
            } else {
                histcounts[values[i]] += 1;
            }
        }
        int max = 0, maxi;
        std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
        sort(tr.begin(), tr.end(), compare_cluster);
        for (int i = 0; i < tr.size(); ++i) {
            if (tr[i].second > min_points) {
                cluster_index.push_back(tr[i].first);
            }
        }

        return true;
    }

    void mergeClusters(std::vector<int> &cluster_indices, int idx1, int idx2) {
        for (int i = 0; i < cluster_indices.size(); i++) {
            if (cluster_indices[i] == idx1) {
                cluster_indices[i] = idx2;
            }
        }
    }

    std::vector<int> cluster(std::unordered_map<int, Voxel> &map_in, const std::vector<PointAPR> &vapr) {
        int current_cluster = 0;
        std::vector<int> cluster_indices = std::vector<int>(vapr.size(), -1);

        for (int i = 0; i < vapr.size(); ++i) {

            if (cluster_indices[i] != -1)
                continue;
            int azimuth_index = (vapr[i].azimuth - min_azimuth) / deltaA;
            int polar_index = int(vapr[i].polar_angle * 180 / M_PI / deltaP);
            int range_index = int((vapr[i].range - min_range_) / deltaR);
            int voxel_index = (polar_index * (length_) + range_index) + azimuth_index * (length_) * (width_);

            typename std::unordered_map<int, Voxel>::iterator it_find;
            typename std::unordered_map<int, Voxel>::iterator it_find2;

            it_find = map_in.find(voxel_index);
            std::vector<int> neightbors;

            if (it_find != map_in.end()) {

                std::vector<int> neighborid;
                find_neighbors(polar_index, range_index, azimuth_index, neighborid);
                for (int k = 0; k < neighborid.size(); ++k) {

                    it_find2 = map_in.find(neighborid[k]);

                    if (it_find2 != map_in.end()) {

                        for (int j = 0; j < it_find2->second.index.size(); ++j) {
                            neightbors.push_back(it_find2->second.index[j]);
                        }
                    }
                }
            }

            neightbors.swap(neightbors);

            if (neightbors.size() > 0) {
                for (int j = 0; j < neightbors.size(); ++j) {
                    int oc = cluster_indices[i];
                    int nc = cluster_indices[neightbors[j]];
                    if (oc != -1 && nc != -1) {
                        if (oc != nc)
                            mergeClusters(cluster_indices, oc, nc);
                    } else {
                        if (nc != -1) {
                            cluster_indices[i] = nc;
                        } else {
                            if (oc != -1) {
                                cluster_indices[neightbors[j]] = oc;
                            }
                        }
                    }

                }
            }

            if (cluster_indices[i] == -1) {
                current_cluster++;
                cluster_indices[i] = current_cluster;
                for (int s = 0; s < neightbors.size(); ++s) {
                    cluster_indices[neightbors[s]] = current_cluster;
                }
            }
        }
        return cluster_indices;
    }

public:

    void extract(const typename PC<PointT>::Ptr &cloud_in,
                 std::vector<int> &cluster_indices,
                 std::vector<int> &cluster_id) {
        std::vector<PointAPR> capr;
        std::unordered_map<int, Voxel> hash_table;

        const pcl::PointCloud<PointT> &cluster_point = *cloud_in;
        capr.reserve(cluster_point.size());
        calculateAPR(cluster_point, capr);
        build_hash_table(capr, hash_table);
        cluster_indices = cluster(hash_table, capr);
        most_frequent_value(cluster_indices, cluster_id);
    }

};


}   // namespace pc_utils
#include "detail/template_specialization.h"
