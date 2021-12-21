//
// Created by ou on 2021/10/10.
//

#include "pc_utils/seg/ground_estimator.h"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pc_utils/common/detail/factory.h"
#include "pc_utils/common/parameter.h"

#define PC_UTILS_CLASS_BASE_TYPE                GroundEstimator
#define PC_UTILS_CLASS_CONSTRUCTION             (const YAML::Node &)(const Params &)
#define PC_UTILS_TEMPLATE_SPECIALIZATION_LIST   PC_UTILS_GROUND_ESTIMATOR_TYPE

#define PC_UTILS_GROUND_ESTIMATOR_TYPE          \
define( RansacGroundEstimator               )   \
define( PatchWorkGroundEstimator            )   \
define( RingShapedElevationConjunctionMap   )

using namespace std;
using namespace YAML;

namespace pc_utils {


/**
 * @brief 基于平面拟合的地面估计
 * @tparam PointT: point type
 */
template<class PointT>
class RansacGroundEstimator final : PC_UTILS_BASE_LIST(RansacGroundEstimator) {
#define PC_UTILS_CLASS                  \
RansacGroundEstimator

#define PC_UTILS_MEMBER_VARIABLE        \
define(int  , max_iter_num, {100}   )   \
define(float, max_dis     , {0.5f}  )

#include "detail/member_define.h"

public:

    void estimate(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                  typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
                  typename pcl::PointCloud<PointT>::Ptr &cloud_no_ground) override {


        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(max_dis);
        seg.setMaxIterations(max_iter_num);
        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coeff);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_no_ground);
        extract.setNegative(false);
        extract.filter(*cloud_ground);
    };

};


/**
 * @brief Patchwork RAL2021 Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
 * @tparam PointT: point type
 */
template<typename PointT>
class PatchWorkGroundEstimatorPrivate {
#define PC_UTILS_CLASS                                                  \
PatchWorkGroundEstimatorPrivate

#define PC_UTILS_MEMBER_VARIABLE                                        \
define(double             , sensor_height                 , )           \
define(double             , max_range                     , )           \
define(double             , min_range                     , )           \
define(double             , num_zones                     , )           \
define(double             , adaptive_seed_selection_margin, )           \
define(double             , num_lpr                       , )           \
define(double             , th_seeds                      , )           \
define(double             , num_iter                      , )           \
define(double             , th_dist                       , )           \
define(double             , num_min_pts                   , )           \
define(double             , uprightness_thr               , )           \
define(double             , num_rings_of_interest         , )           \
define(std::vector<int>   , num_rings_each_zone           , )           \
define(std::vector<int>   , num_sectors_each_zone         , )           \
define(std::vector<double>, elevation_thr                 , )           \
define(std::vector<double>, flatness_thr                  , )

#include "detail/member_define.h"

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

    typedef std::vector<pcl::PointCloud<PointT> > Ring;
    typedef std::vector<Ring> Zone;


protected: // 参数设置
    double th_dist_d;
    bool verbose_ = false;

protected: // 过程变量

    vector<double> min_ranges;
    double min_range_z2_; // 12.3625
    double min_range_z3_; // 22.025
    double min_range_z4_; // 41.35
    vector<double> sector_sizes; // 每个环上扇区的角度大小
    vector<double> ring_sizes;  // 每个区域内ring的宽度大小


    float d_;
    Eigen::MatrixXf normal_;
    Eigen::VectorXf singular_values_;
    Eigen::Matrix3f cov_;
    Eigen::Vector4f pc_mean_;

    vector<Zone> ConcentricZoneModel_;

    pcl::PointCloud<PointT> revert_pc, reject_pc;
    pcl::PointCloud<PointT> ground_pc_;
    pcl::PointCloud<PointT> non_ground_pc_;

    pcl::PointCloud<PointT> regionwise_ground_;
    pcl::PointCloud<PointT> regionwise_nonground_;

    void initialize_zone(Zone &z, int num_sectors, int num_rings) {
        z.clear();
        pcl::PointCloud<PointT> cloud;
        Ring ring;
        // reserve 以免多次拷贝
        cloud.reserve(1000);
        ring.reserve(num_sectors);
        z.reserve(num_rings);

        // 每个环被分成num_sectors个扇区，ring是包含num_sectors个扇区内的点云的vector
        for (int i = 0; i < num_sectors; i++) {
            ring.emplace_back(cloud);
        }
        for (int j = 0; j < num_rings; j++) {
            z.emplace_back(ring);
        }
    }

    void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings) {
        for (int i = 0; i < num_sectors; i++) {
            for (int j = 0; j < num_rings; j++) {
                if (!patches[j][i].points.empty()) patches[j][i].points.clear();
            }
        }
    }

    double calc_principal_variance(const Eigen::Matrix3f &cov, const Eigen::Vector4f &centroid) {
        double angle = atan2(centroid(1, 0), centroid(0, 0)); // y, x
        double c = cos(angle);
        double s = sin(angle);
        double var_x_prime = c * c * cov(0, 0) + s * s * cov(1, 1) + 2 * c * s * cov(0, 1);
        double var_y_prime = s * s * cov(0, 0) + c * c * cov(1, 1) - 2 * c * s * cov(0, 1);
        return max(var_x_prime, var_y_prime);
    }

    inline double xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
        if (y >= 0) {
            return atan2(y, x); // 1, 2 quadrant
        } else {
            return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
        }
    }

    inline double xy2radius(const double &x, const double &y) {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    inline void pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm) {
        for (auto const &pt: src.points) {
            int ring_idx, sector_idx;
            double r = xy2radius(pt.x, pt.y);
            if ((r <= max_range) && (r > min_range)) {
                double theta = xy2theta(pt.x, pt.y);

                if (r < min_range_z2_) { // In First rings
                    ring_idx = min(static_cast<int>(((r - min_range) / ring_sizes[0])), num_rings_each_zone[0] - 1);
                    sector_idx = min(static_cast<int>((theta / sector_sizes[0])), num_sectors_each_zone[0] - 1);
                    czm[0][ring_idx][sector_idx].points.emplace_back(pt);
                } else if (r < min_range_z3_) {
                    ring_idx = min(static_cast<int>(((r - min_range_z2_) / ring_sizes[1])),
                                   num_rings_each_zone[1] - 1);
                    sector_idx = min(static_cast<int>((theta / sector_sizes[1])), num_sectors_each_zone[1] - 1);
                    czm[1][ring_idx][sector_idx].points.emplace_back(pt);
                } else if (r < min_range_z4_) {
                    ring_idx = min(static_cast<int>(((r - min_range_z3_) / ring_sizes[2])),
                                   num_rings_each_zone[2] - 1);
                    sector_idx = min(static_cast<int>((theta / sector_sizes[2])), num_sectors_each_zone[2] - 1);
                    czm[2][ring_idx][sector_idx].points.emplace_back(pt);
                } else { // Far!
                    ring_idx = min(static_cast<int>(((r - min_range_z4_) / ring_sizes[3])),
                                   num_rings_each_zone[3] - 1);
                    sector_idx = min(static_cast<int>((theta / sector_sizes[3])), num_sectors_each_zone[3] - 1);
                    czm[3][ring_idx][sector_idx].points.emplace_back(pt);
                }
            }

        }
    }

    void estimate_plane_(const pcl::PointCloud<PointT> &ground) {
        pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
        // Singular Value Decomposition: SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
        singular_values_ = svd.singularValues();

        // use the least singular vector as normal
        normal_ = (svd.matrixU().col(2));
        // mean ground seeds value
        Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

        // according to normal.T*[x,y,z] = -d
        d_ = -(normal_.transpose() * seeds_mean)(0, 0);
        // set distance threhold to `th_dist - d`
        th_dist_d = th_dist - d_;
    }

    void extract_piecewiseground(
            const int zone_idx, const pcl::PointCloud<PointT> &src,
            pcl::PointCloud<PointT> &dst,
            pcl::PointCloud<PointT> &non_ground_dst) {
        // 0. Initialization
        if (!ground_pc_.empty()) ground_pc_.clear();
        if (!dst.empty()) dst.clear();
        if (!non_ground_dst.empty()) non_ground_dst.clear();

        // 1. set seeds!
        extract_initial_seeds_(zone_idx, src, ground_pc_);

        // 2. Extract ground
        for (int i = 0; i < num_iter; i++) {
            estimate_plane_(ground_pc_);
            ground_pc_.clear();
            //TODO: = =可以用Eigen::Map 来做共享内存，减少拷贝。
            //pointcloud to matrix
            Eigen::MatrixXf points(src.points.size(), 3);
            int j = 0;
            for (auto &p: src.points) {
                points.row(j++) << p.x, p.y, p.z;
            }
            // ground plane model
            Eigen::VectorXf result = points * normal_;
            // threshold cloud_filter
            for (int r = 0; r < result.rows(); r++) {
                if (i < num_iter - 1) {
                    if (result[r] < th_dist_d) {
                        ground_pc_.points.push_back(src[r]);
                    }
                } else { // Final stage
                    if (result[r] < th_dist_d) {
                        dst.points.push_back(src[r]);
                    } else {
                        if (i == num_iter - 1) {
                            non_ground_dst.push_back(src[r]);
                        }
                    }
                }
            }
        }
    }

    void estimate_plane_(const int zone_idx, const pcl::PointCloud<PointT> &ground);

    void extract_initial_seeds_(
            const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
            pcl::PointCloud<PointT> &init_seeds) {
        init_seeds.points.clear();

        // LPR is the mean of low point representative
        double sum = 0;
        int cnt = 0;

        int init_idx = 0;
        if (zone_idx == 0) {
            for (int i = 0; i < p_sorted.points.size(); i++) {
                if (p_sorted.points[i].z < adaptive_seed_selection_margin * sensor_height) {
                    ++init_idx;
                } else {
                    break;
                }
            }
        }

        // Calculate the mean height value.
        for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr; i++) {
            sum += p_sorted.points[i].z;
            cnt++;
        }
        double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

        // iterate pointcloud, cloud_filter those height is less than lpr.height+th_seeds_
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < lpr_height + th_seeds) {
                init_seeds.points.push_back(p_sorted.points[i]);
            }
        }
    }

    void init() {
        if (num_zones != 4 || num_sectors_each_zone.size() != num_rings_each_zone.size()) {
            throw invalid_argument("Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone");
        }
        if (elevation_thr.size() != flatness_thr.size()) {
            throw invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
        }

        num_rings_of_interest = elevation_thr.size();

        revert_pc.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        non_ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);


        min_range_z2_ = (7 * min_range + max_range) / 8.0;
        min_range_z3_ = (3 * min_range + max_range) / 4.0;
        min_range_z4_ = (min_range + max_range) / 2.0;

        min_ranges = {min_range, min_range_z2_, min_range_z3_, min_range_z4_};

        ring_sizes = {(min_range_z2_ - min_range) / num_rings_each_zone.at(0),
                      (min_range_z3_ - min_range_z2_) / num_rings_each_zone.at(1),
                      (min_range_z4_ - min_range_z3_) / num_rings_each_zone.at(2),
                      (max_range - min_range_z4_) / num_rings_each_zone.at(3)};

        sector_sizes = {2 * M_PI / num_sectors_each_zone.at(0),
                        2 * M_PI / num_sectors_each_zone.at(1),
                        2 * M_PI / num_sectors_each_zone.at(2),
                        2 * M_PI / num_sectors_each_zone.at(3)};


        // 一个区域分成num_rings个扇环，每个扇环被分成num_sectors个扇区
        for (int iter = 0; iter < num_zones; ++iter) {
            Zone z;
            initialize_zone(z, num_sectors_each_zone.at(iter), num_rings_each_zone.at(iter));
            ConcentricZoneModel_.push_back(z);
        }
    }

public:

    void estimate(
            const typename pcl::PointCloud<PointT>::Ptr &cloud_input,
            typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
            typename pcl::PointCloud<PointT>::Ptr &cloud_no_ground) {
        const pcl::PointCloud<PointT> &cloud_in = *cloud_input;
        pcl::PointCloud<PointT> &cloud_out = *cloud_ground;
        pcl::PointCloud<PointT> &cloud_nonground = *cloud_no_ground;

        double t_total_ground = 0.0;
        double t_total_estimate = 0.0;

        // 1.Msg to pointcloud
        pcl::PointCloud<PointT> laserCloudIn;
        laserCloudIn = cloud_in;


        // 2.Sort on Z-axis value.
        sort(laserCloudIn.points.begin(), laserCloudIn.end(),
             [](const auto &pa, const auto &pb) { return pa.z < pb.z; });

        // 3.Error point removal
        // As there are some error mirror reflection under the ground,
        // here regardless point under 1.8* sensor_height
        // Sort point according to height, here uses z-axis in default
        auto it = laserCloudIn.points.begin();
        for (int i = 0; i < laserCloudIn.points.size(); i++) {
            if (laserCloudIn.points[i].z < -1.8 * sensor_height) {
                it++;
            } else {
                break;
            }
        }
        // 因为按z排序了，所以直接过滤 从0到阈值以下的点。
        laserCloudIn.points.erase(laserCloudIn.points.begin(), it);


        // 4. pointcloud -> regionwise setting
        for (int k = 0; k < num_zones; ++k) {
            flush_patches_in_zone(ConcentricZoneModel_[k], num_sectors_each_zone[k], num_rings_each_zone[k]);
        }
        pc2czm(laserCloudIn, ConcentricZoneModel_);

        cloud_out.clear();
        cloud_nonground.clear();
        revert_pc.clear();
        reject_pc.clear();

        int concentric_idx = 0;
        // TODO: 多个zone是否可以多线程处理？
        for (int k = 0; k < num_zones; ++k) {
            //TODO: 为何不用  auto &zone = ConcentricZoneModel_[k];
            const auto &zone = ConcentricZoneModel_[k];
            for (uint16_t ring_idx = 0; ring_idx < num_rings_each_zone[k]; ++ring_idx) { //{2, 4, 4, 4};
                for (uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone[k]; ++sector_idx) { //{16, 32, 54, 32};
                    // 遍历每个扇区
                    if (zone[ring_idx][sector_idx].points.size() > num_min_pts) {
                        extract_piecewiseground(k, zone[ring_idx][sector_idx], regionwise_ground_,
                                                regionwise_nonground_);

                        // Status of each patch
                        // used in checking uprightness, elevation, and flatness, respectively
                        const double ground_z_vec = abs(normal_(2, 0));
                        const double ground_z_elevation = pc_mean_(2, 0);
                        const double surface_variable =
                                singular_values_.minCoeff() /
                                (singular_values_(0) + singular_values_(1) + singular_values_(2));
                        // 地面垂直度过低，不太垂直，则吧所有点都认为是非地面点
                        if (ground_z_vec < uprightness_thr) {
                            // All points are rejected
                            cloud_nonground += regionwise_ground_;
                            cloud_nonground += regionwise_nonground_;
                        } else { // satisfy uprightness
                            if (concentric_idx < num_rings_of_interest) {
                                if (ground_z_elevation > elevation_thr[ring_idx + 2 * k]) {
                                    if (flatness_thr[ring_idx + 2 * k] > surface_variable) {
                                        if (verbose_) {
                                            std::cout << "\033[1;36m[Flatness] Recovery operated. Check "
                                                      << ring_idx + 2 * k
                                                      << "th param. flatness_thr_: " << flatness_thr[ring_idx + 2 * k]
                                                      << " > "
                                                      << surface_variable << "\033[0m" << std::endl;
                                            revert_pc += regionwise_ground_;
                                        }
                                        cloud_out += regionwise_ground_;
                                        cloud_nonground += regionwise_nonground_;
                                    } else {
                                        if (verbose_) {
                                            std::cout << "\033[1;34m[Elevation] Rejection operated. Check "
                                                      << ring_idx + 2 * k
                                                      << "th param. of elevation_thr_: "
                                                      << elevation_thr[ring_idx + 2 * k]
                                                      << " < "
                                                      << ground_z_elevation << "\033[0m" << std::endl;
                                            reject_pc += regionwise_ground_;
                                        }
                                        cloud_nonground += regionwise_ground_;
                                        cloud_nonground += regionwise_nonground_;
                                    }
                                } else {
                                    cloud_out += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                }
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        }
                    }
                }
                ++concentric_idx;
            }
        }
    }

};

template<typename PointT>
class PatchWorkGroundEstimator final :
        PC_UTILS_BASE_LIST(PatchWorkGroundEstimator),
        public PatchWorkGroundEstimatorPrivate<PointT> {
#define PC_UTILS_CLASS                  \
PatchWorkGroundEstimator
public:
#include "detail/constructor_default.h"
#include "detail/others_define.h"

public:
    explicit PatchWorkGroundEstimator(const YAML::Node &params) : PatchWorkGroundEstimatorPrivate<PointT>(params) {
        PatchWorkGroundEstimatorPrivate<PointT>::init();
    }

    explicit PatchWorkGroundEstimator(const Params &params) : PatchWorkGroundEstimatorPrivate<PointT>(params) {
        PatchWorkGroundEstimatorPrivate<PointT>::init();
    }

    void estimate(
            const typename pcl::PointCloud<PointT>::Ptr &cloud_input,
            typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
            typename pcl::PointCloud<PointT>::Ptr &cloud_no_ground) override {
        PatchWorkGroundEstimatorPrivate<PointT>::estimate(cloud_input, cloud_ground, cloud_no_ground);
    }
};


/**
 * @brief 基于环形联合高程图的地面分割算法
 * @brief RingShapedElevationConjunctionMap
 * @tparam PointT: point type
 */
template<class PointT>
class RingShapedElevationConjunctionMap final : PC_UTILS_BASE_LIST(RingShapedElevationConjunctionMap) {
#define PC_UTILS_CLASS                          \
RingShapedElevationConjunctionMap

#define PC_UTILS_MEMBER_VARIABLE                \
define(float, sensor_height, {0.f}          )   \
define(float, th_g         , {0.2f}         )   \
define(float, max_slope    , {M_PIf32/12}   )   \
define(float, delta_ring   , {2.0f}         )   \
define(float, min_ring     , {0.f}          )   \
define(float, max_ring     , {50.f}         )   \
define(int  , num_scan     , {36}           )


    struct RegionField {
        int point_id{-1}, ring_id{-1}, scan_id{-1}, ind{-1};
    };
    int num_ring{0}, num_grid{0}, num_points{0};
    std::vector<float> recm;
    std::vector<int> ground, non_ground;
    std::vector<RegionField> indices;

    inline int get_ring_ind(const float &range) const {
        return static_cast<int>(floorf((range - min_ring) / delta_ring));
    }

    inline float get_xoy_angle(const float &y, const float &x) const {
        static float pix2 = 2.0f * M_PIf32;
        auto xoy_angle = atan2f(y, x);
        return xoy_angle < 0 ? xoy_angle + pix2 : xoy_angle;
    }

    inline int get_scan_ind(const float &y, const float &x) const {
        static float pix2 = 2.0f * M_PIf32;
        return static_cast<int>(floorf(num_scan * (get_xoy_angle(y, x) / pix2)));
    }

    inline void reset() {
        indices.clear();
        ground.clear();
        non_ground.clear();
        indices.reserve(num_points);
        ground.reserve(num_points);
        non_ground.reserve(num_points);
        recm.resize(num_grid, FLT_MAX);
    }

#include "detail/member_define.h"

public:

    void estimate(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                  typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
                  typename pcl::PointCloud<PointT>::Ptr &cloud_no_ground) override {

        num_points = static_cast<int>(cloud_in->size());
        num_ring = static_cast<int>(floorf(max_ring - min_ring) / delta_ring);
        num_grid = num_ring * num_scan;

        reset();

        for (int i = 0; i < num_points; ++i) {
            auto &pt = cloud_in->points[i];
            auto r = sqrtf(pt.x * pt.x + pt.y * pt.y);
            if (min_ring < r and r < max_ring) {
                auto ring_ind = get_ring_ind(r);
                auto scan_ind = get_scan_ind(pt.y, pt.x);
                auto ind = ring_ind + scan_ind * num_ring; // ring优先存储.
                indices.push_back({i, ring_ind, scan_ind, ind});

                auto &lowest_in_this_grid = recm[ind];
                lowest_in_this_grid = std::fmin(lowest_in_this_grid, pt.z);
            }
        }

        float delta_ring_x_tan_max_slope = delta_ring * tanf(max_slope);
        float lowest_in_previous_grid = -sensor_height;
        for (int i = 0; i < num_grid; ++i) {
            if (i % num_ring != 0) {
                recm[i] = fmin(recm[i], lowest_in_previous_grid + delta_ring_x_tan_max_slope);
            }
            lowest_in_previous_grid = recm[i];
        }

        for (auto &&rn: indices) {
            if (cloud_in->points[rn.point_id].z < recm[rn.ind] + th_g) {
                ground.push_back(rn.point_id);
            } else {
                non_ground.push_back(rn.point_id);
            }
        }
        cloud_ground = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*cloud_in, ground));
        cloud_no_ground = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*cloud_in, non_ground));
    };

};

}  // namespace pc_utils
#include "detail/template_specialization.h"