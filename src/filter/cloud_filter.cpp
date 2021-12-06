//
// Created by nrsl on 2021/10/10.
//
#include "pc_utils/filter/cloud_filter.h"

#include <random>
#include <yaml-cpp/yaml.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "pc_utils/common/factory.h"
#include "pc_utils/common/parameter.h"
#include "pc_utils/bound/box_extract.h"

#define PC_UTILS_CLASS_BASE_TYPE                CloudFilter
#define PC_UTILS_CLASS_CONSTRUCTION             (const YAML::Node &)(const Params &)

using namespace std;
namespace YAML {
template<>
struct convert<Eigen::Isometry3f> {
    static bool decode(const Node &node, Eigen::Isometry3f &pose) {
        if (node.IsSequence()) {
            pose.setIdentity();
            if (node.size() == 6) {
                pose.translate(Eigen::Vector3f(
                        node[0].as<float>(),
                        node[1].as<float>(),
                        node[2].as<float>()));
                Eigen::AngleAxisf
                        r(node[3].as<float>(), Eigen::Vector3f::UnitX()),
                        p(node[4].as<float>(), Eigen::Vector3f::UnitY()),
                        y(node[5].as<float>(), Eigen::Vector3f::UnitZ());
                pose.rotate(Eigen::Quaternionf{y * p * r});
                return true;
            } else if (node.size() == 7) {
                pose.translate(Eigen::Vector3f(
                        node[0].as<float>(),
                        node[1].as<float>(),
                        node[2].as<float>()));

                pose.rotate(Eigen::Quaternionf(
                        node[3].as<float>(),
                        node[4].as<float>(),
                        node[5].as<float>(),
                        node[6].as<float>())
                );
                return true;
            }
        }
        return false;
    }
};

template<>
struct convert<Eigen::Vector3f> {
    static bool decode(const Node &node, Eigen::Vector3f &vec3) {
        if (node.IsSequence() && node.size() == 3) {
            vec3.x() = node[0].as<float>();
            vec3.y() = node[1].as<float>();
            vec3.z() = node[2].as<float>();
            return true;
        }
        return false;
    }
};
}


namespace pc_utils {
/**
 * @breif 限制点云的xyz各个维度的最大值
 * @tparam PointT point type
 */
template<class PointT>
class PassThroughFilter final : PC_UTILS_BASE_LIST(PassThroughFilter) {
#define PC_UTILS_MEMBER_VARIABLE        \
define(float, max_x   , )               \
define(float, max_y   , )               \
define(float, max_z   , )               \
define(bool , negative, )

#define PC_UTILS_CLASS                  \
PassThroughFilter

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        typename PC<PointT>::Ptr out(new PC<PointT>);
        out->reserve(input->size());
        out->header = input->header;

        if (negative) {
            for (auto &pt: input->points) {
                if (pt.z > max_z || fabs(pt.x) > max_x || fabs(pt.y) > max_y) {
                    out->points.template emplace_back(pt);
                }
            }
        } else {
            for (auto &pt: input->points) {
                if (pt.z < max_z && fabs(pt.x) < max_x && fabs(pt.y) < max_y) {
                    out->points.template emplace_back(pt);
                }
            }
        }
        output = out;
    }

};

/**
 * @breif 从点云中去除AABB内/外的点
 * @tparam PointT point type
 */
template<class PointT>
class CropAABoxFilter final : PC_UTILS_BASE_LIST(CropAABoxFilter) {
#define PC_UTILS_MEMBER_VARIABLE        \
define(float, min_x   , )               \
define(float, min_y   , )               \
define(float, min_z   , )               \
define(float, max_x   , )               \
define(float, max_y   , )               \
define(float, max_z   , )               \
define(bool , negative, )

#define PC_UTILS_CLASS                  \
CropAABoxFilter

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        pcl::CropBox<PointT> clipper;
        clipper.setMin({min_x, min_y, min_z, 1});
        clipper.setMax({max_x, max_y, max_z, 1});
        clipper.setNegative(negative);
        clipper.setInputCloud(input);
        clipper.filter(*output);
    }
};

/**
 * @breif 从点云中去除OBB内/外的点
 * @tparam PointT point type
 */
template<class PointT>
class CropOBoxFilter final : PC_UTILS_BASE_LIST(CropOBoxFilter) {
#define PC_UTILS_MEMBER_VARIABLE                                \
define(bool             , negative, {false})                    \
define(Eigen::Isometry3f, pose    , )                           \
define(Eigen::Vector3f  , dxyz    , )

#define PC_UTILS_CLASS                                          \
CropOBoxFilter

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        if (data) {
            auto &box = *(pc_utils::BoundingBox *) data;
            pcl::CropBox<PointT> clipper;

            Eigen::Vector4f min_pt, max_pt;
            min_pt[0] = -box.dxyz.x() / 2;
            min_pt[1] = -box.dxyz.y() / 2;
            min_pt[2] = -box.dxyz.z() / 2;
            min_pt[3] = 1;
            max_pt[0] = box.dxyz.x() / 2;
            max_pt[1] = box.dxyz.y() / 2;
            max_pt[2] = box.dxyz.z() / 2;
            max_pt[3] = 1;
            clipper.setMin(min_pt);
            clipper.setMax(max_pt);
            clipper.setTransform(box.pose.inverse());
            clipper.setInputCloud(input);
            clipper.setNegative(negative);
            clipper.filter(*output);
        } else {
            pcl::CropBox<PointT> clipper;

            Eigen::Vector4f min_pt, max_pt;
            min_pt[0] = -dxyz.x() / 2;
            min_pt[1] = -dxyz.y() / 2;
            min_pt[2] = -dxyz.z() / 2;
            min_pt[3] = 1;
            max_pt[0] = dxyz.x() / 2;
            max_pt[1] = dxyz.y() / 2;
            max_pt[2] = dxyz.z() / 2;
            max_pt[3] = 1;
            clipper.setMin(min_pt);
            clipper.setMax(max_pt);
            clipper.setTransform(pose.inverse());
            clipper.setInputCloud(input);
            clipper.setNegative(negative);
            clipper.filter(*output);
        }
    }
};

/**
 * @breif 保留距离原点(r_min,r_max)内的点
 * @tparam PointT
 */
template<class PointT>
class RegionOfInterestFilter final : PC_UTILS_BASE_LIST(RegionOfInterestFilter) {
#define PC_UTILS_MEMBER_VARIABLE        \
    define(float,min_r,)                \
    define(float,max_r,)                \
    define(bool,negative,)

#define PC_UTILS_CLASS                  \
RegionOfInterestFilter

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        auto square_min_r = min_r * min_r, square_max_r = max_r * max_r;

        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        out->header = input->header;
        out->reserve(input->size());
        if (fabs(negative - 1) < 1e-4) {
            for (auto &pt: input->points) {
                auto dis = pt.x * pt.x + pt.y * pt.y;
                if (dis < square_min_r || dis > square_max_r) {
                    out->points.template emplace_back(pt);
                }
            }
        } else {
            for (auto &pt: input->points) {
                auto dis = pt.x * pt.x + pt.y * pt.y;
                if (dis > square_min_r && dis < square_max_r) {
                    out->points.template emplace_back(pt);
                }
            }
        }
        output = out;
    }
};

/**
 * @brief 近似体素滤波
 * @tparam PointT point type
 */
template<class PointT>
class ApproximateVoxelFilter final : PC_UTILS_BASE_LIST(ApproximateVoxelFilter) {
#define PC_UTILS_MEMBER_VARIABLE        \
define(float,leaf_x,)                   \
define(float,leaf_y,)                   \
define(float,leaf_z,)

#define PC_UTILS_CLASS                  \
ApproximateVoxelFilter

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        output->reserve(input->size());
        pcl::ApproximateVoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(leaf_x, leaf_y, leaf_z);
        voxel_grid.setInputCloud(input);
        voxel_grid.filter(*output);
    }
};

/**
 * @breif: 随机采取固定个数的点
 * @tparam PointT
 */
template<class PointT>
class MaxPointCount final : PC_UTILS_BASE_LIST(MaxPointCount) {
#define PC_UTILS_MEMBER_VARIABLE        \
define(size_t, count,)

#define PC_UTILS_CLASS                  \
MaxPointCount

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>(*input));
        const size_t N{static_cast<size_t>(input->size() - 1)};
        if (count <= N) {
            size_t seed = 0;
            if (data) {
                seed = *(size_t *) data;
            }
            std::minstd_rand randomNumberGenerator(static_cast<std::uint_fast32_t>(seed));
            std::uniform_real_distribution<float> distribution{0, 1};

            for (size_t j{0u}; j < count; ++j) {
                //Get a random index in [j; N]
                const size_t index{j + static_cast<size_t>((N - j) * distribution(randomNumberGenerator))};

                //Switch columns j and index
                swap((*out)[j], (*out)[index]);
            }
            //Resize the cloud
            out->resize(count);
        }
        output = out;
    }
};


/**
 * @brief: dropout some point with rate <prob>
 * @tparam PointT
 */
template<class PointT>
class RandomSampling final : PC_UTILS_BASE_LIST(RandomSampling) {
#define PC_UTILS_MEMBER_VARIABLE        \
define(float, prob,{0.5f})              \
define(int, method,{0})

#define PC_UTILS_CLASS                  \
RandomSampling

#include "detail/member_define.h"

    Eigen::VectorXf sampleRandomIndices(const size_t nbPoints) {
        std::random_device randomDevice;
        std::minstd_rand randomNumberGenerator(randomDevice());

        switch (method) {
            default:    // Direct RNG.
            {
                const float randomNumberRange{
                        static_cast<float>(randomNumberGenerator.max() - randomNumberGenerator.min())};
                return Eigen::VectorXf::NullaryExpr(nbPoints, [&](float) {
                    return static_cast<float>(randomNumberGenerator() / randomNumberRange);
                });
            }
            case 1:        // Uniform distribution.
            {
                std::uniform_real_distribution<float> distribution(0, 1);
                return Eigen::VectorXf::NullaryExpr(nbPoints,
                                                    [&](float) { return distribution(randomNumberGenerator); });
            }
        }
    }

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        const size_t nbPointsIn = input->size();
        const size_t nbPointsOut = nbPointsIn * prob;
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>(*input));


        const Eigen::VectorXf randomNumbers{sampleRandomIndices(nbPointsIn)};
        size_t j{0u};
        for (size_t i{0u}; i < nbPointsIn && j <= nbPointsOut; ++i) {
            if (randomNumbers(i) < prob) {
                (*out)[j] = (*out)[i];
                ++j;
            }
        }
        out->resize(j);
        output = out;
    }
};


/**
 * @brief remove NaN point in cloud
 * @tparam PointT
 */
template<class PointT>
class RemoveNaN final : PC_UTILS_BASE_LIST(RemoveNaN) {
#define PC_UTILS_MEMBER_VARIABLE

#define PC_UTILS_CLASS                  \
RemoveNaN

#include "detail/member_define.h"

public:
    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input, *output, indices);
    }
};


/**
 * @brief 滤除自身点云
 * @tparam PointT point type
 */
template<class PointT>
class SelfFilter final : PC_UTILS_BASE_LIST(SelfFilter) {
#define PC_UTILS_MEMBER_VARIABLE        \
define(float,delta_x,)                   \
define(float,delta_y,)                   \
define(float,delta_z,)

#define PC_UTILS_CLASS                  \
SelfFilter

#include "detail/member_define.h"

    pcl::CropBox<PointT> self_filter_;
public:

    void self_filter(const typename PC<PointT>::Ptr &input, const Eigen::Vector3f &,
                     const Eigen::Isometry3f &T1, const Eigen::Isometry3f &T2,
                     double delta_x, double delta_y, double delta_z) {

        Eigen::Vector3f x = T1.translation() - T2.translation(),
                z = T1.matrix().block<3, 1>(0, 2),
                y = z.cross(x),
                center = (T1.translation() + T2.translation()) / 2;
        float box_x = x.norm() + delta_x * 2, box_y = delta_y * 2, box_z = delta_z * 2;

        z = x.cross(y);
        Eigen::Matrix3f rot;
        rot << x.normalized(), y.normalized(), z.normalized();
        Eigen::Isometry3f trans = Eigen::Isometry3f::Identity();
        trans.translate(center);
        trans.rotate(rot);

        Eigen::Vector4f min_pt, max_pt;
        min_pt[0] = -box_x / 2;
        min_pt[1] = -box_y / 2;
        min_pt[2] = -box_z / 2;
        min_pt[3] = 1;
        max_pt[0] = box_x / 2;
        max_pt[1] = box_y / 2;
        max_pt[2] = box_z / 2;
        max_pt[3] = 1;
        self_filter_.setMin(min_pt);
        self_filter_.setMax(max_pt);
        self_filter_.setTransform(trans.inverse());
        self_filter_.setInputCloud(input);
        self_filter_.setNegative(true);
        self_filter_.filter(*input);
    }

    void filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data) override {
        if (data) {
            std::vector<Eigen::Isometry3f> &links_pose = *(std::vector<Eigen::Isometry3f> *) data;
            Eigen::Vector3f vec_z = links_pose[0].matrix().block<3, 1>(0, 2);

            for (int i = 0; i < links_pose.size() - 1; i++) {
                self_filter(input, vec_z, links_pose[i + 1], links_pose[i], delta_x, delta_y, delta_z);
            }
        }
    }
};


}   // namespace pc_utils

#include "detail/template_specialization.h"