//
// Created by nrsl on 2021/10/11.
//
#include "pc_utils/bound/box_extract.h"
#include "pc_utils/common/factory.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;

namespace pc_utils {
/**
 * @breif BoundingBoxHelper
 * @tparam PointT point type
 */
template<class PointT>
void BoundingBoxHelper<PointT>::cloud_Twc_cw_PCA(const typename PC<PointT>::Ptr &_cloud_w,
                                                 Matrix4f &_Twc, Matrix4f &_Tcw, bool is_box3d) {

    Matrix3f cov, R_wc;
    Vector4f centroid;

    pcl::computeMeanAndCovarianceMatrix(*_cloud_w, cov, centroid);
    if (not is_box3d) {
        // 使用垂直地面的bbox2.5d，故只保留xy方向的协方差
        Matrix3f cov_no_z = Matrix3f::Zero();
        cov_no_z.block<2, 2>(0, 0) = cov.block<2, 2>(0, 0);
        cov = cov_no_z;
    }
    // 特征向量按特征值从小到大排列
    Matrix3f eigen_vectors = SelfAdjointEigenSolver<Matrix3f>(cov, ComputeEigenvectors).eigenvectors();
    // 用两个主成分做xy轴并计算z轴方向
    R_wc << eigen_vectors.col(2), eigen_vectors.col(1), eigen_vectors.col(2).cross(eigen_vectors.col(1));

    // compute t_wc
    auto t_wc = Vector3f(centroid.head<3>());

    // compute T_wc
    Matrix4f T_wc(Matrix4f::Identity());
    T_wc.block<3, 3>(0, 0) = R_wc;
    T_wc.block<3, 1>(0, 3) = t_wc;

    // compute T_cw
    Matrix4f T_cw(Matrix4f::Identity());   //block<n_r,n_c>(sr,sc)
    T_cw.block<3, 3>(0, 0) = R_wc.transpose(); //R_wc = R_cw'
    T_cw.block<3, 1>(0, 3) = -1.f * (R_wc.transpose() * t_wc); //t_wc = -R_cw'*t_cw

    // move to output
    _Twc = T_wc;
    _Tcw = T_cw;
}

template<class PointT>
void BoundingBoxHelper<PointT>::get3DBBox(const typename PC<PointT>::Ptr &_cloud_w, BoundingBox &_box, bool is_box3d) {
    Matrix4f T_wc;
    Matrix4f T_cw;
    cloud_Twc_cw_PCA(_cloud_w, T_wc, T_cw, is_box3d);

    //get cloud in cloud-frame
    PC<PointT> _cloud_c;
    pcl::transformPointCloud(*_cloud_w, _cloud_c, T_cw);

    // compute center of geometry
    Vector4f min_pt, max_pt;
    pcl::getMinMax3D(_cloud_c, min_pt, max_pt);
    const Vector3f geometry_centroid_c = 0.5f * (min_pt + max_pt).head<3>();

    //compute T_wc
    T_wc.block<3, 3>(0, 0) = T_wc.block<3, 3>(0, 0);
    T_wc.block<3, 1>(0, 3) += T_wc.block<3, 3>(0, 0) * geometry_centroid_c;

    //comput box's scale and
    Vector3f scale = (max_pt - min_pt).head<3>();


    _box.pose = T_wc;
    _box.dxyz = scale;
}

template<class PointT>
void BoundingBoxHelper<PointT>::extract(const typename PC<PointT>::Ptr &_cloud_w, BoundingBox &_box, bool is_box3d) {
    get3DBBox(_cloud_w, _box, is_box3d);
}

PC_UTILS_TEMPLATE_SPECIALIZATIONS(BoundingBoxHelper)
}  // namespace pc_utils

namespace pc_utils {
/**
 * @breif extract 3d obb
 * @tparam PointT point type
 */
template<class PointT>
class BoundingBox3DExtract final :
        public BoundingExtract<PointT>,
        protected BoundingBoxHelper<PointT>,
        public AutoRegister<BoundingExtract<PointT>, BoundingBox3DExtract<PointT>>::template OverLoad<> {
public:

    void extract(const typename PC<PointT>::Ptr &input, BoundingBox &output) override {
        BoundingBoxHelper<PointT>::get3DBBox(input, output, true);
    }

    BoundingBox3DExtract() {};
};

PC_UTILS_TEMPLATE_SPECIALIZATIONS(BoundingBox3DExtract)


/**
 * @breif extract 2.5d obb
 * @tparam PointT point type
 */
template<class PointT>
class BoundingBox2_5DExtract final
        : public BoundingExtract<PointT>,
          protected BoundingBoxHelper<PointT>,
          public AutoRegister<BoundingExtract<PointT>, BoundingBox2_5DExtract<PointT>>::template OverLoad<> {
public:
    void extract(const typename PC<PointT>::Ptr &input, BoundingBox &output) override {
        BoundingBoxHelper<PointT>::get3DBBox(input, output, false);
    }

    BoundingBox2_5DExtract() {};
};

PC_UTILS_TEMPLATE_SPECIALIZATIONS(BoundingBox2_5DExtract)
}    // namespace pc_utils

PC_UTILS_LINK_HELPER_CPP(box_extract)
