//
// Created by ou on 2021/10/10.
//

#ifndef PERCEPTION3D_GROUND_ESTIMATE_H
#define PERCEPTION3D_GROUND_ESTIMATE_H

#include "pc_utils/common/common.h"
/**
 * @typedef RansacGroundEstimator
 * @typedef PatchWorkGroundEstimator
 * @typedef RingShapedElevationConjunctionMap
 */
#define PC_UTILS_GROUND_ESTIMATOR_TYPE  \
define( RansacGroundEstimator       )   \
define( PatchWorkGroundEstimator    )

namespace pc_utils {

template<class PointT>
class GroundEstimator {
public:
    virtual void estimate(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                                 typename pcl::PointCloud<PointT>::Ptr &cloud_ground,
                                 typename pcl::PointCloud<PointT>::Ptr &cloud_no_ground) = 0;

};

}  // namespace pc_utils


PC_UTILS_LINK_HELPER_HEADER(GroundEstimator)
#endif //PERCEPTION3D_GROUND_ESTIMATE_H
