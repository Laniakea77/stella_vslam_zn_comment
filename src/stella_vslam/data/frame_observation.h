#ifndef STELLA_VSLAM_DATA_FRAME_OBSERVATION_H
#define STELLA_VSLAM_DATA_FRAME_OBSERVATION_H

#include "stella_vslam/type.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace stella_vslam {
namespace data {

// 观测到的特征点
struct frame_observation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame_observation() = default;
    frame_observation(unsigned int num_keypts, const cv::Mat& descriptors, // keypoint数目,描述子
                      const std::vector<cv::KeyPoint>& undist_keypts, // keypoint vector
                      // eigen_alloc_vector = std::vector<T, Eigen::aligned_allocator<T>>;
                      const eigen_alloc_vector<Vec3_t>& bearings, 
                      const std::vector<float>& stereo_x_right, 
                      const std::vector<float>& depths,
                      const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells)
        : num_keypts_(num_keypts), descriptors_(descriptors), undist_keypts_(undist_keypts), bearings_(bearings),
          stereo_x_right_(stereo_x_right), depths_(depths), keypt_indices_in_cells_(keypt_indices_in_cells) {}

    //! 关键点的数目
    unsigned int num_keypts_ = 0;

    //! descriptors 描述子
    cv::Mat descriptors_;// ?

    //! undistorted keypoints of monocular or stereo left image
    // 单目或双目左视图的非畸变的关键点
    std::vector<cv::KeyPoint> undist_keypts_;

    //! bearing vectors
    // 由相机指向 landmark
    eigen_alloc_vector<Vec3_t> bearings_;

    //! disparities
    std::vector<float> stereo_x_right_;

    //! depths
    std::vector<float> depths_;

    //! keypoint indices in each of the cells
    std::vector<
      std::vector<
        std::vector<unsigned int>>> keypt_indices_in_cells_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_FRAME_OBSERVATION_H
