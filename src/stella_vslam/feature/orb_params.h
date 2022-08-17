#ifndef STELLA_VSLAM_FEATURE_ORB_PARAMS_H
#define STELLA_VSLAM_FEATURE_ORB_PARAMS_H

#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

/**
 * @brief 
 * feature : keypoint + descriptor
 * Oriented FAST + Rotated BRIEF
 */

namespace stella_vslam {
namespace feature {

struct orb_params {
    orb_params() = delete;

    //! Constructor
    // scale_factor 决定金字塔层数间缩放倍率的
    orb_params(const std::string& name, const float scale_factor, 
               const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr);
    orb_params(const std::string& name);

    //! Constructor
    // 从 yaml 文件中读
    explicit orb_params(const YAML::Node& yaml_node);

    //! Destructor
    virtual ~orb_params() = default;

    nlohmann::json to_json() const;

    //! name (id for saving)
    const std::string name_;

    const float scale_factor_ = 1.2; // 缩放因子
    const float log_scale_factor_ = std::log(1.2); // 缩放因子的对数
    const unsigned int num_levels_ = 8; // 金字塔的层数
    // 像素差
    const unsigned int ini_fast_thr_ = 20; // 初始的FAST角点阈值，可以提取出最明显的角点
    const unsigned int min_fast_thr_ = 7;// 在设置的初始阈值下没有检测到足够的角点, 降低到最小阈值

    //! A list of the scale factor of each pyramid layer
    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    //! A list of σ of each pyramid layer
    std::vector<float> level_sigma_sq_;
    std::vector<float> inv_level_sigma_sq_;

    //! Calculate scale factors
    // 每层缩放 scale_factor_ = 1.2 倍
    static std::vector<float> calc_scale_factors(
        const unsigned int num_scale_levels,
        const float scale_factor);

    //! Calculate inverses of scale factors
    static std::vector<float> calc_inv_scale_factors(
        const unsigned int num_scale_levels,
        const float scale_factor);

    //! Calculate σ² at all levels (σ²= scale_factor²)
    static std::vector<float> calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate 1/σ² at all levels
    static std::vector<float> calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);
};

std::ostream& operator<<(std::ostream& os, const orb_params& oparam);

} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_ORB_PARAMS_H
