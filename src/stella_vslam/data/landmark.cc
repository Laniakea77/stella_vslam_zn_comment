#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/match/base.h"

#include <nlohmann/json.hpp>

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace data {

std::atomic<unsigned int> landmark::next_id_{0};

landmark::landmark(const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm)
    : id_(next_id_++), first_keyfrm_id_(ref_keyfrm->id_), pos_w_(pos_w),
      ref_keyfrm_(ref_keyfrm) {}

landmark::landmark(const unsigned int id, const unsigned int first_keyfrm_id,
                   const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm,
                   const unsigned int num_visible, const unsigned int num_found)
    : id_(id), first_keyfrm_id_(first_keyfrm_id), pos_w_(pos_w), ref_keyfrm_(ref_keyfrm),
      num_observable_(num_visible), num_observed_(num_found) {}

landmark::~landmark() {
    SPDLOG_TRACE("landmark::~landmark: {}", id_);
}

void landmark::set_pos_in_world(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    SPDLOG_TRACE("landmark::set_pos_in_world {}", id_);
    pos_w_ = pos_w;
    has_valid_prediction_parameters_ = false;
}

Vec3_t landmark::get_pos_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pos_w_;
}

Vec3_t landmark::get_obs_mean_normal() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    assert(has_valid_prediction_parameters_);
    return mean_normal_;
}

std::shared_ptr<keyframe> landmark::get_ref_keyframe() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return ref_keyfrm_.lock();
}

void landmark::add_observation(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    // id_ 关键帧的id
    SPDLOG_TRACE("landmark::add_observation {} {} {}", id_, keyfrm->id_, idx);
    assert(!static_cast<bool>(observations_.count(keyfrm)));
    // 在map中添加 key=keyfrm,value=idx 的关键帧
    observations_[keyfrm] = idx;
    assert(static_cast<bool>(observations_.count(keyfrm)));

    has_valid_prediction_parameters_ = false;
    has_representative_descriptor_ = false;

    if (!keyfrm->frm_obs_.stereo_x_right_.empty() && // 双目
        0 <= keyfrm->frm_obs_.stereo_x_right_.at(idx)) { //
        num_observations_ += 2; // 加入两个关键帧
    }
    else { // 单目
        num_observations_ += 1;
    }
    // 
}

/**
 * @brief 删掉地图中的一个关键帧
 * 
 * @param map_db 存储地图的数据库
 * @param keyfrm 要删除的关键帧
 */
void landmark::erase_observation(map_database* map_db, const std::shared_ptr<keyframe>& keyfrm) {
    bool discard = false; // ?
    {
        // 创建即加锁，作用域结束自动析构并解锁，无需手工解锁
        std::lock_guard<std::mutex> lock(mtx_observations_);
        SPDLOG_TRACE("清理冗余关键点");
        SPDLOG_TRACE("landmark::erase_observation {} {}", id_, keyfrm->id_);
        // std::map.count() 
        // 对map中特定的元素进行计数, 存在返回1, 否则返回0
        assert(observations_.count(keyfrm));
        // 返回要删除keypoint的id
        int idx = observations_.at(keyfrm);
        // 双目
        if (!keyfrm->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm->frm_obs_.stereo_x_right_.at(idx)) {
            num_observations_ -= 2;
        }
        else { // 单目
            num_observations_ -= 1;
        }

        // 从observations中把关键帧给删除了???????
        observations_.erase(keyfrm);

        has_valid_prediction_parameters_ = false;
        has_representative_descriptor_ = false;

        if (observations_.empty()) { // 空了// 观测区域没有地图点了
            discard = true;
        }
        // weak_ptr<keyframe> ref_keyfrm_;
        // 地图点对应的关键帧 就是 要删的关键帧
        else if (ref_keyfrm_.lock()->id_ == keyfrm->id_) {
            // 观测中存在关键帧的第一帧
            ref_keyfrm_ = observations_.begin()->first.lock();
        }
        assert(discard || observations_.count(ref_keyfrm_));
    }

    if (discard) {
        prepare_for_erasing(map_db);
    }
}

landmark::observations_t landmark::get_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return observations_;
}

unsigned int landmark::num_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observations_;
}

bool landmark::has_observation() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return 0 < num_observations_;
}

int landmark::get_index_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (observations_.count(keyfrm)) { // 是否存在该关键帧
        return observations_.at(keyfrm); // 返回该关键帧观测到的 landmark(keypoint) 的index
    }
    else {
        return -1;
    }
}

bool landmark::is_observed_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<bool>(observations_.count(keyfrm));
}

bool landmark::has_representative_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return has_representative_descriptor_;
}

cv::Mat landmark::get_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    assert(has_representative_descriptor_);
    return descriptor_.clone();
}

void landmark::compute_descriptor() {
    // 确保有 landmark 可处理
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        assert(!has_representative_descriptor_);
        assert(!will_be_erased_);
        assert(!observations_.empty());
        observations = observations_;
    }
    SPDLOG_TRACE("landmark::compute_descriptor {}", id_);

    // Append features of corresponding points
    std::vector<cv::Mat> descriptors;
    descriptors.reserve(observations.size());// 能观测到 landmark 的关键帧的个数
    for (const auto& observation : observations) { // 一个 landmark 在各个帧中的观测

        auto keyfrm = observation.first.lock(); // 键key -> 关键帧
        const auto idx = observation.second; // 值 value -> landmark id

        if (!keyfrm->will_be_erased()) {
            descriptors.push_back(keyfrm->frm_obs_.descriptors_.row(idx));
        }
    }

    // Get median[中位数] of Hamming distance
    // Calculate all the Hamming distances between every pair of the features
    // 计算每对特征点之间的汉明距离(Hamming distances)
    const auto num_descs = descriptors.size();
    std::vector<std::vector<unsigned int>> hamm_dists(num_descs,
                                                      // 描述子数组大小的 uint vector     
                                                      std::vector<unsigned int>(num_descs));
    // 遍历描述子
    for (unsigned int i = 0; i < num_descs; ++i) {
        hamm_dists.at(i).at(i) = 0; // 初始化为 0
        // 遍历之后的每一个
        for (unsigned int j = i + 1; j < num_descs; ++j) {
            const auto dist = match::compute_descriptor_distance_32(
                descriptors.at(i), 
                descriptors.at(j));
            
            hamm_dists.at(i).at(j) = dist;
            hamm_dists.at(j).at(i) = dist;
        }
    }

    // Get the nearest value to median
    // 得到离中位数最近的值
    unsigned int best_median_dist = match::MAX_HAMMING_DIST; // (256)
    unsigned int best_idx = 0;
    for (unsigned idx = 0; idx < num_descs; ++idx) {
        std::vector<unsigned int> partial_hamm_dists(hamm_dists.at(idx).begin(),
                                                     hamm_dists.at(idx).begin() + num_descs);
        // 排序
        std::sort(partial_hamm_dists.begin(), partial_hamm_dists.end());

        const auto median_dist = // 取中位数
                partial_hamm_dists.at(static_cast<unsigned int>(0.5 * (num_descs - 1)));

        if (median_dist < best_median_dist) { // 取最小中位数???
            best_median_dist = median_dist;
            best_idx = idx;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        descriptor_ = descriptors.at(best_idx).clone();
        has_representative_descriptor_ = true;
    }
}

// mean_normal : 均值归一化
void landmark::compute_mean_normal(const observations_t& observations,
                                   const Vec3_t& pos_w,
                                   Vec3_t& mean_normal) const {
    mean_normal = Vec3_t::Zero();
    for (const auto& observation : observations) {
        auto keyfrm = observation.first.lock();// 关键帧
        const Vec3_t normal = pos_w - keyfrm->get_trans_wc();
        mean_normal += normal.normalized();
    }
    mean_normal = mean_normal.normalized();
}

void landmark::compute_orb_scale_variance(const observations_t& observations,
                                          const std::shared_ptr<keyframe>& ref_keyfrm,
                                          const Vec3_t& pos_w,
                                          float& max_valid_dist,
                                          float& min_valid_dist) const {
    // 关键帧到landmark的向量
    const Vec3_t vec_ref_keyfrm_to_lm = pos_w - ref_keyfrm->get_trans_wc();
    // 关键帧到landmark的距离
    const auto dist_ref_keyfrm_to_lm = vec_ref_keyfrm_to_lm.norm();
    assert(!observations.empty());
    // 得到landmark的id
    const auto idx = observations.at(ref_keyfrm);
    // octave :在特定尺寸(长宽)下，经不同高斯核模糊的图像的集合
    // 八度的集合是高斯金字塔。
    const auto scale_level = ref_keyfrm->frm_obs_.undist_keypts_.at(idx).octave;
    // orb_params_ : ORB feature extraction
    const auto scale_factor = ref_keyfrm->orb_params_->scale_factors_.at(scale_level);
    const auto num_scale_levels = ref_keyfrm->orb_params_->num_levels_;

    max_valid_dist = dist_ref_keyfrm_to_lm * scale_factor;
    min_valid_dist = max_valid_dist * ref_keyfrm->orb_params_->inv_scale_factors_.at(num_scale_levels - 1);
}

void landmark::update_mean_normal_and_obs_scale_variance() {
    SPDLOG_TRACE("landmark::update_mean_normal_and_obs_scale_variance {}", id_);
    observations_t observations;
    std::shared_ptr<keyframe> ref_keyfrm = nullptr;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        assert(!has_valid_prediction_parameters_);
        assert(!observations_.empty());
        assert(observations_.count(ref_keyfrm_));
        observations = observations_;
        ref_keyfrm = ref_keyfrm_.lock();
    }
    // 路标点的世界坐标
    Vec3_t pos_w;
    {
        std::lock_guard<std::mutex> lock2(mtx_position_);
        pos_w = pos_w_;
    }

    Vec3_t mean_normal;
    compute_mean_normal(observations, pos_w, mean_normal);

    float max_valid_dist;
    float min_valid_dist;
    compute_orb_scale_variance(observations, ref_keyfrm, pos_w, max_valid_dist, min_valid_dist);

    {
        std::lock_guard<std::mutex> lock3(mtx_position_);
        max_valid_dist_ = max_valid_dist;
        min_valid_dist_ = min_valid_dist;
        mean_normal_ = mean_normal;
        has_valid_prediction_parameters_ = true;
    }
}

bool landmark::has_valid_prediction_parameters() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return has_valid_prediction_parameters_;
}

float landmark::get_min_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    assert(has_valid_prediction_parameters_);
    return min_valid_dist_;
}

float landmark::get_max_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    assert(has_valid_prediction_parameters_);
    return max_valid_dist_;
}

// Acquire keypoints in the cell where the reprojected 3D points exist
unsigned int landmark::predict_scale_level(
                        const float cam_to_lm_dist,
                        float num_scale_levels, 
                        float log_scale_factor) const {
    // static_cast 隐式转换    
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mtx_position_);
        ratio = max_valid_dist_ / cam_to_lm_dist;
    }
    // std::ceil 向上取整
    const auto pred_scale_level = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor));
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (num_scale_levels <= static_cast<unsigned int>(pred_scale_level)) {
        return num_scale_levels - 1;
    }
    else {
        return static_cast<unsigned int>(pred_scale_level);
    }
}

void landmark::prepare_for_erasing(map_database* map_db) {
    SPDLOG_TRACE("landmark::prepare_for_erasing {}", id_);
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        observations = observations_;
        observations_.clear();
        will_be_erased_ = true;
    }

    for (const auto& keyfrm_and_idx : observations) {// 遍历目前存在的观测
        // 删除对应关键帧中的路标点
        keyfrm_and_idx.first.lock()->erase_landmark_with_index(keyfrm_and_idx.second);
    }

    map_db->erase_landmark(id_);
}

bool landmark::will_be_erased() {
    return will_be_erased_;
}

void landmark::connect_to_keyframe(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx) {
    assert(!observations_.count(keyfrm));
    keyfrm->add_landmark(shared_from_this(), idx);
    add_observation(keyfrm, idx);
}

void landmark::replace(std::shared_ptr<landmark> lm, data::map_database* map_db) {
    SPDLOG_TRACE("landmark::replace {} {}", id_, lm->id_);
    if (lm->id_ == id_) { // 同一个landmark, 无需替换
        return;
    }

    // 1. Erase this
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        observations = observations_;
    }

    prepare_for_erasing(map_db);

    // 2. Merge lm with this
    unsigned int num_observable, num_observed;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        num_observable = num_observable_;
        num_observed = num_observed_;
    }

    for (const auto& keyfrm_and_idx : observations) {// 遍历目前存在的观测
        const auto& keyfrm = keyfrm_and_idx.first.lock(); // 定位到关键帧
        if (!lm->is_observed_in_keyframe(keyfrm)) { // 路标点不在当前关键帧中
            lm->connect_to_keyframe(keyfrm, keyfrm_and_idx.second); // 加入
        }
    }
    // 增加路标点被观测到的关键帧数目
    lm->increase_num_observed(num_observed);
    lm->increase_num_observable(num_observable);
}

void landmark::increase_num_observable(unsigned int num_observable) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observable_ += num_observable;
}

void landmark::increase_num_observed(unsigned int num_observed) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observed_ += num_observed;
}

float landmark::get_observed_ratio() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<float>(num_observed_) / num_observable_;
}

unsigned int landmark::get_num_observed() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observed_;
}

unsigned int landmark::get_num_observable() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observable_;
}

nlohmann::json landmark::to_json() const {
    return {{"1st_keyfrm", first_keyfrm_id_},
            {"pos_w", {pos_w_(0), pos_w_(1), pos_w_(2)}},
            {"ref_keyfrm", ref_keyfrm_.lock()->id_},
            {"n_vis", num_observable_},
            {"n_fnd", num_observed_}};
}

} // namespace data
} // namespace stella_vslam
