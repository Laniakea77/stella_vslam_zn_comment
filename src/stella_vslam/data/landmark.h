#ifndef STELLA_VSLAM_DATA_LANDMARK_H
#define STELLA_VSLAM_DATA_LANDMARK_H

#include "stella_vslam/type.h"

#include <map>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <nlohmann/json_fwd.hpp>

namespace stella_vslam {
namespace data {

class frame;

class keyframe;

class map_database;


// 该类针对的是一个一个的keypoint
// 一个keypoint是一个对象,其id对应于能观测到它的关键帧上
class landmark : public std::enable_shared_from_this<landmark> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Data structure for sorting keyframes by ID 
    // for consistent results in local map cleaning/BA
    //! 第三个参数可看做排序函数, 定义了map中key值的排序规则
    using observations_t = std::map<std::weak_ptr<keyframe>, unsigned int,
                                    id_less<std::weak_ptr<keyframe>>>;

    //! constructor
    /**
     * @brief Construct a new landmark object
     * 
     * @param pos_w 点的世界坐标
     * @param ref_keyfrm 对应的关键帧
     */
    landmark(const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm);

    //! constructor for map loading with computing parameters which can be recomputed
    /**
     * @brief ?
     *  
     * @param id 
     * @param first_keyfrm_id 
     * @param pos_w 
     * @param ref_keyfrm 
     * @param num_visible 
     * @param num_found 
     */
    landmark(const unsigned int id, const unsigned int first_keyfrm_id,
             const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm,
             const unsigned int num_visible, const unsigned int num_found);

    virtual ~landmark();

    //! 设置路标点(提出来的特征点)世界坐标
    void set_pos_in_world(const Vec3_t& pos_w);

    //! 取路标点(提出来的特征点)世界坐标
    Vec3_t get_pos_in_world() const;

    //! get mean normalized vector of keyframe->lm vectors,
    // for keyframes such that observe the 3D point.
    // lm vectors -> ? landmark vector
    Vec3_t get_obs_mean_normal() const;

    //! get reference keyframe, a keyframe at the creation of a given 3D point
    // 取对应 3D landmark 的关键帧
    std::shared_ptr<keyframe> get_ref_keyframe() const;

    //! add observation
    void add_observation(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx);
    //! erase observation
    void erase_observation(map_database* map_db, const std::shared_ptr<keyframe>& keyfrm);

    //! get observations (keyframe and keypoint idx)
    //?? 一个关键帧里应该有很多个点, 为什么
    /**
     * @brief Get the observations object
     *observations_t = std::map<std::weak_ptr<keyframe>, unsigned int,
                                id_less<std::weak_ptr<keyframe>>>;
     * @return observations_t 
     */
    observations_t get_observations() const;

    //! 返回该 landmark(keypoint) 能被几个关键帧观测到
    unsigned int num_observations() const;

    //! 是否有关键帧能观测到 landmark(keypoint)
    bool has_observation() const;

    //! get index of associated keypoint in the specified keyframe
    // 得到指定关键帧=keyfrm 观测到 landmark(keypoint) 对应的index
    int get_index_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const;

    //! whether this landmark is observed in the specified keyframe
    // 该 landmark(keypoint) 有没有在我们指定的关键帧=keyfrm 中被观测到
    bool is_observed_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const;

    //! check the distance between landmark and camera is in ORB scale variance
    /**
     * @brief 
     * 
     * @param cam_to_lm_dist 相机到 landmark的距离?
     * @param margin_far 
     * @param margin_near 
     * @return true 
     * @return false 
     */
    inline bool is_inside_in_orb_scale(const float cam_to_lm_dist, 
                                       const float margin_far, const float margin_near) const {
        const float max_dist = margin_far * get_max_valid_distance();
        const float min_dist = margin_near * get_min_valid_distance();
        return (min_dist <= cam_to_lm_dist && cam_to_lm_dist <= max_dist);
    }

    //! true if the landmark has representative descriptor
    bool has_representative_descriptor() const;

    //! get representative descriptor
    // 得到 landmark 的描述子
    cv::Mat get_descriptor() const;

    //! compute representative descriptor
    void compute_descriptor();

    //! update observation mean normal and ORB scale variance
    void update_mean_normal_and_obs_scale_variance();

    //! true if the landmark has valid prediction parameters
    bool has_valid_prediction_parameters() const;
    //! get max valid distance between landmark and camera
    float get_min_valid_distance() const;
    //! get min valid distance between landmark and camera
    float get_max_valid_distance() const;

    //! predict scale level assuming this landmark is observed in the specified frame/keyframe
    unsigned int predict_scale_level(const float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) const;

    //! erase this landmark from database
    void prepare_for_erasing(map_database* map_db);
    //! whether this landmark will be erased shortly or not
    bool will_be_erased();

    //! Make an interconnection by landmark::add_observation and keyframe::add_landmark
    void connect_to_keyframe(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx);

    //! replace this with specified landmark
    void replace(std::shared_ptr<landmark> lm, data::map_database* map_db);

    void increase_num_observable(unsigned int num_observable = 1);
    void increase_num_observed(unsigned int num_observed = 1);
    unsigned int get_num_observed() const;
    unsigned int get_num_observable() const;
    float get_observed_ratio() const;

    //! encode landmark information as JSON
    nlohmann::json to_json() const;

public:
    unsigned int id_;
    static std::atomic<unsigned int> next_id_;
    unsigned int first_keyfrm_id_ = 0;
    unsigned int num_observations_ = 0;

protected:
    void compute_mean_normal(const observations_t& observations,
                             const Vec3_t& pos_w,
                             Vec3_t& mean_normal) const;
    void compute_orb_scale_variance(const observations_t& observations,
                                    const std::shared_ptr<keyframe>& ref_keyfrm,
                                    const Vec3_t& pos_w,
                                    float& max_valid_dist,
                                    float& min_valid_dist) const;

private:
    //! world coordinates of this landmark
    Vec3_t pos_w_;

    //! observations (keyframe and keypoint index)
    observations_t observations_;

    //! true if the landmark has representative descriptor
    std::atomic<bool> has_representative_descriptor_{false};
    //! representative descriptor
    cv::Mat descriptor_;

    //! reference keyframe
    std::weak_ptr<keyframe> ref_keyfrm_;

    // track counter
    unsigned int num_observable_ = 1;
    unsigned int num_observed_ = 1;

    //! this landmark will be erased shortly or not
    std::atomic<bool> will_be_erased_{false};

    // parameters for prediction
    //! true if the landmark has valid prediction parameters
    std::atomic<bool> has_valid_prediction_parameters_{false};
    //! Normalized average vector (unit vector) of keyframe->lm,
    // for keyframes such that observe the 3D point.
    Vec3_t mean_normal_ = Vec3_t::Zero();
    //! max valid distance between landmark and camera
    float min_valid_dist_ = 0;
    //! min valid distance between landmark and camera
    float max_valid_dist_ = 0;

    mutable std::mutex mtx_position_;
    mutable std::mutex mtx_observations_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_LANDMARK_H
