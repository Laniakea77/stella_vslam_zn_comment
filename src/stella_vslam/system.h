#ifndef STELLA_VSLAM_SYSTEM_H
#define STELLA_VSLAM_SYSTEM_H

#include "stella_vslam/type.h"
#include "stella_vslam/data/bow_vocabulary_fwd.h"

#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/mat.hpp>

namespace stella_vslam {

class config;
class tracking_module;
class mapping_module;
class global_optimization_module;

namespace camera {
class base;
} // namespace camera

namespace data {
class frame;
class camera_database;
class orb_params_database;
class map_database;
class bow_database;
} // namespace data

namespace feature {
class orb_extractor;
struct orb_params;
} // namespace feature

namespace marker_detector {
class base;
} // namespace marker_detector

namespace publish {
class map_publisher;
class frame_publisher;
} // namespace publish

namespace io {
class map_database_io_base;
}

class system {
public:
    //! Constructor 构造函数
    system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path);

    //! Destructor 析构函数
    ~system();

    //-----------------------------------------
    // system startup and shutdown 系统开启/关闭

    //! 启动
    void startup(const bool need_initialize = true);

    //! 终止
    void shutdown();

    //-----------------------------------------
    // data I/O 数据读写

    //! Save the frame trajectory in the specified format
    // 按设置格式保存轨迹
    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    //! Save the keyframe trajectory in the specified format
    // 按设置格式保存关键帧
    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

    //! Load the map database from the MessagePack file
    // 从 MessagePack file 中载入地图
    void load_map_database(const std::string& path) const;

    //! Save the map database to the MessagePack file
    // 保存地图到 MessagePack file
    void save_map_database(const std::string& path) const;

    //! Get the map publisher
    // 地图publisher
    const std::shared_ptr<publish::map_publisher> get_map_publisher() const;

    //! Get the frame publisher
    // 帧publisher
    const std::shared_ptr<publish::frame_publisher> get_frame_publisher() const;

    //-----------------------------------------
    // module management 模块管理

    //! Enable the mapping module
    // 使能地图模块
    void enable_mapping_module();

    //! Disable the mapping module
    // 关闭地图模块
    void disable_mapping_module();

    //! The mapping module is enabled or not
    // 地图模块使能状态
    bool mapping_module_is_enabled() const;

    //===========回环检测模块==============//
    //! Enable the loop detector
    void enable_loop_detector();

    //! Disable the loop detector
    void disable_loop_detector();

    //! The loop detector is enabled or not
    bool loop_detector_is_enabled() const;

    //! Request loop closure
    // 查询 KF1 和 KF2 是否回环
    bool request_loop_closure(int keyfrm1_id, int keyfrm2_id);

    //===========BA模块==============//
    //! Loop BA is running or not
    // 
    bool loop_BA_is_running() const;

    //! Abort the loop BA externally
    // 终止 loop BA
    void abort_loop_BA();


    //-----------------------------------------
    // data feeding methods
    // 数据流

    // Mat44_t = Eigen::Matrix4d;

    std::shared_ptr<Mat44_t> feed_frame(const data::frame& frm, const cv::Mat& img);

    //! Feed a monocular frame to SLAM system
    //! (NOTE: distorted images are acceptable if calibrated)
    data::frame create_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask = cv::Mat{});
    std::shared_ptr<Mat44_t> feed_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! Feed a stereo frame to SLAM system
    //! (Note: Left and Right images must be stereo-rectified)
    data::frame create_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask = cv::Mat{});
    std::shared_ptr<Mat44_t> feed_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! Feed an RGBD frame to SLAM system
    //! (Note: RGB and Depth images must be aligned)
    data::frame create_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask);
    std::shared_ptr<Mat44_t> feed_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //-----------------------------------------
    // pose initializing/updating

    //! Request to update the pose to a given one.
    //! Return failure in case if previous request was not finished.
    bool relocalize_by_pose(const Mat44_t& cam_pose_wc);
    bool relocalize_by_pose_2d(const Mat44_t& cam_pose_wc, const Vec3_t& normal_vector);

    //-----------------------------------------
    // management for pause

    //! Pause the tracking module
    void pause_tracker();

    //! The tracking module is paused or not
    bool tracker_is_paused() const;

    //! Resume the tracking module
    void resume_tracker();

    //-----------------------------------------
    // management for reset

    //! Request to reset the system
    void request_reset();

    //! Reset of the system is requested or not
    bool reset_is_requested() const;

    //-----------------------------------------
    // management for terminate

    //! Request to terminate the system
    void request_terminate();

    //!! Termination of the system is requested or not
    bool terminate_is_requested() const;

    //-----------------------------------------
    // config

    //! depthmap factor (pixel_value / depthmap_factor = true_depth)
    double depthmap_factor_ = 1.0;

private:
    //! Check reset request of the system
    // 检测系统是否要求重置
    void check_reset_request();

    //! Pause the mapping module and the global optimization module
    // 暂停建图和全局优化模块
    void pause_other_threads() const;

    //! Resume the mapping module and the global optimization module
    // 恢复建图和全局优化模块
    void resume_other_threads() const;

    //! config 配置
    const std::shared_ptr<config> cfg_;
    //! camera model
    // 相机模型
    camera::base* camera_ = nullptr;

    //! camera database
    data::camera_database* cam_db_ = nullptr;

    //! parameters for orb feature extraction
    feature::orb_params* orb_params_ = nullptr;

    //! orb_params database
    data::orb_params_database* orb_params_db_ = nullptr;

    //! map database
    data::map_database* map_db_ = nullptr;

    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;

    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! tracker
    tracking_module* tracker_ = nullptr;

    //! mapping module
    mapping_module* mapper_ = nullptr;
    //! mapping thread
    std::unique_ptr<std::thread> mapping_thread_ = nullptr;

    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;
    //! global optimization thread
    std::unique_ptr<std::thread> global_optimization_thread_ = nullptr;

    // ORB extractors
    //! ORB extractor for left/monocular image
    feature::orb_extractor* extractor_left_ = nullptr;
    //! ORB extractor for right image
    feature::orb_extractor* extractor_right_ = nullptr;
    //! ORB extractor only when used in initializing
    feature::orb_extractor* ini_extractor_left_ = nullptr;

    //! marker detector
    marker_detector::base* marker_detector_ = nullptr;

    //! frame publisher
    std::shared_ptr<publish::frame_publisher> frame_publisher_ = nullptr;
    //! map publisher
    std::shared_ptr<publish::map_publisher> map_publisher_ = nullptr;

    //! map I/O
    std::shared_ptr<io::map_database_io_base> map_database_io_ = nullptr;

    //! system running status flag
    std::atomic<bool> system_is_running_{false};

    //! mutex for reset flag
    // 重置flag的mutex[互斥锁]
    mutable std::mutex mtx_reset_;
    //! reset flag
    bool reset_is_requested_ = false;

    //! mutex for terminate flag
    mutable std::mutex mtx_terminate_;
    //! terminate flag
    bool terminate_is_requested_ = false;

    //! mutex for flags of enable/disable mapping module
    mutable std::mutex mtx_mapping_;

    //! mutex for flags of enable/disable loop detector
    mutable std::mutex mtx_loop_detector_;

    //! Temporary variables for visualization
    std::vector<cv::KeyPoint> keypts_;
};

} // namespace stella_vslam

#endif // STELLA_VSLAM_SYSTEM_H
