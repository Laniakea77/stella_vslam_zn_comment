#ifndef STELLA_VSLAM_MATCH_BASE_H
#define STELLA_VSLAM_MATCH_BASE_H

#include <array>
#include <algorithm>
#include <numeric>

#include <opencv2/core/mat.hpp>

namespace stella_vslam {
namespace match {

static constexpr unsigned int HAMMING_DIST_THR_LOW = 50;
static constexpr unsigned int HAMMING_DIST_THR_HIGH = 100;
static constexpr unsigned int MAX_HAMMING_DIST = 256;

//! 计算ORB特征点间的汉明距离
inline unsigned int compute_descriptor_distance_32(
    const cv::Mat& desc_1, 
    const cv::Mat& desc_2) {
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    // 参考: https://stackoverflow.com/questions/55013464/where-the-hamming-distance-constants-came-from
    // 32 = 4*8 
    // 两个位一组
    // k=1 01_01_01_01 
    constexpr uint32_t mask_1 = 0x55555555U;
    // k=2 00_11_00_11 // double k=1 +1
    constexpr uint32_t mask_2 = 0x33333333U;
    // k=3 00_00_11_11 // double k=2 +1
    constexpr uint32_t mask_3 = 0x0F0F0F0FU;
    // k=4 00_00_00_01 // double k=3 +1
    constexpr uint32_t mask_4 = 0x01010101U;

    const auto* pa = desc_1.ptr<uint32_t>();
    const auto* pb = desc_2.ptr<uint32_t>();

    unsigned int dist = 0;

    // 看成一个树形,相邻的两个bit先合成2-bit number
    // 然后相邻的2-bit number 合成 4-bit number

    // 举例 : 相邻的两个bit sum to 2-bit number
    // 0 0 -> 0 + 0 = 00
    // 0 1 -> 0 + 1 = 01
    // 1 0 -> 1 + 0 = 01
    // 1 1 -> 1 + 1 = 10
    // 其效果等价于  v -= ((v >> 1) & mask_1)

    for (unsigned int i = 0; i < 8; ++i, ++pa, ++pb) {
        auto v = *pa ^ *pb;
        v -= ((v >> 1) & mask_1); // 相邻两位sum to 2-bit number

        v = (v & mask_2) + ((v >> 2) & mask_2); // 2-bit sum to 4-bit
        // ((v + (v >> 4)) & mask_3) 4-bit sum to 8-bit
        dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 24;
    }

    return dist;
}

//! ORB特徴量間のハミング距離を計算する
inline unsigned int compute_descriptor_distance_64(
    const cv::Mat& desc_1, 
    const cv::Mat& desc_2) {
    // https://stackoverflow.com/questions/21826292/t-sql-hamming-distance-function-capable-of-decimal-string-uint64?lq=1

    constexpr uint64_t mask_1 = 0x5555555555555555UL;
    constexpr uint64_t mask_2 = 0x3333333333333333UL;
    constexpr uint64_t mask_3 = 0x0F0F0F0F0F0F0F0FUL;
    constexpr uint64_t mask_4 = 0x0101010101010101UL;

    const auto* pa = desc_1.ptr<uint64_t>();
    const auto* pb = desc_2.ptr<uint64_t>();

    unsigned int dist = 0;

    for (unsigned int i = 0; i < 4; ++i, ++pa, ++pb) {
        auto v = *pa ^ *pb;
        v -= (v >> 1) & mask_1;
        v = (v & mask_2) + ((v >> 2) & mask_2);
        dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 56;
    }

    return dist;
}

class base {
public:
    base(const float lowe_ratio, const bool check_orientation)
        : lowe_ratio_(lowe_ratio), check_orientation_(check_orientation) {}

    virtual ~base() = default;

protected:
    const float lowe_ratio_;
    const bool check_orientation_;
};

} // namespace match
} // namespace stella_vslam

#endif // STELLA_VSLAM_MATCH_BASE_H
