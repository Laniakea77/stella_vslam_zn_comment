#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/graph_node.h"
#include "stella_vslam/data/landmark.h"

namespace {
struct {
    bool operator()(const std::pair<unsigned int, std::shared_ptr<stella_vslam::data::keyframe>>& a, const std::pair<unsigned int, std::shared_ptr<stella_vslam::data::keyframe>>& b) {
        return a.first < b.first || (a.first == b.first && a.second != nullptr && (b.second == nullptr || a.second->id_ < b.second->id_));
    }
} cmp_num_shared_lms_and_keyfrm_pairs;
} // namespace

namespace stella_vslam {
namespace data {

graph_node::graph_node(std::shared_ptr<keyframe>& keyfrm)
    : owner_keyfrm_(keyfrm), has_spanning_parent_(false) {}

void graph_node::add_connection(const std::shared_ptr<keyframe>& keyfrm, 
                                const unsigned int num_shared_lms) {
    std::lock_guard<std::mutex> lock(mtx_); // 维系共视图的
    bool need_update = false;

    if (!connected_keyfrms_and_num_shared_lms_.count(keyfrm)) {
        // if `keyfrm` not exists
        connected_keyfrms_and_num_shared_lms_[keyfrm] = num_shared_lms; // 创建
        need_update = true;
    }
    else if (connected_keyfrms_and_num_shared_lms_.at(keyfrm) != num_shared_lms) {
        // if the number of shared landmarks is updated
        connected_keyfrms_and_num_shared_lms_.at(keyfrm) = num_shared_lms;
        need_update = true;
    }

    if (need_update) {
        update_covisibility_orders_impl();
    }
}

void graph_node::erase_connection(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool need_update = false;
    if (connected_keyfrms_and_num_shared_lms_.count(keyfrm)) {
        connected_keyfrms_and_num_shared_lms_.erase(keyfrm);
        need_update = true;
    }

    if (need_update) {
        update_covisibility_orders_impl();
    }
}

void graph_node::erase_all_connections() {
    // remote myself from the connected keyframes
    for (const auto& keyfrm_and_num_shared_lms : connected_keyfrms_and_num_shared_lms_) {
        if (keyfrm_and_num_shared_lms.first.expired()) {
            continue;
        }
        keyfrm_and_num_shared_lms.first.lock()->graph_node_->erase_connection(owner_keyfrm_.lock());
    }
    // remove the buffers
    connected_keyfrms_and_num_shared_lms_.clear();
    ordered_covisibilities_.clear();
    ordered_num_shared_lms_.clear();
}

void graph_node::update_connections(unsigned int min_num_shared_lms) {
    const auto owner_keyfrm = owner_keyfrm_.lock();
    const auto landmarks = owner_keyfrm->get_landmarks();

    // 根据关键帧id排序的map 
    id_ordered_map<std::weak_ptr<keyframe>, unsigned int> keyfrm_and_num_shared_lms;

    for (const auto& lm : landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // 观测到该 landmark 的关键帧
        const auto observations = lm->get_observations();
        // 遍历观测到该landmark的关键帧
        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto locked_keyfrm = keyfrm.lock();
            // 没有 spanning parent ??
            if (!locked_keyfrm->graph_node_->has_spanning_parent_ && locked_keyfrm->id_ != 0) {
                continue;
            }
            if (locked_keyfrm->id_ == owner_keyfrm->id_) { // 不考虑自身
                continue;
            }
            // count up number of shared landmarks of `keyfrm`
            keyfrm_and_num_shared_lms[keyfrm]++;
        }
    }

    if (keyfrm_and_num_shared_lms.empty()) {
        return;
    }

    unsigned int max_num_shared_lms = 0;
    std::shared_ptr<keyframe> nearest_covisibility = nullptr;

    // vector for sorting
    std::vector<std::pair<unsigned int, std::shared_ptr<keyframe>>> num_shared_lms_and_covisibility_pairs;
    num_shared_lms_and_covisibility_pairs.reserve(keyfrm_and_num_shared_lms.size());
    for (const auto& keyfrm_and_num_shared_lms : keyfrm_and_num_shared_lms) {
        auto keyfrm = keyfrm_and_num_shared_lms.first.lock();
        const auto num_shared_lms = keyfrm_and_num_shared_lms.second;

        // nearest_covisibility with greatest id_ will be selected if number of shared landmarks are the same due to ordering of keyfrm_and_num_shared_lms.
        if (max_num_shared_lms <= num_shared_lms) {
            max_num_shared_lms = num_shared_lms;
            nearest_covisibility = keyfrm;
        }

        if (min_num_shared_lms < num_shared_lms) {
            num_shared_lms_and_covisibility_pairs.emplace_back(std::make_pair(num_shared_lms, keyfrm));
        }
    }
    // add ONE node at least
    if (num_shared_lms_and_covisibility_pairs.empty()) {
        num_shared_lms_and_covisibility_pairs.emplace_back(std::make_pair(max_num_shared_lms, nearest_covisibility));
    }

    // add connection from the covisibility to myself
    for (const auto& num_shared_lms_and_covisibility : num_shared_lms_and_covisibility_pairs) {
        auto covisibility = num_shared_lms_and_covisibility.second;
        const auto num_shared_lms = num_shared_lms_and_covisibility.first;
        covisibility->graph_node_->add_connection(owner_keyfrm, num_shared_lms);
    }

    // sort with number of shared landmarks and keyframe IDs for consistency; IDs are also in reverse order
    // to match selection of nearest_covisibility.
    std::sort(num_shared_lms_and_covisibility_pairs.rbegin(), num_shared_lms_and_covisibility_pairs.rend(), cmp_num_shared_lms_and_keyfrm_pairs);

    decltype(ordered_covisibilities_) ordered_covisibilities;
    ordered_covisibilities.reserve(num_shared_lms_and_covisibility_pairs.size());
    decltype(ordered_num_shared_lms_) ordered_num_shared_lms;
    ordered_num_shared_lms.reserve(num_shared_lms_and_covisibility_pairs.size());
    for (const auto& num_shared_lms_and_keyfrm_pair : num_shared_lms_and_covisibility_pairs) {
        ordered_covisibilities.push_back(num_shared_lms_and_keyfrm_pair.second);
        ordered_num_shared_lms.push_back(num_shared_lms_and_keyfrm_pair.first);
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);

        connected_keyfrms_and_num_shared_lms_ = decltype(connected_keyfrms_and_num_shared_lms_)(keyfrm_and_num_shared_lms.begin(), keyfrm_and_num_shared_lms.end());

        ordered_covisibilities_ = ordered_covisibilities;
        ordered_num_shared_lms_ = ordered_num_shared_lms;

        if (!has_spanning_parent_ && owner_keyfrm->id_ != 0) {
            // set the parent of spanning tree
            assert(nearest_covisibility->id_ == ordered_covisibilities.front().lock()->id_);
            spanning_parent_ = nearest_covisibility;
            nearest_covisibility->graph_node_->add_spanning_child(owner_keyfrm);
            has_spanning_parent_ = true;
        }
    }
}

void graph_node::update_covisibility_orders() {
    std::lock_guard<std::mutex> lock(mtx_);
    update_covisibility_orders_impl();
}

void graph_node::update_covisibility_orders_impl() {
    // 
    std::vector<std::pair<unsigned int, std::shared_ptr<keyframe>>> num_shared_lms_and_keyfrm_pairs;
    num_shared_lms_and_keyfrm_pairs.reserve(connected_keyfrms_and_num_shared_lms_.size());

    for (const auto& keyfrm_and_num_shared_lms : connected_keyfrms_and_num_shared_lms_) {
        num_shared_lms_and_keyfrm_pairs.emplace_back(
            std::make_pair(keyfrm_and_num_shared_lms.second,
                           keyfrm_and_num_shared_lms.first.lock()));
    }

    // sort with number of shared landmarks and keyframe IDs for consistency
    std::sort(num_shared_lms_and_keyfrm_pairs.rbegin(), 
              num_shared_lms_and_keyfrm_pairs.rend(), cmp_num_shared_lms_and_keyfrm_pairs);

    ordered_covisibilities_.clear();
    ordered_covisibilities_.reserve(num_shared_lms_and_keyfrm_pairs.size());

    ordered_num_shared_lms_.clear();
    ordered_num_shared_lms_.reserve(num_shared_lms_and_keyfrm_pairs.size());

    for (const auto& num_shared_lms_and_keyfrm_pair : num_shared_lms_and_keyfrm_pairs) {
        ordered_covisibilities_.push_back(num_shared_lms_and_keyfrm_pair.second);
        ordered_num_shared_lms_.push_back(num_shared_lms_and_keyfrm_pair.first);
    }
}

std::set<std::shared_ptr<keyframe>> graph_node::get_connected_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<std::shared_ptr<keyframe>> keyfrms;

    for (const auto& keyfrm_and_num_shared_lms : connected_keyfrms_and_num_shared_lms_) {
        keyfrms.insert(keyfrm_and_num_shared_lms.first.lock());
    }

    return keyfrms;
}

std::vector<std::shared_ptr<keyframe>> graph_node::get_covisibilities() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::shared_ptr<keyframe>> covisibilities;

    for (const auto& covisibility : ordered_covisibilities_) {
        if (covisibility.expired()) {
            continue;
        }
        covisibilities.push_back(covisibility.lock());
    }
    return covisibilities;
}

std::vector<std::shared_ptr<keyframe>> graph_node::get_top_n_covisibilities(const unsigned int num_covisibilities) const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::shared_ptr<keyframe>> covisibilities;
    unsigned int i = 0;
    for (const auto& covisibility : ordered_covisibilities_) {
        if (i == num_covisibilities) {
            break;
        }
        if (covisibility.expired()) {
            continue;
        }
        covisibilities.push_back(covisibility.lock());
        i++;
    }
    return covisibilities;
}

std::vector<std::shared_ptr<keyframe>> graph_node::get_covisibilities_over_min_num_shared_lms(const unsigned int min_num_shared_lms) const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (ordered_covisibilities_.empty()) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    auto itr = std::upper_bound(ordered_num_shared_lms_.begin(), ordered_num_shared_lms_.end(), min_num_shared_lms, std::greater<unsigned int>());
    if (itr == ordered_num_shared_lms_.end()) {
        std::vector<std::shared_ptr<keyframe>> covisibilities;
        for (const auto& covisibility : ordered_covisibilities_) {
            if (covisibility.expired()) {
                continue;
            }
            covisibilities.push_back(covisibility.lock());
        }
        return covisibilities;
    }
    else {
        const auto upper_bound_idx = static_cast<unsigned int>(itr - ordered_num_shared_lms_.begin());
        std::vector<std::shared_ptr<keyframe>> covisibilities;
        unsigned int idx = 0;
        for (const auto& covisibility : ordered_covisibilities_) {
            if (idx == upper_bound_idx) {
                break;
            }
            idx++;
            if (covisibility.expired()) {
                continue;
            }
            covisibilities.push_back(covisibility.lock());
        }
        return covisibilities;
    }
}

unsigned int graph_node::get_num_shared_landmarks(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (connected_keyfrms_and_num_shared_lms_.count(keyfrm)) {
        return connected_keyfrms_and_num_shared_lms_.at(keyfrm);
    }
    else {
        return 0;
    }
}

void graph_node::set_spanning_parent(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    assert(!has_spanning_parent_);
    spanning_parent_ = keyfrm;
    // 这里为什么没有添加孩子节点 ??
    // in graph_node::change_spanning_parent
    // keyfrm->graph_node_->add_spanning_child(owner_keyfrm_.lock());
}
    has_spanning_parent_ = true;
}

bool graph_node::has_spanning_parent() const {
    return has_spanning_parent_;
}

std::shared_ptr<keyframe> graph_node::get_spanning_parent() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return spanning_parent_.lock();
}
// 将
void graph_node::change_spanning_parent(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_parent_ = keyfrm;
    keyfrm->graph_node_->add_spanning_child(owner_keyfrm_.lock());
}

void graph_node::add_spanning_child(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.insert(keyfrm);
}

void graph_node::erase_spanning_child(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.erase(keyfrm);
}

void graph_node::recover_spanning_connections() {
    std::lock_guard<std::mutex> lock(mtx_);

    // 1. find new parents for my children
    // 以当前生成树的父节点们作为 新父节点的候选
    std::set<std::shared_ptr<keyframe>> new_parent_candidates;
    new_parent_candidates.insert(spanning_parent_.lock());
    // 生成树的孩子节点不为空 [这些孩子节点还没有指向对应的父节点]
    while (!spanning_children_.empty()) {
        bool max_is_found = false;

        // 初始化部分, 先都设空
        unsigned int max_num_shared_lms = 0; // 共享最多的landmark [应该对应于orb2中的mappoint]
        // 定义两共享最多landmark的关键帧, 谁是父节点,谁是子节点
        std::shared_ptr<keyframe> max_num_shared_lms_parent = nullptr;
        std::shared_ptr<keyframe> max_num_shared_lms_child = nullptr;

        for (const auto& spanning_child : spanning_children_) {
            auto locked_spanning_child = spanning_child.lock();
            if (locked_spanning_child->will_be_erased()) {
                continue;
            }

            // get intersection between the parent candidates and the spanning-child's covisibilities
            // 得到生成树中父节点集和孩子节点共视图直接的 关键帧交集
            const auto child_covisibilities = 
                locked_spanning_child->graph_node_->get_covisibilities();
            const auto intersection = 
                extract_intersection(new_parent_candidates, child_covisibilities);

            // find the new parent 
            // (which has the maximum number of shared landmarks with the spanning child) from the intersection
            for (const auto& parent_candidate : intersection) {
                // 当前遍历的子节点和指定的parent_candidate节点之间共享 landmark的数目
                const auto num_shared_lms = locked_spanning_child->graph_node_->get_num_shared_landmarks(parent_candidate);
                if (max_num_shared_lms < num_shared_lms) {
                    max_num_shared_lms = num_shared_lms; // 更新最大共享landmark数
                    max_num_shared_lms_parent = parent_candidate;
                    max_num_shared_lms_child = locked_spanning_child;
                    max_is_found = true;
                }
            }
        }
        // 找到了权值最大 [共享路标最多的两个关键帧]

        if (max_is_found) {
            // update spanning tree
            // max_num_shared_lms_parent 指定为 父节点
            // max_num_shared_lms_child 添加到 max_num_shared_lms_parent 的子节点中
            max_num_shared_lms_child->graph_node_->change_spanning_parent(max_num_shared_lms_parent);
            // 从未分配的生成树的孩子节点集中删掉 max_num_shared_lms_child
            spanning_children_.erase(max_num_shared_lms_child);
            // 在 新生成树父节点候选 中, 增加max_num_shared_lms_child
            new_parent_candidates.insert(max_num_shared_lms_child);
        }
        else {
            // cannot update anymore
            break;
        }
    // 继续while, 直到遍历完
    }

    // set my parent as the new parent
    // 遍历生成树的孩子节点 [这些孩子节点都是之前连不上的]
    for (const auto& spanning_child : spanning_children_) {
        const auto child = spanning_child.lock();
        const auto parent = spanning_parent_.lock();
        // 全部分配给生成树的原始父节点
        child->graph_node_->change_spanning_parent(parent); // ?????
    }
    // 清空
    spanning_children_.clear();

    // 2. remove myself from my parent's children list

    spanning_parent_.lock()->graph_node_->erase_spanning_child(owner_keyfrm_.lock());
}

std::set<std::shared_ptr<keyframe>> graph_node::get_spanning_children() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<std::shared_ptr<keyframe>> locked_spanning_children;
    for (const auto& keyfrm : spanning_children_) {
        locked_spanning_children.insert(keyfrm.lock());
    }
    return locked_spanning_children;
}

bool graph_node::has_spanning_child(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<bool>(spanning_children_.count(keyfrm));
}

void graph_node::add_loop_edge(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    loop_edges_.insert(keyfrm);
    // cannot erase loop edges
    owner_keyfrm_.lock()->set_not_to_be_erased();
}

std::set<std::shared_ptr<keyframe>> graph_node::get_loop_edges() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<std::shared_ptr<keyframe>> locked_loop_edges;
    for (const auto& keyfrm : loop_edges_) {
        locked_loop_edges.insert(keyfrm.lock());
    }
    return locked_loop_edges;
}

bool graph_node::has_loop_edge() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return !loop_edges_.empty();
}

template<typename T, typename U>
std::vector<std::shared_ptr<keyframe>> graph_node::extract_intersection(const T& keyfrms_1, const U& keyfrms_2) {
    std::vector<std::shared_ptr<keyframe>> intersection;
    intersection.reserve(std::min(keyfrms_1.size(), keyfrms_2.size()));
    for (const auto keyfrm_1 : keyfrms_1) {
        for (const auto keyfrm_2 : keyfrms_2) {
            if (*keyfrm_1 == *keyfrm_2) {
                intersection.push_back(keyfrm_1);
            }
        }
    }
    return intersection;
}

} // namespace data
} // namespace stella_vslam
