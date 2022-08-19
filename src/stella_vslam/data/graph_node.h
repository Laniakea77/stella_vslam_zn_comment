#ifndef STELLA_VSLAM_DATA_GRAPH_NODE_H
#define STELLA_VSLAM_DATA_GRAPH_NODE_H

#include <atomic>
#include <mutex>
#include <vector>
#include <map>
#include <set>
#include <memory>

namespace stella_vslam {
namespace data {

class keyframe;
// 共视图中的关键帧节点
// 图节点
class graph_node {
public:
    /**
     * Constructor
     * 以关键帧为节点
     */
    explicit graph_node(std::shared_ptr<keyframe>& keyfrm);

    /**
     * Destructor
     */
    ~graph_node() = default;

    //-----------------------------------------
    // covisibility graph
    // 共视图 : 无向加权图

    /**
     * Add connection between myself and specified keyframes with the number of shared landmarks
     * [通过共享的landmark?]连接自身和指定关键帧
     */
    void add_connection(const std::shared_ptr<keyframe>& keyfrm, const unsigned int num_shared_lms);

    /**
     * Erase connection between myself and specified keyframes
     * 在当前关键帧和一个指定帧之间删除一个连接
     */
    void erase_connection(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Erase all connections
     */
    void erase_all_connections();

    /**
     * Update the connections and the covisibilities by referring landmark observations
     */
    void update_connections(unsigned int min_num_shared_lms);

    /**
     * Update the order of the covisibilities
     * (NOTE: the new keyframe won't inserted)
     */
    void update_covisibility_orders();

    /**
     * Get the connected keyframes
     */
    std::set<std::shared_ptr<keyframe>> get_connected_keyframes() const;

    /**
     * Get the covisibility keyframes
     */
    std::vector<std::shared_ptr<keyframe>> get_covisibilities() const;

    /**
     * Get the top-n covisibility keyframes
     */
    std::vector<std::shared_ptr<keyframe>> get_top_n_covisibilities(const unsigned int num_covisibilities) const;

    /**
     * Get the covisibility keyframes which have shared landmarks over the threshold
     */
    std::vector<std::shared_ptr<keyframe>> get_covisibilities_over_min_num_shared_lms(const unsigned int min_num_shared_lms) const;

    /**
     * Get the number of shared landmarks between this and specified keyframe
     */
    unsigned int get_num_shared_landmarks(const std::shared_ptr<keyframe>& keyfrm) const;

    //-----------------------------------------
    // spanning tree

    /**
     * Set the parent node of spanning tree
     * (NOTE: this functions will be only used for map loading)
     * 设置当前节点生成树的parent为keyfrm
     */
    void set_spanning_parent(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Whether this node has the parent or not
     * 查 bool值 has_spanning_parent_;
     */
    bool has_spanning_parent() const;

    /**
     * Get the parent of spanning tree
     * 得到当前节点生成树的parent
     * spanning_parent_.lock()
     */
    std::shared_ptr<keyframe> get_spanning_parent() const;

    /**
     * Change the parent node of spanning tree
     * 改变生成树的父节点
     */
    void change_spanning_parent(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Add the child note of spanning tree
     * 为生成树增加子节点
     */
    void add_spanning_child(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Erase the child node of spanning tree
     * 从生成树的子节点中删掉 keyfrm
     */
    void erase_spanning_child(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Recover the spanning connections of the connected keyframes
     * 更新生成树
     */
    void recover_spanning_connections();

    /**
     * Get the children of spanning tree
     * 锁住线程, 取id_ordered_set<std::weak_ptr<keyframe>> spanning_children_;
     */
    std::set<std::shared_ptr<keyframe>> get_spanning_children() const;

    /**
     * Whether this node has the specified child or not
     * spanning_children_ 中是否含有节点keyfrm
     */
    bool has_spanning_child(const std::shared_ptr<keyframe>& keyfrm) const;

    //-----------------------------------------
    // loop edge

    /**
     * Add the loop edge
     */
    void add_loop_edge(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Get the loop edges
     */
    std::set<std::shared_ptr<keyframe>> get_loop_edges() const;

    /**
     * Whether this node has any loop edges or not
     */
    bool has_loop_edge() const;

private:
    /**
     * Update the order of the covisibilities (without mutex)
     * 更新 共视图的顺序
     * (NOTE: the new keyframe won't inserted)
     */
    void update_covisibility_orders_impl();

    /**
     * Extract intersection from the two lists of keyframes
     */
    template<typename T, typename U>
    static std::vector<std::shared_ptr<keyframe>> extract_intersection(const T& keyfrms_1, const U& keyfrms_2);

    //! keyframe of this node
    // 当前节点的关键帧
    std::weak_ptr<keyframe> const owner_keyfrm_;

    //! all connected keyframes and the number of shared landmarks between the keyframes
    // 当前关键帧连接的所有关键帧和共享landmark的个数
    // id_ordered_map<std::weak_ptr<keyframe>, unsigned int> <=> map<keyF,int> 根据int
    // 大小排序
    id_ordered_map<std::weak_ptr<keyframe>, unsigned int> connected_keyfrms_and_num_shared_lms_;

    //! covisibility keyframe in descending order of the number of shared landmarks
    // 共视关键帧根据 shared landmarks 数目进行降序排列
    std::vector<std::weak_ptr<keyframe>> ordered_covisibilities_;
    //! number of shared landmarks in descending order
    // 降序排列的 shared landmarks 数
    std::vector<unsigned int> ordered_num_shared_lms_;

    //! parent of spanning tree
    // 生成树的父节点
    std::weak_ptr<keyframe> spanning_parent_;
    //! children of spanning tree
    // 生成树的子节点
    id_ordered_set<std::weak_ptr<keyframe>> spanning_children_;
    //! flag which indicates spanning tree is not set yet or not
    std::atomic<bool> has_spanning_parent_;

    //! loop edges 回环边, 为什么里面是关键帧?
    //! 按关键帧序号排列的关键帧set
    id_ordered_set<std::weak_ptr<keyframe>> loop_edges_;

    //! need mutex for access to connections
    mutable std::mutex mtx_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_GRAPH_NODE_H
