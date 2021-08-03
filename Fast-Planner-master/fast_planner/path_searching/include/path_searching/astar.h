/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _ASTAR_H
#define _ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>    /// 散列表
// #include "grad_spline/sdf_map.h"
#include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>  ///
#include <queue>
namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class Node {
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node* parent;
  char node_state;

  double time;  // dyn
  int time_idx;

  /* -------------------- */
  Node() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~Node(){};
};

typedef Node* NodePtr;   ///

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;    ///
  }
};

///
template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);  /// std::hash<typename T::Scalar>()
    }
    return seed;
  }
};

///
class NodeHashTable0 {
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, NodePtr, matrix_hash0<Eigen::Vector4i>> data_4d_;

public:
  NodeHashTable0(/* args */) {
  }
  ~NodeHashTable0() {
  }
  ///
  void insert(Eigen::Vector3i idx, NodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }
  /// 动态障碍物时用到
  void insert(Eigen::Vector3i idx, int time_idx, NodePtr node) {
    data_4d_.insert(make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  ///
  NodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  ///
  NodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

/// 论文1  standard A* algorithm
class Astar {
private:
  /* ---------- main data structure ---------- */
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable0 expanded_nodes_;   /// 扩展的节点
  ///  priority_queue<Type, Container, Functional>
  ///  Type 就是数据类型，Container 就是容器类型（Container必须是用数组实现的容器，比如vector,deque等等，但不能用 list。STL里面默认用的是vector），Functional 就是比较的方式
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;   ///
  std::vector<NodePtr> path_nodes_;  /// 存放的是路径上的节点

  /* ---------- record data ---------- */
  EDTEnvironment::Ptr edt_environment_;   ///
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double lambda_heu_;
  double margin_;
  int allocate_num_;   /// 存储池的大小
  double tie_breaker_;   ///
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(NodePtr end_node);  ///

  /* heuristic function */
  double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

public:
  Astar(){};
  ~Astar();

  enum { REACH_END = 1, NO_PATH = 2 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic = false,
             double time_start = -1.0);

  void setEnvironment(const EDTEnvironment::Ptr& env);
  std::vector<Eigen::Vector3d> getPath();
  std::vector<NodePtr> getVisitedNodes();

  typedef shared_ptr<Astar> Ptr;  ///
};

}  // namespace fast_planner

#endif