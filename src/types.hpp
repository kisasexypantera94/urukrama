#pragma once

#include <tsl/sparse_map.h>
#include <tsl/sparse_set.h>
#include <Eigen/Eigen>


namespace urukrama {

template <typename K>
using HashSet = tsl::sparse_set<K>;

template <typename K, typename V>
using HashMap = tsl::sparse_map<K, V>;

template <typename T>
using Point = Eigen::VectorX<T>;

}  // namespace urukrama