#pragma once

#include <tsl/robin_map.h>
#include <tsl/robin_set.h>
#include <Eigen/Eigen>


namespace urukrama {

template <typename K>
using HashSet = tsl::robin_set<K>;

template <typename K, typename V>
using HashMap = tsl::robin_map<K, V>;

template <typename T>
using Point = Eigen::VectorX<T>;

}  // namespace urukrama