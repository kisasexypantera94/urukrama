#pragma once

#include <ranges>
#include <vector>

#include <Eigen/Eigen>

namespace urukrama {

template <typename T>
using Point = Eigen::VectorX<T>;

}  // namespace urukrama