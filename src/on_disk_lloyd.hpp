#pragma once

#include <mio/mmap.hpp>

#include <string_view>

namespace urukrama {

void ComputeClustersOnDisk(std::string_view fvecs_filename, size_t k, size_t niter = 50);

}  // namespace urukrama