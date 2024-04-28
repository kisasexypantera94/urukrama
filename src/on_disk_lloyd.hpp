#pragma once

#include <mio/mmap.hpp>

#include <string_view>
#include <vector>

namespace urukrama {

std::vector<std::vector<size_t>> ComputeClustersOnDisk(std::string_view fvecs_filename,
                                                       size_t k,
                                                       size_t niter = 50,
                                                       size_t l = 2);

}  // namespace urukrama