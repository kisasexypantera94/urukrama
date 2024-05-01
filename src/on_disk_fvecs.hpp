#pragma once

#include <mio/mmap.hpp>

#include <span>
#include <string_view>
#include <vector>

namespace urukrama {

template <bool MUTABLE>
class OnDiskFVecs {
public:
    OnDiskFVecs(std::string_view filename);

public:
    std::vector<std::vector<size_t>> ComputeClusters(size_t k, size_t niter = 50, size_t l = 2) const;

    size_t FindMedoid() const;

    void Shuffle()
        requires(MUTABLE);

    std::span<const float> GetPoint(size_t p_idx) const;
    std::span<float> GetPointMut(size_t p_idx)
        requires(MUTABLE);

    size_t Dimension() const;
    size_t Size() const;

private:
    using MemMap = std::conditional<MUTABLE, mio::mmap_sink, mio::mmap_source>::type;

private:
    size_t PointByteLength() const;

private:
    MemMap m_mmap;
    size_t m_dimension;
    size_t m_num_points;
};

}  // namespace urukrama