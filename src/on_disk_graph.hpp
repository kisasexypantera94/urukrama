#pragma once

#include "types.hpp"

#include <faiss/IndexPQ.h>
#include <boost/noncopyable.hpp>
#include <mio/mmap.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace urukrama {

template <typename T>
class InMemoryGraph;

template <typename T>
class OnDiskGraph: boost::noncopyable {
public:
    OnDiskGraph(std::string_view index_filename);

public:
    std::vector<std::pair<T, size_t>> GreedySearch(const Point<T>& query, size_t k) const;
    std::vector<std::pair<T, size_t>> GreedySearchWithPQ(const faiss::IndexPQ& index_pq,
                                                         const Point<T>& query,
                                                         size_t k) const;

    static void Write(const InMemoryGraph<T>& in_mem_graph, std::string_view filename);

    static void WriteEmpty(std::string_view filename, size_t dimension, size_t num_points, size_t R, size_t L);
    static void Merge(std::string_view merged_filename, const InMemoryGraph<T>& in_mem_graph);

private:
    static constexpr size_t POINTS_NUM_OFFSET = 0;
    static constexpr size_t POINTS_DIM_OFFSET = POINTS_NUM_OFFSET + sizeof(size_t);
    static constexpr size_t DTYPE_OFFSET = POINTS_DIM_OFFSET + sizeof(size_t);
    static constexpr size_t R_OFFSET = DTYPE_OFFSET + sizeof(uint8_t);
    static constexpr size_t L_OFFSET = R_OFFSET + sizeof(size_t);
    static constexpr size_t MEDOID_IDX_OFFSET = L_OFFSET + sizeof(size_t);
    static constexpr size_t POINTS_OFFSET = MEDOID_IDX_OFFSET + sizeof(size_t);

    static constexpr size_t DUMMY_P_IDX = std::numeric_limits<size_t>::max();

    enum class DataType : uint8_t {
        F32,
    };

private:
    std::vector<std::pair<T, size_t>> GreedySearchInternal(auto distance_func, size_t k) const;

    std::span<const float> GetPoint(size_t p_idx) const;
    std::span<const size_t> GetPointNeighbors(size_t p_idx) const;

    size_t GetPointSectionOffset(size_t p_idx) const;

    template <typename U>
    U ReadAs(size_t offset) const;

    T FullPrecisionDistance(const size_t a_idx, const Point<T>& b) const;

    static DataType GetDataType();

    static void WriteInternal(std::string_view filename,
                              const auto& points,
                              size_t dimension,
                              size_t num_points,
                              size_t R,
                              size_t L,
                              size_t medoid_idx,
                              auto get_n_out);

private:
    mio::mmap_source m_mmap_src;
    size_t m_points_number;
    size_t m_points_dimension;
    size_t m_R;
    size_t m_L;
    size_t m_medoid_idx;
};

template <typename T>
void WriteOnDisk(const InMemoryGraph<T>& graph, std::string_view filename)
{
    OnDiskGraph<T>::Write(graph, filename);
}

}  // namespace urukrama