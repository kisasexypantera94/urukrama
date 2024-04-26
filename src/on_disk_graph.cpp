#include "on_disk_graph.hpp"
#include "bounded_sorted_vector.hpp"
#include "in_memory_graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <format>
#include <fstream>
#include <iostream>
#include <ranges>

namespace urukrama {


template <typename T>
OnDiskGraph<T>::OnDiskGraph(std::string_view index_filename)
    : m_mmap_src(mio::mmap_source(index_filename, 0, mio::map_entire_file))
    , m_points_number(ReadAs<size_t>(POINTS_NUM_OFFSET))
    , m_points_dimension(ReadAs<size_t>(POINTS_DIM_OFFSET))
    , m_R(ReadAs<size_t>(R_OFFSET))
    , m_L(ReadAs<size_t>(L_OFFSET))
    , m_medoid_idx(ReadAs<size_t>(MEDOID_IDX_OFFSET))
{
    if (const auto data_type = ReadAs<DataType>(DTYPE_OFFSET); data_type != GetDataType()) {
        ksp::log::Error("Unexpected data type: {}", uint8_t(data_type));
        throw std::runtime_error(std::format("unexpected data type: {}", uint8_t(data_type)));
    }

    ksp::log::Info("Loaded graph: points_number=[{}], points_dimension=[{}], R=[{}], medoid_idx=[{}]",
                   m_points_number,
                   m_points_dimension,
                   m_R,
                   m_medoid_idx);
}

template <typename T>
std::vector<std::pair<T, size_t>> OnDiskGraph<T>::GreedySearchWithPQ(const faiss::IndexPQ& index_pq,
                                                                     const Point<T>& query,
                                                                     size_t k) const
{
    auto computer = index_pq.get_FlatCodesDistanceComputer();
    computer->set_query(query.data());

    return GreedySearchInternal([&](size_t p_idx) { return (*computer)(faiss::idx_t(p_idx)); }, k);
}

template <typename T>
std::vector<std::pair<T, size_t>> OnDiskGraph<T>::GreedySearch(const Point<T>& query, size_t k) const
{
    return GreedySearchInternal([&](size_t p_idx) { return FullPrecisionDistance(p_idx, query); }, k);
}

template <typename T>
std::vector<std::pair<T, size_t>> OnDiskGraph<T>::GreedySearchInternal(auto distance_func, size_t k) const
{
    HashSet<size_t> fast_visited;
    fast_visited.reserve(m_L * 2);

    std::vector<std::pair<T, size_t>> visited;
    visited.reserve(m_L * 2);

    BoundedSortedVector<T, size_t> candidates(m_L);
    candidates.reserve(m_L + 1);
    candidates.emplace(distance_func(m_medoid_idx), m_medoid_idx);

    while (true) {
        auto it = std::find_if(candidates.begin(), candidates.end(), [&](const auto& c) {
            return not fast_visited.contains(c.second);
        });

        if (it == candidates.end()) {
            break;
        }

        const auto [min_distance, p_star_idx] = *it;

        fast_visited.insert(p_star_idx);
        visited.emplace_back(min_distance, p_star_idx);

        const std::span n_out = GetPointNeighbors(p_star_idx);

        for (const size_t n_idx: n_out | std::views::filter([](const size_t n_idx) { return n_idx != DUMMY_P_IDX; })) {
            if (not fast_visited.contains(n_idx)) {
                candidates.emplace(distance_func(n_idx), n_idx);
            }
        }
    }

    std::sort(visited.begin(), visited.end());
    candidates.resize(k);

    return candidates;
}

template <typename T>
void OnDiskGraph<T>::Write(const InMemoryGraph<T>& in_mem_graph, std::string_view filename)
{
    std::ofstream ofs(filename.data(), std::ios::binary | std::ios::trunc);

    if (not ofs.is_open()) {
        ksp::log::Error("Could not open file for writing: {}", filename);
        throw std::runtime_error(std::format("could not open file for writing: {}", filename));
    }

    const auto write = [&]<typename... Ts>(Ts... vals) {
        static_assert(((alignof(decltype(vals)) == sizeof(vals)) && ...));

        // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
        (ofs.write(reinterpret_cast<const char*>(&vals), sizeof(vals)), ...);
        // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

        return SizeOf<Ts...>{};
    };

    const auto& [R, L, points, medoid_idx, n_out] = in_mem_graph;

    const auto points_offset = write(points.size(), points.front().size(), GetDataType(), R, L, medoid_idx);
    static_assert(points_offset.value == POINTS_OFFSET);

    for (const auto& [p_idx, p]: points | std::views::enumerate) {
        for (const T x: p) {
            write(x);
        }

        for (const size_t n_idx: n_out[p_idx]) {
            write(n_idx);
        }

        for (size_t i = 0; i < R - n_out[p_idx].size(); ++i) {
            write(DUMMY_P_IDX);
        }
    }
}

template <typename T>
std::span<const float> OnDiskGraph<T>::GetPoint(size_t p_idx) const
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t point_offset = GetPointSectionOffset(p_idx);
    auto point = std::span{reinterpret_cast<const float*>(&m_mmap_src[point_offset]), m_points_dimension};
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    return point;
}

template <typename T>
std::span<const size_t> OnDiskGraph<T>::GetPointNeighbors(size_t p_idx) const
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t point_neighbors_offset = GetPointSectionOffset(p_idx) + m_points_dimension * sizeof(T);
    auto point_neighbors = std::span{reinterpret_cast<const size_t*>(&m_mmap_src[point_neighbors_offset]), m_R};
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    return point_neighbors;
}


template <typename T>
size_t OnDiskGraph<T>::GetPointSectionOffset(size_t p_idx) const
{
    return POINTS_OFFSET + p_idx * (m_points_dimension * sizeof(T) + m_R * sizeof(size_t));
}

template <typename T>
template <typename U>
U OnDiskGraph<T>::ReadAs(size_t offset) const
{
    U val;
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    std::memcpy(reinterpret_cast<char*>(&val), &m_mmap_src[offset], sizeof(val));
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
    return val;
}

template <>
OnDiskGraph<float>::DataType OnDiskGraph<float>::GetDataType()
{
    return DataType::F32;
}

template <typename T>
T OnDiskGraph<T>::FullPrecisionDistance(const size_t a_idx, const Point<T>& b) const
{
    std::span point = GetPoint(a_idx);

    Point<T> a(Eigen::Map<const Point<T>>(point.data(), point.size()));

    return (a - b).squaredNorm();
}


template class OnDiskGraph<float>;

}  // namespace urukrama