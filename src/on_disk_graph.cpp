#include "on_disk_graph.hpp"
#include "bounded_sorted_vector.hpp"
#include "in_memory_graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <cassert>
#include <format>
#include <fstream>
#include <iostream>
#include <ranges>

namespace urukrama {


template <typename T, bool MUTABLE>
OnDiskGraph<T, MUTABLE>::OnDiskGraph(std::string_view index_filename)
    : m_mmap(MemMap(index_filename))
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

template <typename T, bool MUTABLE>
std::vector<std::pair<T, size_t>> OnDiskGraph<T, MUTABLE>::GreedySearchWithPQ(const faiss::IndexPQ& index_pq,
                                                                              const Point<T>& query,
                                                                              size_t k) const
{
    auto computer = index_pq.get_FlatCodesDistanceComputer();
    computer->set_query(query.data());

    auto top = GreedySearchInternal([&](size_t p_idx) { return (*computer)(faiss::idx_t(p_idx)); }, k);

    for (auto& [distance, p_idx]: top) {
        distance = FullPrecisionDistance(p_idx, query);
    }

    std::sort(top.begin(), top.end());

    return top;
}

template <typename T, bool MUTABLE>
std::vector<std::pair<T, size_t>> OnDiskGraph<T, MUTABLE>::GreedySearch(const Point<T>& query, size_t k) const
{
    return GreedySearchInternal([&](size_t p_idx) { return FullPrecisionDistance(p_idx, query); }, k);
}

template <typename T, bool MUTABLE>
std::vector<std::pair<T, size_t>> OnDiskGraph<T, MUTABLE>::GreedySearchInternal(auto distance_func, size_t k) const
{
    if (m_medoid_idx == DUMMY_P_IDX) {
        ksp::log::Warn("Medoid is not set");
        return {};
    }

    HashSet<size_t> fast_visited;
    fast_visited.reserve(m_L * 2);

    std::vector<std::pair<T, size_t>> visited;
    visited.reserve(m_L * 2);

    BoundedSortedVector<std::pair<T, size_t>> candidates(m_L);
    candidates.reserve(m_L + 1);
    candidates.insert({distance_func(m_medoid_idx), m_medoid_idx});

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
                candidates.insert({distance_func(n_idx), n_idx});
            }
        }
    }

    std::sort(visited.begin(), visited.end());
    candidates.resize(k);

    return candidates;
}

template <typename T, bool MUTABLE>
void OnDiskGraph<T, MUTABLE>::Write(const InMemoryGraph<T>& in_mem_graph, std::string_view filename)
{
    const auto& [R, L, points, medoid_idx, n_out] = in_mem_graph;

    WriteInternal(filename,
                  points,
                  points.front().size(),
                  points.size(),
                  R,
                  L,
                  medoid_idx,
                  [&n_out = n_out](size_t p_idx) { return n_out[p_idx]; });
}

template <typename T, bool MUTABLE>
void OnDiskGraph<T, MUTABLE>::WriteEmpty(
    std::string_view filename, size_t dimension, size_t num_points, size_t R, size_t L)
{
    WriteInternal(filename,
                  // generates N zero points with required dimension
                  std::views::iota(size_t(0), num_points) | std::views::transform([&](size_t) {
                      return std::views::repeat(0) | std::views::take(dimension);
                  }),
                  dimension,
                  num_points,
                  R,
                  L,
                  DUMMY_P_IDX,
                  [](size_t p_idx) { return std::vector<size_t>{}; });
}

template <typename T, bool MUTABLE>
void OnDiskGraph<T, MUTABLE>::WriteInternal(std::string_view filename,
                                            const auto& points,
                                            size_t dimension,
                                            size_t num_points,
                                            size_t R,
                                            size_t L,
                                            size_t medoid_idx,
                                            auto get_n_out)
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

    const auto points_offset = write(num_points, dimension, GetDataType(), R, L, medoid_idx);
    static_assert(points_offset.value == POINTS_OFFSET);

    for (const auto& [p_idx, p]: points | std::views::enumerate) {
        for (const T x: p) {
            write(x);
        }

        for (const size_t n_idx: get_n_out(p_idx)) {
            write(n_idx);
        }

        for (size_t i = 0; i < R - get_n_out(p_idx).size(); ++i) {
            write(DUMMY_P_IDX);
        }
    }
}

template <typename T, bool MUTABLE>
void OnDiskGraph<T, MUTABLE>::Merge(const InMemoryGraph<T>& in_mem_graph, std::span<const size_t> indices)
    requires(MUTABLE)

{
    const auto& [R, L, points, medoid_idx, n_out] = in_mem_graph;

    assert(m_R == R);
    assert(m_L == L);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<size_t&>(m_mmap[MEDOID_IDX_OFFSET]) = medoid_idx;
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    for (const auto& [p_idx, p]: std::views::zip(indices, points)) {
        // update point
        {
            auto point = GetPointMut(p_idx);

            for (const auto& [dst, src]: std::views::zip(point, p)) {
                dst = src;
            }
        }

        // update neighbors
        {
            auto nbrs = GetPointNeighborsMut(p_idx);


            BoundedSortedVector<std::pair<T, size_t>> new_n_out(R);

            for (size_t n_idx: nbrs | std::views::filter([](size_t n_idx) { return n_idx != DUMMY_P_IDX; })) {
                new_n_out.insert({FullPrecisionDistance(n_idx, p), n_idx});
            }

            for (size_t n_idx: n_out[p_idx]) {
                new_n_out.insert({FullPrecisionDistance(n_idx, p), n_idx});
            }

            for (const auto& [dst, src]: std::views::zip(nbrs, new_n_out | std::views::values)) {
                dst = src;
            }
        }
    }
}

template <typename T, bool MUTABLE>
std::span<const float> OnDiskGraph<T, MUTABLE>::GetPoint(size_t p_idx) const
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t point_offset = GetPointOffset(p_idx);
    auto point = std::span{reinterpret_cast<const float*>(&m_mmap[point_offset]), m_points_dimension};
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    return point;
}

template <typename T, bool MUTABLE>
std::span<const size_t> OnDiskGraph<T, MUTABLE>::GetPointNeighbors(size_t p_idx) const
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t point_neighbors_offset = GetPointNeighborsOffset(p_idx);
    auto point_neighbors = std::span{reinterpret_cast<const size_t*>(&m_mmap[point_neighbors_offset]), m_R};
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    return point_neighbors;
}

template <typename T, bool MUTABLE>
std::span<float> OnDiskGraph<T, MUTABLE>::GetPointMut(size_t p_idx)
    requires(MUTABLE)
{
    auto span = GetPoint(p_idx);
    return std::span{const_cast<float*>(span.data()), span.size()};
}

template <typename T, bool MUTABLE>
std::span<size_t> OnDiskGraph<T, MUTABLE>::GetPointNeighborsMut(size_t p_idx)
    requires(MUTABLE)
{
    auto span = GetPointNeighbors(p_idx);
    return std::span{const_cast<size_t*>(span.data()), span.size()};
}


template <typename T, bool MUTABLE>
size_t OnDiskGraph<T, MUTABLE>::GetPointOffset(size_t p_idx) const
{
    return POINTS_OFFSET + p_idx * (m_points_dimension * sizeof(T) + m_R * sizeof(size_t));
}

template <typename T, bool MUTABLE>
size_t OnDiskGraph<T, MUTABLE>::GetPointNeighborsOffset(size_t p_idx) const
{
    return GetPointOffset(p_idx) + m_points_dimension * sizeof(T);
}

template <typename T, bool MUTABLE>
template <typename U>
U OnDiskGraph<T, MUTABLE>::ReadAs(size_t offset) const
{
    U val;
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    std::memcpy(reinterpret_cast<char*>(&val), &m_mmap[offset], sizeof(val));
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
    return val;
}

template <typename T, bool MUTABLE>
OnDiskGraph<T, MUTABLE>::DataType OnDiskGraph<T, MUTABLE>::GetDataType()
{
    if constexpr (std::same_as<T, float>) {
        return DataType::F32;
    }
}

template <typename T, bool MUTABLE>
T OnDiskGraph<T, MUTABLE>::FullPrecisionDistance(const size_t a_idx, const Point<T>& b) const
{
    std::span point = GetPoint(a_idx);

    Point<T> a(Eigen::Map<const Point<T>>(point.data(), point.size()));

    return (a - b).squaredNorm();
}


template class OnDiskGraph<float, false>;
template class OnDiskGraph<float, true>;

}  // namespace urukrama