#include "on_disk_fvecs.hpp"
#include "bounded_sorted_vector.hpp"
#include "types.hpp"

#include "lib/logger/logger.hpp"

#include <tbb/flow_graph.h>
#include <tbb/parallel_for.h>

#include <atomic>
#include <random>
#include <ranges>

namespace urukrama {

template <bool MUTABLE>
OnDiskFVecs<MUTABLE>::OnDiskFVecs(std::string_view filename)
    : m_mmap(MemMap(filename))
    , m_dimension(
          // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
          *reinterpret_cast<const int*>(m_mmap.data())
          // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
          )
    , m_num_points(m_mmap.size() / (PointByteLength()))
{
}

template <bool MUTABLE>
std::vector<std::vector<size_t>> OnDiskFVecs<MUTABLE>::ComputeClusters(size_t k, size_t niter, size_t l) const
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
{
    std::vector<Point<float>> centroids(k, Point<float>::Zero(m_dimension));

    // Random initialize
    {
        std::mt19937_64 random_engine{std::random_device{}()};
        std::uniform_int_distribution<size_t> dis(0, centroids.size() - 1);

        for (size_t p_idx = 0; p_idx < m_num_points; ++p_idx) {
            auto point_span = GetPoint(p_idx);
            Eigen::Map<const Point<float>> point(point_span.data(), point_span.size());

            auto& centroid = centroids[dis(random_engine)];

            centroid += (point - centroid) / (p_idx + 1);
            centroid = centroid.unaryExpr([](const float x) { return std::isfinite(x) ? x : 0; });
        }
    }

    for (size_t ni = 0; ni < niter; ++ni) {
        std::atomic<float> objective = 0;
        std::vector<Point<float>> new_centroids(k, Point<float>::Zero(m_dimension));
        std::vector<std::atomic<size_t>> cnt(k);

        const auto update_centroid = [&](const std::tuple<size_t, Point<float>>& t) {
            const auto& [centroid_idx, point] = t;
            auto& new_centroid = new_centroids[centroid_idx];
            new_centroid += (point - new_centroid) / (1 + cnt[centroid_idx]++);
            new_centroid = new_centroid.unaryExpr([](float x) { return std::isfinite(x) ? x : 0; });
        };

        tbb::flow::graph g;

        // TBR: Should be equivalent to strand executor I guess
        std::vector update_serializers(
            k,
            tbb::flow::function_node<std::tuple<size_t, Point<float>>>(g, 1, update_centroid));

        tbb::parallel_for(tbb::blocked_range<size_t>(0, m_num_points - 1), [&](tbb::blocked_range<size_t> r) {
            for (size_t p_idx = r.begin(); p_idx < r.end(); ++p_idx) {
                auto point_span = GetPoint(p_idx);
                Eigen::Map<const Point<float>> point(point_span.data(), point_span.size());

                const auto centroid_it = std::ranges::min_element(centroids, {}, [&](const Point<float>& c) {
                    return (c - point).squaredNorm();
                });

                objective += (*centroid_it - point).squaredNorm();

                size_t centroid_idx = std::distance(centroids.begin(), centroid_it);

                while (not update_serializers[centroid_idx].try_put({centroid_idx, point})) {
                }
            }
        });

        g.wait_for_all();

        std::exchange(centroids, std::move(new_centroids));

        ksp::log::Info("Lloyd: iteration=[{}], objective=[{}]", ni + 1, objective.load() / float(m_num_points));
    }

    std::vector<std::vector<size_t>> cluster_to_points(k);
    for (size_t p_idx = 0; p_idx < m_num_points; ++p_idx) {
        auto point_span = GetPoint(p_idx);
        Eigen::Map<const Point<float>> point(point_span.data(), point_span.size());

        BoundedSortedVector<std::pair<float, size_t>> closest(l);

        for (const auto& [c_idx, c]: centroids | std::views::enumerate) {
            closest.insert({(c - point).squaredNorm(), c_idx});
        }

        for (const auto& [_, c_idx]: closest) {
            cluster_to_points[c_idx].emplace_back(p_idx);
        }
    }

    return cluster_to_points;
}
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

template <bool MUTABLE>
size_t OnDiskFVecs<MUTABLE>::FindMedoid() const
{
    Point<float> centroid(m_dimension);

    for (size_t p_idx = 0; p_idx < m_num_points; ++p_idx) {
        auto point_span = GetPoint(p_idx);
        Eigen::Map<const Point<float>> point(point_span.data(), point_span.size());

        centroid += (point - centroid) / (p_idx + 1);
        centroid = centroid.unaryExpr([](float x) { return std::isfinite(x) ? x : 0; });
    }

    size_t medoid_idx = 0;
    float min_distance = std::numeric_limits<float>::max();
    for (size_t p_idx = 0; p_idx < m_num_points; ++p_idx) {
        auto point_span = GetPoint(p_idx);
        Eigen::Map<const Point<float>> point(point_span.data(), point_span.size());

        if (auto distance = (point - centroid).squaredNorm(); distance < min_distance) {
            min_distance = distance;
            medoid_idx = p_idx;
        }
    }

    ksp::log::Info("Found medoid on disk: idx=[{}], dist=[{}]", medoid_idx, min_distance);

    return medoid_idx;
}

template <bool MUTABLE>
void OnDiskFVecs<MUTABLE>::Shuffle()
    requires(MUTABLE)
{
    std::mt19937_64 random_engine{std::random_device{}()};
    std::uniform_int_distribution<size_t> dis(0, m_num_points - 1);

    for (size_t p_idx = 0; p_idx < m_num_points; ++p_idx) {
        size_t swap_p_idx = dis(random_engine);

        for (const auto& [x, y]: std::views::zip(GetPointMut(p_idx), GetPointMut(swap_p_idx))) {
            std::swap(x, y);
        }
    }
}

template <bool MUTABLE>
std::span<const float> OnDiskFVecs<MUTABLE>::GetPoint(size_t p_idx) const
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    return std::span{reinterpret_cast<const float*>(m_mmap.data() + p_idx * PointByteLength() + sizeof(float)),
                     m_dimension};
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
}

template <bool MUTABLE>
std::span<float> OnDiskFVecs<MUTABLE>::GetPointMut(size_t p_idx)
    requires(MUTABLE)
{
    auto span = GetPoint(p_idx);
    return std::span{const_cast<float*>(span.data()), span.size()};
}

template <bool MUTABLE>
size_t OnDiskFVecs<MUTABLE>::PointByteLength() const
{
    return (m_dimension + 1) * sizeof(float);
}

template <bool MUTABLE>
size_t OnDiskFVecs<MUTABLE>::Dimension() const
{
    return m_dimension;
}

template <bool MUTABLE>
size_t OnDiskFVecs<MUTABLE>::Size() const
{
    return m_num_points;
}

template class OnDiskFVecs<false>;
template class OnDiskFVecs<true>;

}  // namespace urukrama