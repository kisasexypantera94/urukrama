#include "on_disk_lloyd.hpp"
#include "bounded_sorted_vector.hpp"
#include "types.hpp"

#include "lib/logger/logger.hpp"

#include <tbb/flow_graph.h>
#include <tbb/parallel_for.h>

#include <atomic>
#include <random>
#include <ranges>

namespace urukrama {

// NOLINTBEGIN (cppcoreguidelines-pro-type-reinterpret-cast)
std::vector<std::vector<size_t>> ComputeClustersOnDisk(std::string_view fvecs_filename,
                                                       size_t k,
                                                       size_t niter,
                                                       size_t l)
{
    mio::mmap_source mmap(fvecs_filename);

    int dimension = *reinterpret_cast<const int*>(mmap.data());
    size_t num_points = mmap.size() / ((dimension + 1) * 4);

    std::vector<Point<float>> centroids(k, Point<float>::Zero(dimension));

    // Random initialize
    {
        std::mt19937_64 random_engine{std::random_device{}()};
        std::uniform_int_distribution<size_t> dis(0, centroids.size() - 1);

        for (size_t offset = 0, i = 0; offset < mmap.size(); offset += (dimension + 1) * 4, ++i) {
            Eigen::Map<const Point<float>> point(reinterpret_cast<const float*>(mmap.data() + 4 + offset), dimension);

            auto& centroid = centroids[dis(random_engine)];

            centroid += (point - centroid) / (i + 1);
            centroid = centroid.unaryExpr([](const float x) { return std::isfinite(x) ? x : 0; });
        }
    }

    for (size_t ni = 0; ni < niter; ++ni) {
        std::atomic<float> objective = 0;
        std::vector<Point<float>> new_centroids(k, Point<float>::Zero(dimension));
        std::vector<std::atomic<size_t>> cnt(k);

        tbb::flow::graph g;

        const auto update_centroid = [&](const std::tuple<size_t, Point<float>>& t) {
            const auto& [centroid_idx, point] = t;
            auto& new_centroid = new_centroids[centroid_idx];
            new_centroid += (point - new_centroid) / (1 + cnt[centroid_idx]++);
            new_centroid = new_centroid.unaryExpr([](float x) { return std::isfinite(x) ? x : 0; });
        };

        // TBR: Should be equivalent to strand executor I guess
        std::vector update_serializers(
            k,
            tbb::flow::function_node<std::tuple<size_t, Point<float>>>(g, 1, update_centroid));

        tbb::parallel_for(tbb::blocked_range<size_t>(0, num_points - 1), [&](tbb::blocked_range<size_t> r) {
            for (size_t p_idx = r.begin(); p_idx < r.end(); ++p_idx) {
                size_t offset = p_idx * (dimension + 1) * 4;

                Eigen::Map<const Point<float>> point(reinterpret_cast<const float*>(mmap.data() + 4 + offset),
                                                     dimension);

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

        ksp::log::Info("Lloyd: iteration=[{}], objective=[{}]", ni + 1, objective.load() / num_points);
    }

    std::vector<std::vector<size_t>> cluster_to_points(k);
    for (size_t p_idx = 0; p_idx < num_points; ++p_idx) {
        size_t offset = p_idx * (dimension + 1) * 4;

        BoundedSortedVector<std::pair<float, size_t>> closest(l);

        Eigen::Map<const Point<float>> point(reinterpret_cast<const float*>(mmap.data() + 4 + offset), dimension);

        for (const auto& [c_idx, c]: centroids | std::views::enumerate) {
            closest.insert({(c - point).squaredNorm(), c_idx});
        }

        for (const auto& [_, c_idx]: closest) {
            cluster_to_points[c_idx].emplace_back(p_idx);
        }
    }

    return cluster_to_points;
}
// NOLINTEND (cppcoreguidelines-pro-type-reinterpret-cast)

}  // namespace urukrama
