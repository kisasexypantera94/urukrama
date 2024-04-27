#include "on_disk_lloyd.hpp"
#include "types.hpp"

#include "lib/logger/logger.hpp"

#include <tbb/mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/queuing_mutex.h>

#include <atomic>
#include <random>

namespace urukrama {

// NOLINTBEGIN (cppcoreguidelines-pro-type-reinterpret-cast)
void ComputeClustersOnDisk(std::string_view fvecs_filename, size_t k, size_t niter)
{
    mio::mmap_source mmap(fvecs_filename);

    int dimension = *reinterpret_cast<const int*>(mmap.data());
    size_t num_points = mmap.size() / ((dimension + 1) * 4);

    std::vector<Point<float>> centroids(k, Point<float>::Zero(dimension));

    std::mt19937_64 random_engine{std::random_device{}()};
    std::uniform_int_distribution<size_t> dis(0, centroids.size() - 1);

    for (size_t offset = 0, i = 0; offset < mmap.size(); offset += (dimension + 1) * 4, ++i) {
        Eigen::Map<const Point<float>> point(reinterpret_cast<const float*>(mmap.data() + 4 + offset), dimension);

        auto& centroid = centroids[dis(random_engine)];

        centroid += (point - centroid) / (i + 1);
        centroid = centroid.unaryExpr([](const float x) { return std::isfinite(x) ? x : 0; });
    }

    for (size_t i = 0; i < niter; ++i) {
        std::atomic<float> objective = 0;

        std::vector<Point<float>> new_centroids(k, Point<float>::Zero(dimension));
        std::vector<tbb::mutex> mtxs(k);
        std::vector<size_t> cnt(k, 0);

        tbb::parallel_for(tbb::blocked_range<size_t>(0, num_points - 1), [&](tbb::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                size_t offset = i * (dimension + 1) * 4;

                Eigen::Map<const Point<float>> point(reinterpret_cast<const float*>(mmap.data() + 4 + offset),
                                                     dimension);

                const auto centroid_it = std::ranges::min_element(centroids, {}, [&](const Point<float>& c) {
                    return (c - point).squaredNorm();
                });

                objective += (*centroid_it - point).squaredNorm();

                size_t centroid_idx = std::distance(centroids.begin(), centroid_it);

                tbb::mutex::scoped_lock lock(mtxs[centroid_idx]);
                auto& new_centroid = new_centroids[centroid_idx];
                new_centroid += (point - new_centroid) / (cnt[centroid_idx]++ + 1);
                new_centroid = new_centroid.unaryExpr([](const float x) { return std::isfinite(x) ? x : 0; });
            }
        });

        std::exchange(centroids, std::move(new_centroids));

        ksp::log::Info("Lloyd: iteration=[{}], objective=[{}]", i + 1, objective.load());
    }

    // return clusters;
}
// NOLINTEND (cppcoreguidelines-pro-type-reinterpret-cast)

}  // namespace urukrama
