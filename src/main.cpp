#include "faiss.hpp"
#include "graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include <chrono>
#include <optional>
#include <ranges>
#include <vector>

constexpr size_t NUM_BATCHES = 8;
constexpr size_t l = 2;
constexpr size_t M = 32;

auto PrepareBatches()
{
    auto [points, dimension, num_points] = urukrama::FVecsRead("../data/deep1b/deep1M_base.fvecs");
    // auto pq = ComputeProductQuantization(points, dimension, M);
    const auto [clusters, indices, distances] = ComputeClusters(points, dimension, NUM_BATCHES, l);

    std::vector<std::vector<urukrama::Point<float>>> batches(NUM_BATCHES);

    const std::span points_span = points;

    for (const auto& [p_idx, closest_clusters]:
         std::views::zip(indices, distances) | std::views::chunk(l) | std::views::enumerate) {
        std::optional<float> closest = std::nullopt;

        for (const auto& [cluster_idx, cluster_distance]: closest_clusters) {
            batches[cluster_idx].emplace_back(
                Eigen::Map<urukrama::Point<float>>(points_span.subspan(p_idx * dimension).data(),
                                                   Eigen::Index(dimension)));
        }
    }

    std::mt19937_64 random_engine{std::random_device{}()};

    for (auto& batch: batches) {
        std::ranges::shuffle(batch, random_engine);
    }

    return batches;
}

int main()
{
    const auto batches = PrepareBatches();

    boost::asio::thread_pool pool(8);
    std::atomic<size_t> sum_proc_time = 0;

    for (const auto& [batch_idx, batch]: batches | std::views::enumerate) {
        boost::asio::post(pool, [&, &batch = batch, batch_idx = batch_idx] {
            using namespace std::chrono;


            auto t0 = high_resolution_clock::now();
            urukrama::Graph gc(std::span{batch}, 70, 75);
            auto duration = duration_cast<seconds>(high_resolution_clock::now() - t0).count();

            sum_proc_time += duration;

            ksp::log::Info("Processed batch: batch_idx=[{}], batch_size=[{}], time=[{}s]",
                           batch_idx,
                           batch.size(),
                           duration);
        });
    }

    pool.join();

    ksp::log::Info("Mean processing time: {}", sum_proc_time / batches.size());

    return 0;
}
