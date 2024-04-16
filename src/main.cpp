#include "faiss.hpp"
#include "graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include <chrono>
#include <iostream>
#include <optional>
#include <ranges>
#include <vector>


constexpr size_t NUM_BATCHES = 40;
constexpr size_t l = 2;
constexpr size_t M = 32;

int main()
{
    const auto batches = [] {
        auto [points, dimension, _] = urukrama::FVecsRead("/data/deep1m_base.fvecs");
        // auto pq = ComputeProductQuantization(points, dimension, M);
        const auto [_, indices, distances] = ComputeClusters(points, dimension, NUM_BATCHES, l);

        std::vector<std::vector<urukrama::Point<float>>> batches(NUM_BATCHES);

        for (const auto& [p_idx, closest_clusters]:
             std::views::zip(indices, distances) | std::views::chunk(l) | std::views::enumerate) {
            std::optional<float> closest = std::nullopt;

            for (const auto& [cluster_idx, cluster_distance]: closest_clusters) {
                // if (closest.has_value()) {
                //     // std::cout << (cluster_distance - closest.value()) / cluster_distance << std::endl;
                //     if ((cluster_distance - closest.value()) / cluster_distance > 0.5) {
                //         // std::cout << "good" << std::endl;
                //         break;
                //     }
                // } else {
                //     closest.emplace(cluster_distance);
                // }

                // std::cout << p_idx << std::endl;
                batches[cluster_idx].push_back(
                    Eigen::Map<urukrama::Point<float>>(points.data() + p_idx * dimension, dimension));
            }
        }

        std::mt19937_64 random_engine{std::random_device{}()};

        for (auto& batch: batches) {
            std::ranges::shuffle(batch, random_engine);
        }

        // std::sort(batches.begin(), batches.end(), [](const auto& a, const auto& b) { return a.size() < b.size(); });

        return batches;
    }();


    boost::asio::thread_pool pool(4);
    std::atomic<size_t> sum_proc_time = 0;

    for (const auto& [batch_idx, batch]: batches | std::views::enumerate) {
        boost::asio::post(pool, [&, &batch = batch, batch_idx = batch_idx] {
            using namespace std::chrono;


            auto t0 = high_resolution_clock::now();
            urukrama::GraphConstructor gc(std::span{batch}, 70, 75);
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
