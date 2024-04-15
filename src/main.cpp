#include "faiss.hpp"
#include "graph.hpp"
#include "utils.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

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
        auto pq = ComputeProductQuantization(points, dimension, M);
        const auto [_, indices, distances] = ComputeClusters(points, dimension, NUM_BATCHES, l);

        std::vector<std::vector<urukrama::Point<uint8_t>>> batches(NUM_BATCHES);

        for (const auto& [p_idx, closest_clusters]:
             std::views::zip(indices, distances) | std::views::chunk(l) | std::views::enumerate) {
            std::optional<float> closest = std::nullopt;

            for (const auto& [cluster_idx, cluster_distance]: closest_clusters) {
                // if (closest.has_value()) {
                //     // std::cout << (cluster_distance - closest.value()) / cluster_distance << std::endl;
                //     if ((cluster_distance - closest.value()) / cluster_distance > 0.1) {
                //         // std::cout << "good" << std::endl;
                //         break;
                //     }
                // } else {
                //     closest.emplace(cluster_distance);
                // }

                // std::cout << p_idx << std::endl;
                batches[cluster_idx].push_back(Eigen::Map<urukrama::Point<uint8_t>>(pq.data() + p_idx * M, M));
            }
        }

        std::mt19937_64 random_engine{std::random_device{}()};

        for (auto& batch: batches) {
            std::cout << batch.size() << std::endl;
            std::ranges::shuffle(batch, random_engine);
        }

        return batches;
    }();

    boost::asio::thread_pool pool;

    for (const auto& batch: batches) {
        boost::asio::post(pool, [&] { urukrama::GraphConstructor gc(std::span{batch}, 70, 75); });
    }

    pool.join();

    return 0;
}
