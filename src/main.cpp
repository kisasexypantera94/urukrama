#include "in_memory_graph.hpp"
#include "on_disk_graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <random>
#include <ranges>
#include <vector>

int main()
{
    auto points = [] {
        auto [flat_points, dimension, num_points] = urukrama::FVecsRead("/data/deep1M_base.fvecs");
        std::vector<urukrama::Point<float>> points;

        for (const auto& p: flat_points | std::views::chunk(dimension)) {
            points.emplace_back(Eigen::Map<const urukrama::Point<float>>(p.data(), Eigen::Index(dimension)));
        }

        std::mt19937_64 random_engine{std::random_device{}()};
        std::ranges::shuffle(points, random_engine);

        return points;
    }();

    ksp::log::Info("Loaded points: size=[{}]", points.size());

    urukrama::InMemoryGraph graph(std::vector{points}, 70, 75);
    urukrama::WriteOnDisk(graph, "deep1m.index");

    urukrama::OnDiskGraph<float> on_disk_graph("deep1m.index");

    size_t total_search_time = 0;
    size_t n_search = 100;

    for (const auto& [p_idx, p]: points | std::views::enumerate | std::views::take(n_search)) {
        using namespace std::chrono;

        auto t0 = high_resolution_clock::now();
        auto top = on_disk_graph.GreedySearch(p, 1);
        auto t1 = high_resolution_clock::now();

        total_search_time += duration_cast<microseconds>(t1 - t0).count();

        for (const auto [dist, n_idx]: top) {
            ksp::log::Info("Top entry: p_idx=[{}], n_idx=[{}], dist=[{}]", p_idx, n_idx, dist);
        }
    }

    ksp::log::Info("Mean processing time: {}us", total_search_time / n_search);

    return 0;
}
