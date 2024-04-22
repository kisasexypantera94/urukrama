#include "in_memory_graph.hpp"
#include "on_disk_graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

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

    urukrama::InMemoryGraph graph(std::vector{points}, 70, 75);
    urukrama::WriteOnDisk(graph, "deep1m.index");

    urukrama::OnDiskGraph<float> on_disk_graph("deep1m.index");

    for (const auto& [p_idx, p]: points | std::views::enumerate | std::views::take(10)) {
        for (const auto [dist, n_idx]: on_disk_graph.GreedySearch(p, 1)) {
            ksp::log::Info("Top entry: p_idx=[{}], n_idx=[{}], dist=[{}]", p_idx, n_idx, dist);
        }
    }

    return 0;
}
