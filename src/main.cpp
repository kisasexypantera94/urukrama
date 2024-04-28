#include "faiss.hpp"
#include "in_memory_graph.hpp"
#include "on_disk_graph.hpp"
#include "on_disk_lloyd.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <faiss/IndexPQ.h>
#include <faiss/index_io.h>

#include <random>
#include <ranges>
#include <vector>

void Search(std::span<const urukrama::Point<float>> queries)
{
    urukrama::OnDiskGraph<float> on_disk_graph("deep1m.index");
    auto index_pq = dynamic_cast<faiss::IndexPQ*>(faiss::read_index("deep1m.index_pq"));

    size_t total_search_time = 0;
    size_t n_search = 100;

    for (const auto& [p_idx, p]: queries | std::views::enumerate | std::views::take(n_search)) {
        using namespace std::chrono;

        auto t0 = high_resolution_clock::now();
        auto top = on_disk_graph.GreedySearchWithPQ(*index_pq, p, 10);
        auto t1 = high_resolution_clock::now();

        total_search_time += duration_cast<microseconds>(t1 - t0).count();

        for (const auto [dist, n_idx]: top | std::views::take(1)) {
            ksp::log::Info("Top entry: p_idx=[{}], n_idx=[{}], dist=[{}]", p_idx, n_idx, dist);
        }
    }

    ksp::log::Info("Mean processing time: {}us", total_search_time / n_search);
}

void BuildIndex(const std::vector<urukrama::Point<float>>& points)
{
    urukrama::InMemoryGraph graph(std::vector{points}, 70, 75);
    urukrama::WriteOnDisk(graph, "deep1m.index");

    // TODO: inplace
    std::vector<float> flat_points;
    for (const auto& x: points | std::views::join) {
        flat_points.emplace_back(x);
    }

    auto index_pq = urukrama::BuildIndexPQ(flat_points, points.front().size());
    faiss::write_index(&index_pq, "deep1m.index_pq");
}

int main()
{
    auto points = [] {
        urukrama::ComputeClustersOnDisk("/data/deep1M_base.fvecs", 40, 50);
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

    BuildIndex(points);
    Search(points);

    return 0;
}
