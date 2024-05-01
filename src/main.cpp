#include "faiss.hpp"
#include "in_memory_graph.hpp"
#include "on_disk_fvecs.hpp"
#include "on_disk_graph.hpp"
#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <faiss/IndexPQ.h>
#include <faiss/index_io.h>

#include <ranges>
#include <vector>

constexpr auto FVECS_FILENAME = "/data/deep1M_base.fvecs";
constexpr auto GRAPH_INDEX_FILENAME = "deep1m.index";
constexpr auto PQ_INDEX_FILENAME = "deep1m.pq_index";
constexpr size_t R = 70;
constexpr size_t L = 75;
constexpr size_t K = 16;
constexpr size_t NITER = 50;

void Search(std::span<const urukrama::Point<float>> queries)
{
    urukrama::OnDiskGraph<float> on_disk_graph(GRAPH_INDEX_FILENAME);
    auto index_pq = dynamic_cast<faiss::IndexPQ*>(faiss::read_index(PQ_INDEX_FILENAME));

    size_t total_search_time = 0;
    size_t cnt = 0;
    size_t n_search = 50000;

    for (const auto& [p_idx, p]: queries | std::views::enumerate | std::views::take(n_search)) {
        using namespace std::chrono;

        auto t0 = high_resolution_clock::now();
        auto top = on_disk_graph.GreedySearchWithPQ(*index_pq, p, 10);
        auto t1 = high_resolution_clock::now();

        total_search_time += duration_cast<microseconds>(t1 - t0).count();
        cnt += (top.front().first == 0);

        for (const auto [dist, n_idx]: top | std::views::take(1)) {
            ksp::log::Info("Top entry: p_idx=[{}], n_idx=[{}], dist=[{}]", p_idx, n_idx, dist);
        }
    }

    ksp::log::Info("Mean processing time: {}us", total_search_time / n_search);
    ksp::log::Info("Recall: {}", float(cnt) / float(n_search));
}

void BuildIndexOnDisk(std::string_view filename)
{
    // Build Graph index
    {
        urukrama::OnDiskFVecs<false> on_disk_fvecs(FVECS_FILENAME);
        std::vector clusters = on_disk_fvecs.ComputeClusters(K, NITER);
        size_t medoid_idx = on_disk_fvecs.FindMedoid();

        auto on_disk_graph = urukrama::OnDiskGraph<float, true>::Init(GRAPH_INDEX_FILENAME,
                                                                      medoid_idx,
                                                                      on_disk_fvecs.Dimension(),
                                                                      on_disk_fvecs.Size(),
                                                                      R,
                                                                      L);

        for (const auto& [cluster_idx, p_indices]: clusters | std::views::enumerate) {
            std::vector<urukrama::Point<float>> points;

            for (size_t p_idx: p_indices) {
                auto point_span = on_disk_fvecs.GetPoint(p_idx);

                points.emplace_back(
                    Eigen::Map<const urukrama::Point<float>>(point_span.data(), Eigen::Index(point_span.size())));
            }

            ksp::log::Info("Building graph for cluster: cluster_idx=[{}], cluster_size=[{}]",
                           cluster_idx,
                           p_indices.size());

            urukrama::InMemoryGraph graph(std::vector{points}, 70, 75);

            on_disk_graph.Merge(graph, p_indices);
        }
    }

    // Build PQ index
    {
        auto [flat_points, dimension, num_points] = urukrama::FVecsRead(FVECS_FILENAME);
        auto index_pq = urukrama::BuildIndexPQ(flat_points, dimension);
        faiss::write_index(&index_pq, PQ_INDEX_FILENAME);
    }
}

auto LoadPoints()
{
    auto [flat_points, dimension, num_points] = urukrama::FVecsRead(FVECS_FILENAME);
    std::vector<urukrama::Point<float>> points;

    for (const auto& p: flat_points | std::views::chunk(dimension)) {
        points.emplace_back(Eigen::Map<const urukrama::Point<float>>(p.data(), Eigen::Index(dimension)));
    }

    return points;
}

int main()
{
    urukrama::OnDiskFVecs<true> on_disk_fvecs(FVECS_FILENAME);
    on_disk_fvecs.Shuffle();

    BuildIndexOnDisk(FVECS_FILENAME);

    auto points = LoadPoints();
    ksp::log::Info("Loaded points: size=[{}]", points.size());

    Search(points);

    return 0;
}
