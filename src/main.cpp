#include "graph.hpp"
#include "utils.hpp"

#include <chrono>
#include <ranges>
#include <vector>

int main()
{
    using namespace std::chrono;

    const auto points = [] {
        auto [flat_points, dimension, num_points] = urukrama::FVecsRead("/data/deep1M_base.fvecs");
        std::vector<urukrama::Point<float>> points;

        for (const auto& p: flat_points | std::views::chunk(dimension)) {
            points.emplace_back(Eigen::Map<urukrama::Point<float>>(p.data(), Eigen::Index(dimension)));
        }

        std::mt19937_64 random_engine{std::random_device{}()};
        std::ranges::shuffle(points, random_engine);

        return points;
    }();

    auto t0 = high_resolution_clock::now();
    urukrama::Graph gc(std::span{points}, 70, 75);
    auto duration = duration_cast<seconds>(high_resolution_clock::now() - t0).count();

    return 0;
}
