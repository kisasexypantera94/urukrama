#include "graph.hpp"
#include "utils.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <vector>


constexpr size_t NUM_BATCHES = 40;

int main()
{
    auto points = urukrama::FVecsRead("/data/deep1m_base.fvecs");

    std::mt19937_64 random_engine{std::random_device{}()};
    std::ranges::shuffle(points, random_engine);

    size_t batch_size = points.size() / NUM_BATCHES;

    boost::asio::thread_pool pool;

    const std::span points_span = std::as_const(points);

    for (size_t i = 0; i < points.size(); i += batch_size) {
        boost::asio::post(pool, [=] { urukrama::GraphConstructor gc(points_span.subspan(i, batch_size), 70); });
    }

    pool.join();

    return 0;
}
