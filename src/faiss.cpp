#include "faiss.hpp"

#include <faiss/Clustering.h>
#include <faiss/IndexFlat.h>
#include <faiss/IndexPQ.h>
#include <faiss/impl/ProductQuantizer.h>

namespace urukrama {

using faiss::idx_t;

std::tuple<std::vector<float>, std::vector<int64_t>, std::vector<float>> ComputeClustersCPU(
    const std::vector<float>& vecs,
    size_t dim,
    size_t number_of_clusters,
    size_t k,
    size_t number_of_iterations,
    bool verbose)
{
    faiss::IndexFlatL2 index{idx_t(dim)};

    faiss::ClusteringParameters clustering_params;
    clustering_params.niter = int(number_of_iterations);
    clustering_params.verbose = verbose;  // print out per-iteration stats

    // For spherical k-means, use GpuIndexFlatIP and set cp.spherical = true

    // By default faiss only samples 256 vectors per centroid, in case
    // you are asking for too few centroids for too many vectors.
    // e.g., numberOfClusters = 1000, numVecsToCluster = 1000000 would
    // only sample 256000 vectors.
    //
    // You can override this to use any number of clusters
    // cp.max_points_per_centroid =
    //   ((numVecsToCluster + numberOfClusters - 1) / numberOfClusters);

    faiss::Clustering k_means(int(dim), int(number_of_clusters), clustering_params);

    // do the work!
    k_means.train(idx_t(vecs.size() / dim), vecs.data(), index);

    // kMeans.centroids contains the resulting cluster centroids (on CPU)
    // printf("centroid 3 dim 6 is %f\n", kMeans.centroids[3 * dim + 6]);
    std::vector<float> distances(vecs.size() * k / dim);
    std::vector<int64_t> indexes(vecs.size() * k / dim);

    index.search(idx_t(vecs.size() / dim), vecs.data(), idx_t(k), distances.data(), indexes.data());

    return std::make_tuple(std::move(k_means.centroids), std::move(indexes), std::move(distances));
}

std::tuple<std::vector<float>, std::vector<int64_t>, std::vector<float>> ComputeClusters(const std::vector<float>& vecs,
                                                                                         size_t dim,
                                                                                         size_t number_of_clusters,
                                                                                         size_t k,
                                                                                         size_t number_of_iterations,
                                                                                         bool verbose)
{
    return ComputeClustersCPU(vecs, dim, number_of_clusters, k, number_of_iterations, verbose);
}

faiss::IndexPQ BuildIndexPQ(const std::vector<float>& vecs, size_t dim, size_t M, size_t nbits)
{
    faiss::IndexPQ index_pq(int(dim), M, nbits);

    index_pq.train(idx_t(vecs.size() / dim), vecs.data());
    index_pq.add(idx_t(vecs.size() / dim), vecs.data());

    return index_pq;
}

}  // namespace urukrama