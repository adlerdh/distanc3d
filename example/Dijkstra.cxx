#include "Dijkstra.h"
#include "Params.h"
#include "Util.h"

// Wrapper around C++ std::priority_queue
#include "queues/StdPriorityQueue.h"

// Uncomment to try other priority queue implementations:
// #include "queues/GPriorityQueue.h"
// #include "queues/IndexedPriorityQueue.h"
// #include "queues/JkdsPriorityQueue.h"
// #include "queues/UpdatablePriorityQueue.h"

#include <glm/glm.hpp>

#include <itkImage.h>
#include <itkImageFileReader.h>

#include <chrono>
#include <iostream>
#include <vector>

// ./example/Dijkstra --seed ../data/seed.nii.gz --image ../data/image.nii.gz --seg ../data/seg.nii.gz --total-dist ../data/total-dist.nii.gz --euclid-dist ../data/euclid-dist.nii.gz --image-dist ../data/image-dist.nii.gz --image-weight 0.0 --euclid-weight 1.0 --stop-euclid-dist 50

namespace
{
/// Greater-than comparator that provides a strict weak ordering on priority queue items
template<typename T>
struct GreaterThanComparer
{
  bool operator()(const d3d::QueueItem<T>& a, const d3d::QueueItem<T>& b) const
  {
    return a.second > b.second;
  }
};

class NullBuffer : public std::streambuf
{
public:
  int overflow(int c) override { return c; }
  std::streamsize xsputn(const char*, std::streamsize n) override { return n; }
};

class NullOstream : public std::ostream
{
public:
  NullOstream() : std::ostream(&m_buffer) {}
private:
  NullBuffer m_buffer;
};
} // namespace

int main(int argc, char* argv[])
{
  using namespace std::chrono;

  constexpr unsigned int Dim = 3;

  // Image voxel types, which could be made configurable
  using TVoxel = float; // image voxels
  using TLabel = unsigned char; // labels of sources and segmentation voxels
  using TMask = unsigned char; // mask voxels
  using TDist = float; // distances

  using TImage = itk::Image<TVoxel, Dim>; // for computing image-based distance
  using TLabelImage = itk::Image<TLabel, Dim>; // for sources and segmentation labels
  using TMaskImage = itk::Image<TMask, Dim>; // for masking out computations
  using TDistImage = itk::Image<TDist, Dim>; // for distances and edge weights between voxels

#ifndef NDEBUG
  std::cout << "Attach debugger now. Press Enter to continue..." << std::endl;
  std::cin.get();
#endif

  const auto params = parse(argc, argv);

  if (!params) {
    return EXIT_FAILURE;
  }

  std::cout << *params << std::endl;

  const auto source = itk::ReadImage<TLabelImage>(params->source);
  if (!source) {
    std::cerr << "Null source image " << params->source << std::endl;
    return EXIT_FAILURE;
  }

  const auto seedEuclidDist = params->sourceEuclidDist
      ? itk::ReadImage<TDistImage>(*params->sourceEuclidDist) : nullptr;
  const auto image = params->image ? itk::ReadImage<TImage>(*params->image) : nullptr;
  const auto mask = params->mask ? itk::ReadImage<TMaskImage>(*params->mask) : nullptr;

  const auto size = source->GetLargestPossibleRegion().GetSize();
  const auto sp = source->GetSpacing();
  const auto N = static_cast<std::size_t>(size[0] * size[1] * size[2]);

  const glm::ivec3 dims{size[0], size[1], size[2]};
  const glm::dvec3 spacing(sp[0], sp[1], sp[2]);

  std::cout << "\nImage dimensions: " << dims.x << ", " << dims.y << ", " << dims.z
            << "\nVoxel spacing (mm): " << spacing.x << ", " << spacing.y << ", " << spacing.z << '\n';

  // Distance function between image voxels u and v -- when null, not used
  std::function<TDist(int u, int v)> computeImageDist = nullptr;

  if (image)
  {
    computeImageDist = [image, &params](int u, int v) -> TDist
    {
      const TVoxel* img = image->GetBufferPointer();
      switch (params->imageEdgeWeightType)
      {
      case ImageEdgeWeightType::AbsoluteDiff: return TDist(std::abs(img[u] - img[v]));
      case ImageEdgeWeightType::SquaredDiff: return TDist(std::pow(img[u] - img[v], 2.0));
      case ImageEdgeWeightType::Source: return TDist(img[u]);
      case ImageEdgeWeightType::Destination: return TDist(img[v]);
      case ImageEdgeWeightType::Minimum: return TDist(std::min(img[u], img[v]));
      case ImageEdgeWeightType::Mean: return TDist(0.5 * (img[u] + img[v]));
      case ImageEdgeWeightType::Maximum: return TDist(std::max(img[u], img[v]));
      }
      return 0;
    };

    std::cout << "Created computeImageDist" << std::endl;
  }

  // Function for determining whether a voxel index is valid -- when null, not used
  std::function<bool(int u)> isValidIndex = nullptr;

  if (mask)
  {
    isValidIndex = [mask](int u) -> bool {
      return (mask->GetBufferPointer()[u] > 0);
    };

    std::cout << "Created isValidIndex" << std::endl;
  }

  auto isSource = [&source] (int u) {
    return source->GetBufferPointer()[static_cast<std::size_t>(u)] > 0;
  };

  // Output images:
  std::vector<int> sourceIndex(N, -1); // Source index
  std::vector<int> parentIndex(N, -1); // Parent index
  std::vector<TDist> imageDist(N, 0); // Image distance (should be made optional)
  std::vector<TDist> euclidDist(N, 0); // Euclidean distance (should be made optional)

  std::span<const TDist> sourceEuclidDistSpan;

  if (seedEuclidDist && seedEuclidDist->GetBufferPointer()) {
    sourceEuclidDistSpan = std::span<const TDist>(seedEuclidDist->GetBufferPointer(), N);
  }

  // Choose a priority queue to use for running Dijkstra's algorithm:
  StdPriorityQueue<d3d::QueueItem<TDist>, GreaterThanComparer<TDist>> pqueue;

  // constexpr bool MutableKeys = true;
  // MyIndexedPriorityQueue<d3d::QueueItem<TDistance>, MutableKeys> queue(N);
  // UpdatablePriorityQueue<d3d::QueueItem<TDistance>, MutableKeys> queue;
  // GPriorityQueue<d3d::QueueItem<TDistance>, LessComparer> queue;
  // JkdsPriorityQueue<d3d::QueueItem<TDistance>, MutableKeys> queue;

  if (!d3d::validateData(
        dims,
        span<const TLabel>{source->GetBufferPointer(), N},
        sourceEuclidDistSpan,
        span<const TDist>{imageDist},
        span<const TDist>{euclidDist},
        span<const int>{sourceIndex},
        span<const int>{parentIndex},
        std::cerr))
  {
    std::cerr << "Failed to validate data" << std::endl;
    return EXIT_FAILURE;
  }

  if (!d3d::initializeData(
        span<const TLabel>{source->GetBufferPointer(), N},
        sourceEuclidDistSpan,
        span{imageDist},
        span{euclidDist},
        span{sourceIndex},
        span{parentIndex},
        pqueue))
  {
    std::cerr << "Failed to initialize priority queue" << std::endl;
    return EXIT_FAILURE;
  }

  std::optional<TDist> stoppingEuclidDist = std::nullopt;
  if (params->stoppingEuclideanDistance) {
    stoppingEuclidDist = *params->stoppingEuclideanDistance;
  }

  time_point<steady_clock> tick = steady_clock::now();

  NullOstream nullStream;

  const auto neighborhood = d3d::makeNeighborhood26(dims, spacing);

  const std::size_t updates = d3d::runDijkstra(
    dims, spacing, neighborhood,
    sourceEuclidDistSpan,
    span{sourceIndex},
    span{parentIndex},
    span{euclidDist},
    span{imageDist},
    static_cast<TDist>(params->euclideanWeight),
    static_cast<TDist>(params->imageWeight),
    pqueue,
    computeImageDist,
    isValidIndex,
    isSource,
    stoppingEuclidDist,
    params->debugInterval,
    std::cout);

  time_point<steady_clock> tock = steady_clock::now();

  std::cout << "\nExecution time: " << duration_cast<milliseconds>(tock - tick).count() << " ms"
            << "\nQueue updates: " << updates << std::endl;

  if (params->seg)
  {
    // Create segmentation label image:
    std::vector<TLabel> segLabel(N, 0);

    const bool ret = d3d::makeSegmentationImage(
      sourceIndex, span<const TLabel>{source->GetBufferPointer(), N}, span{segLabel});

    if (!ret) {
      std::cerr << "Error creating segmentation image" << std::endl;
      return EXIT_FAILURE;
    }

    writeImage<TLabelImage>(createImage(source, segLabel.data(), segLabel.size()), *params->seg);
  }

  if (params->euclidDist) {
    writeImage<TDistImage>(createImage(source, euclidDist.data(), euclidDist.size()), *params->euclidDist);
  }

  if (computeImageDist && params->imageDist) {
    writeImage<TDistImage>(createImage(source, imageDist.data(), imageDist.size()), *params->imageDist);
  }

  if (params->totalDist)
  {
    const auto imageWeight = static_cast<TDist>(params->imageWeight);
    const auto euclidWeight = static_cast<TDist>(params->euclideanWeight);

    // Compute total distance image:
    std::vector<TDist> totalDist(N, 0);

    if (computeImageDist && image) {
      for (std::size_t i = 0; i < N; ++i) {
        totalDist[i] = imageWeight * imageDist[i] + euclidWeight * euclidDist[i];
      }
    }
    else {
      for (std::size_t i = 0; i < N; ++i) {
        totalDist[i] = euclidDist[i];
      }
    }

    writeImage<TDistImage>(createImage(source, totalDist.data(), totalDist.size()), *params->totalDist);
  }

  if (params->destVoxel)
  {
    // Create and print shortest path:
    const int destIndex = d3d::coordToIndex(*params->destVoxel, dims);
    const std::vector<int> path = d3d::makeShortestPath(span<const int>{parentIndex}, destIndex);

    d3d::printShortestPath(std::cout, path, dims);

    if (params->pathImage) {
      // Create shortest path image:
      std::vector<TMask> pathImage(N);
      d3d::makePathImage(path, std::span{pathImage});

      writeImage<TMaskImage>(createImage(source, pathImage.data(), pathImage.size()), *params->pathImage);
    }
  }

  return EXIT_SUCCESS;
}
