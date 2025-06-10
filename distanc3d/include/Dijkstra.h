#pragma once

#include "IPriorityQueue.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <cstdlib>
#include <functional>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#ifdef USE_STD_SPAN
#include <span>
using std::span;
#else
#include <tcb/span.hpp>
using tcb::span;
#endif

namespace d3d
{
auto ST = [] (int i) -> std::size_t {
  return static_cast<std::size_t>(i);
};

/// Priority queue entries for the Dijkstra algorithm are pairs, where:
/// -first element is voxel index
/// -second element is voxel distance (physical units)
template<typename TDistance>
using QueueItem = std::pair<int, TDistance>;

/// Index value used to indicate that a voxel is a source
constexpr int SOURCE_INDEX = 0;

/// Index value used to indicate that a voxel has no source
constexpr int NO_SOURCE = -1;

/**
 * @brief Convert 3D image voxel coordinates to a linear voxel index.
 * @param[in] coord 3D voxel coordinates
 * @param[in] imageDims Image dimensions
 * @return Linear voxel index
 */
int coordToIndex(const glm::ivec3& coord, const glm::ivec3& imageDims)
{
  return (coord.z * imageDims.y + coord.y) * imageDims.x + coord.x;
}

/**
 * @brief Convert a linear index in a 3D image to 3D voxel coordinates
 * @param[in] index Linear index
 * @param[in] imageDims Image dimensions
 * @return 3D voxel coordinates
 */
glm::ivec3 indexToCoord(int index, const glm::ivec3& imageDims)
{
  const std::div_t zQuotRem = std::div(index, imageDims.x * imageDims.y);
  const std::div_t xyQuotRem = std::div(zQuotRem.rem, imageDims.x);
  return glm::ivec3{xyQuotRem.rem, xyQuotRem.quot, zQuotRem.quot};
}

/**
 * @brief A neighbor in a 3D cubic neighborhood of any radius. Given a voxel u = (x, y, z),
 * this struct specifies a neighbor v = (x', y', z') of u within a full 3x3x3 neighborhood.
 */
struct Neighbor
{
  /// Direction vector of the neighbor (v) relative to u, which is
  /// v - u = (x' - x, y' - y, z' - z), where the components are in {-1, 0, 1}.
  glm::ivec3 dir;

  /// The signed linear offset of this neighbor relative to the cube center.
  /// The offset is a function of the image dimensions.
  int offset;

  /// Euclidean distance (in physical units) between u and this neighbor (v),
  /// accounting for voxel spacing.
  double distance;
};

std::ostream& operator<<(std::ostream& os, const std::vector<Neighbor>& hood)
{
  for (const Neighbor& n : hood) {
    os << "\n-Direction: " << n.dir.x << ", " << n.dir.y << ", " << n.dir.z
       << "\n-Offset: " << n.offset
       << "\n-Distance: " << n.distance << '\n';
  }
  return os;
}

/**
 * @brief Create a 3x3x3 neighborhood of 26 voxels for a 3D image.
 * @param[in] imageDims Image dimensions
 * @param[in] voxelSize Image voxel size in physical units
 * @return Neighborhood
 */
std::vector<Neighbor> makeNeighborhood(const glm::ivec3& imageDims, const glm::dvec3& voxelSize)
{
  std::vector<Neighbor> hood;

  for (int z = -1; z <= 1; ++z) {
    for (int y = -1; y <= 1; ++y) {
      for (int x = -1; x <= 1; ++x) {
        if (0 == x && 0 == y && 0 == z) {
          continue;
        }

        Neighbor n;
        n.dir = glm::ivec3{x, y, z};
        n.offset = coordToIndex(n.dir, imageDims);
        n.distance = glm::length(voxelSize * glm::dvec3{n.dir});
        hood.push_back(n);
      }
    }
  }

  return hood;
}

/**
 * @brief Validate that all Dijkstra algorithm inputs are non-null and have the same data size.
 *
 * @tparam TLabel Label type (must be integral)
 * @tparam TDist (must be floating-point)
 *
 * @param[in] imageDims Dimensions of all images
 * @param[in] sourceLabel Input source label image
 * @param[in] sourceEuclidDist Input source Euclidean distance image
 * @param[in] imageDist Output image-based distance image
 * @param[in] euclidDist Output Euclidean distance image
 * @param[in] sourceIndex Output source index image
 * @param[in] parentIndex Output parent index image
 * @param[out] os Output stream for logging
 *
 * @return True iff validation passes.
 */
template<typename TLabel, typename TDist>
bool validateData(
  const glm::ivec3& imageDims,
  const span<TLabel> sourceLabel,
  // const span<TDist> sourceEuclidDist,
  const span<TDist> imageDist,
  const span<TDist> euclidDist,
  const span<int> sourceIndex,
  const span<int> parentIndex,
  std::ostream& os)
{
  const auto N = static_cast<std::size_t>(imageDims.x * imageDims.y * imageDims.z);

  if (!sourceLabel.data()) {
    os << "Source label image is null\n";
    return false;
  }
  else if (N != sourceLabel.size()) {
    os << "Source label image size mismatch\n";
    return false;
  }

  // if (!sourceEuclidDist.data()) {
  //   os << "Source Euclidean distance image is null\n";
  //   return false;
  // }
  // else if (N != sourceEuclidDist.size()) {
  //   os << "Source Euclidean distance image size mismatch\n";
  //   return false;
  // }

  if (!imageDist.data()) {
    os << "Image-based distance image is null\n";
    return false;
  }
  else if (N != imageDist.size()) {
    os << "Image-based distance image size mismatch\n";
    return false;
  }

  if (!euclidDist.data()) {
    os << "Euclidean distance image is null\n";
    return false;
  }
  else if (N != euclidDist.size()) {
    os << "Euclidean distance image size match\n";
    return false;
  }

  if (!sourceIndex.data()) {
    os << "Source index image is null\n";
    return false;
  }
  else if (N != sourceIndex.size()) {
    os << "Source index image size mismatch\n";
    return false;
  }

  if (!parentIndex.data()) {
    os << "Parent index image is null\n";
    return false;
  }
  else if (N != parentIndex.size()) {
    os << "Parent index image size mismatch\n";
    return false;
  }

  return true;
}

/**
 * @brief Insert source voxels into a priority queue and initialize their distances.
 *
 * @tparam TLabel Source label component type
 * @tparam TDistance Type used to represent distances of entries in the priority queue and
 * also in distance images
 *
 * @param[in] sourceLabel Input source label
 * @param[in] sourceEuclidDist Input source Euclidean distance
 * @param[out] euclidDist Euclidean distance:
 * source voxel distance set to either \c sourceDistance (if it exists) or to zero otherwise;
 * all other distances set to \c std::numeric_limits<DistanceType>::max()
 * @param[out] imageDist Image distance
 * @param[out] sourceIndex Source index image: each source voxel i set to i (itself as source);
 * all other voxels set to -1
 * @param[out] parentIndex Parent index image: each voxel i set to i (itself as parent)
 * @param[out] pqueue Priority queue
 *
 * @return True iff successful initialization
 */
template<typename TLabel, typename TDist>
bool initializeData(
  const span<TLabel> sourceLabel,
  const span<TDist> sourceEuclidDist,
  span<TDist> euclidDist,
  span<TDist> imageDist,
  span<int> sourceIndex,
  span<int> parentIndex,
  IPriorityQueue<QueueItem<TDist>>& pqueue)
{
  static constexpr auto MAX_DIST = std::numeric_limits<TDist>::max();
  static constexpr auto ZERO_DIST = static_cast<TDist>(0.0);

  pqueue.clear();

  for (std::size_t i = 0; i < sourceLabel.size(); ++i)
  {
    const auto ii = static_cast<int>(i);

    if (sourceLabel[i] > 0) {
      // i is a source, so assign it zero distance and itself as its source:
      imageDist[i] = ZERO_DIST;
      sourceIndex[i] = ii;
      pqueue.push(std::make_pair(ii, ZERO_DIST));

      if (sourceEuclidDist.data()) {
        euclidDist[i] = sourceEuclidDist[i];
      }
    }
    else {
      // i is not a source, so assign it max distance and no source:
      imageDist[i] = MAX_DIST;
      euclidDist[i] = MAX_DIST;
      sourceIndex[i] = NO_SOURCE;
    }

    if (parentIndex.data()) {
      parentIndex[i] = ii; // i is its own parent
    }
  }

  return true;
}

/**
 * @brief Execute Dijkstra's shortest path algorithm on a 3D image to completion.
 * Graph nodes correspond to voxel centers of the 3D image and graph edges connect
 * neighboring voxels together.
 *
 * @tparam TDist Type used to represent distances of priority queue entries and
 * also the component type of voxels in the distance image.
 *
 * @param[in] imageDims Image dimensions (voxels)
 * @param[in] voxelSpacing Voxel spacing (mm)
 * @param[in] sourceEuclidDist Euclidean distance of source
 * @param[in, out] sourceIndex
 * Input: The value of each source (seed) voxel i is its own index i. All non-source voxels
 * have index -1 (NO_SOURCE).
 * Output: Each voxel is assigned the index of its nearest source, thereby creating a
 * segmentation of the image.
 * @param[in, out] parentIndex Parent index image
 * @param[in, out] imageDistance Distances based on image voxel values
 * @param[in, out] euclidDistance Distances based on Euclidean distance from sources
 * @param[in] imageWeight Weight of the image distance
 * @param[in] euclidWeight Weight of the Euclidean distance
 * @param[in, out] pqueue Priority queue
 * @param[in] computeImageDistance Function to compute distances between two voxels of an image
 * (optional: ignored if set to nullptr)
 * @param[in] isValidIndex Function to determine whether v is a valid voxel node in the graph
 * @param[in] stopEuclidDist Stopping Euclidean distance: the algorithm stops when the Euclidean distance
 * of the minimum voxel in the priority queue exceeds this value
 * @param[in] debugPrintInterval Number of iterations between successive debug print statements
 * (optional: ignored if set to zero)
 *
 * @return Number of priority queue updates
 */
template<typename TDist>
std::size_t runDijkstra(
  const glm::ivec3& imageDims,
  const glm::dvec3& voxelSpacing,
  const span<TDist> sourceEuclidDist,
  span<int> sourceIndex,
  span<int> parentIndex,
  span<TDist> imageDist,
  span<TDist> euclidDist,
  TDist imageWeight,
  TDist euclidWeight,
  IPriorityQueue<QueueItem<TDist>>& pqueue,
  std::function<TDist(int u, int v)> computeImageDist,
  std::function<bool(int index)> isValidIndex,
  const std::optional<TDist>& stopEuclidDist,
  unsigned int debugPrintInterval,
  std::ostream& os)
{
  static constexpr auto MAX_DIST = std::numeric_limits<TDist>::max();
  const glm::ivec3 zeroIndex{0, 0, 0};
  const auto neighborhood = makeNeighborhood(imageDims, voxelSpacing);

  std::size_t numIter = 0;
  std::size_t numQueueUpdates = 0;

  if (debugPrintInterval) {
    os << "Iteration, queue size, queue updates" << std::endl;
  }

  while (!pqueue.empty())
  {
    if (debugPrintInterval && (numIter % debugPrintInterval == 0)) {
      os << numIter << ", " << pqueue.size() << ", " << numQueueUpdates << std::endl;
    }

    ++numIter;

    // Pop off u, the index of the minimum distance voxel in the queue
    const int u = pqueue.top().first;
    pqueue.pop();

    const glm::ivec3 uCoord = indexToCoord(u, imageDims);
    const std::size_t uu = ST(u);

    if (stopEuclidDist && *stopEuclidDist < euclidDist[uu]) {
      os << "Stopping Euclidean distance " << *stopEuclidDist << " reached" << std::endl;
      break; // Minimum distance exceeds stopping distance
    }

    // Index, voxel coordinate, physical position, and Euclidean distance of u's source
    const int uSrcIndex = sourceIndex[uu];
    const glm::ivec3 uSrcCoord = indexToCoord(uSrcIndex, imageDims);
    const glm::dvec3 uSrcPos = voxelSpacing * glm::dvec3{uSrcCoord};

    const TDist uSrcEuclidDist = static_cast<TDist>(sourceEuclidDist.data() ? sourceEuclidDist[ST(uSrcIndex)] : 0);

    // Loop over all neighbors v of u
    for (const Neighbor& neighbor : neighborhood)
    {
      const glm::ivec3 vCoord = uCoord + neighbor.dir;

      if (glm::any(glm::lessThan(vCoord, zeroIndex)) ||
          glm::any(glm::greaterThanEqual(vCoord, imageDims))) {
        continue; // Do not update v, since it is outside the image bounds
      }

      const glm::dvec3 vPos = voxelSpacing * glm::dvec3{vCoord};
      const int v = u + neighbor.offset; // voxel index of v
      const std::size_t vv = ST(v);

      if (isValidIndex && !isValidIndex(v)) {
        continue; // Do not update v, since it is outside the mask
      }

      if (SOURCE_INDEX == sourceIndex[vv]) {
        continue; // Do not update v, since it is a source
      }

      TDist vTotalDistNew = MAX_DIST; // Total (image + Euclidean) distance of v through u
      bool pushV = false; // Should v be pushed onto the queue?

      // New Euclidean distance of v equals (source distance) + (distance from source to v)
      const TDist vEuclidDistNew = uSrcEuclidDist + static_cast<TDist>(glm::distance(uSrcPos, vPos));

      if (computeImageDist && imageDist.data())
      {
        // Use both image and Euclidean distances:
        const TDist vTotalDistOld = imageWeight * imageDist[vv] + euclidWeight * euclidDist[vv];
        const TDist vImageDistNew = imageDist[uu] + computeImageDist(u, v);

        vTotalDistNew = imageWeight * vImageDistNew + euclidWeight * vEuclidDistNew;

        if (vTotalDistNew < vTotalDistOld) {
          pushV = true;
          sourceIndex[vv] = uSrcIndex;
          euclidDist[vv] = vEuclidDistNew;
          imageDist[vv] = vImageDistNew;

          if (parentIndex.data()) {
            parentIndex[vv] = u;
          }
        }
      }
      else
      {
        // Use only Euclidean distance:
        vTotalDistNew = vEuclidDistNew;

        if (vTotalDistNew < euclidDist[vv]) {
          pushV = true;
          sourceIndex[vv] = uSrcIndex;
          euclidDist[vv] = vEuclidDistNew;

          if (parentIndex.data()) {
            parentIndex[vv] = u;
          }
        }
      }

      if (pushV)
      {
        const bool enqueued = (euclidDist[vv] != std::numeric_limits<TDist>::max());

        if (pqueue.mutableKeys() && !enqueued) {
          pqueue.decreaseKey(std::make_pair(vv, vTotalDistNew));
        }
        else {
          pqueue.push(std::make_pair(vv, vTotalDistNew));
        }

        ++numQueueUpdates;
      }
    }
  }

  return numQueueUpdates;
}

/**
 * @brief Create a segmentation label image from the source and label images
 * @param[in] sourceIndexImage Source index image
 * @param[in] sourceLabelImage Source label iamge
 * @param[out] segLabelImage Segmentation label iamge
 * @return True iff the segmentation label image was created successfully
 */
template<typename TLabel>
bool makeSegmentationImage(
  const span<int> sourceIndex, const span<TLabel> sourceLabel, span<TLabel> segLabel)
{
  if (!sourceIndex.data() || !sourceLabel.data() || !segLabel.data()) {
    return false;
  }

  if (sourceIndex.size() != sourceLabel.size() || sourceIndex.size() != segLabel.size()) {
    return false;
  }

  for (std::size_t i = 0; i < sourceIndex.size(); ++i) {
    segLabel[i] = sourceLabel[ST(sourceIndex[i])];
  }

  return true;
}

/**
 * @brief Create the shortest path from a voxel to its closest source voxel
 *
 * @param[in] parentIndexImage Parent index image (computed from \c dijkstra)
 * @param[in] destIndex Index of destination voxel of path
 *
 * @return Vector of voxel indices along the path, beginning at the destination voxel
 * and ending at the closest source voxel.
 */
std::vector<int> makeShortestPath(const span<int> parentIndexImage, int destIndex)
{
  std::vector<int> path;

  if (!parentIndexImage.data()) {
    return path;
  }

  int i = destIndex;
  path.push_back(i);

  while (i != parentIndexImage[ST(i)])
  {
    i = parentIndexImage[ST(i)];
    path.push_back(i);
  }

  return path;
}

/**
 * @brief Create a path iamge
 *
 * @tparam TLabel Seed image voxel component type
 *
 * @param[in] path Path of voxel indices
 * @param[out] pathImage Binary path image, with path voxels set to 1
 */
template<typename TMask>
void makePathImage(const std::vector<int>& path, span<TMask> pathImage)
{
  static constexpr TMask bg = 0;
  static constexpr TMask fg = 1;

  if (!pathImage.data()) {
    return;
  }

  for (TMask& l : pathImage) {
    l = bg;
  }

  for (int j : path) {
    pathImage[ST(j)] = fg;
  }
}

void printShortestPath(std::ostream& os, const std::vector<int>& path, const glm::ivec3& imageDims)
{
  std::size_t step = 0;

  // Loop over indices of the path from end to start
  os << "Step, voxel coord i, j, k\n";

  for (auto index = path.rbegin(); index != path.rend(); ++index)
  {
    const glm::ivec3 coord = indexToCoord(*index, imageDims);
    os << step++ << ", " << coord.x << ", " << coord.y << ", " << coord.z << '\n';
  }
}
} // namespace d3d
