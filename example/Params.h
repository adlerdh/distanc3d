#pragma once

#include <glm/glm.hpp>

#include <filesystem>
#include <ostream>
#include <optional>

/// @brief How to compute the edge weight between two voxels?
enum class ImageEdgeWeightType
{
  AbsoluteDiff, //!< absolute difference
  SquaredDiff, //!< squared difference
  Source, //!< source value
  Destination, //!< destination value
  Minimum, //!< minimum value
  Mean, //!< mean value
  Maximum //!< maximum value
};

struct Params
{
  using path = std::filesystem::path;

  path source; //!< source label image (input, required)
  std::optional<path> sourceEuclidDist; //!< source Elucidean distances (input)
  std::optional<path> image; //!< image used to derive graph weights (input)
  std::optional<path> mask; //!< mask image within which distances are computed (input)
  std::optional<path> seg; //!< segmented image (output)
  std::optional<path> totalDist; //!< total distance image (output)
  std::optional<path> euclidDist; //!< Euclidean distance image (output)
  std::optional<path> imageDist; //!< image-based distance image (output)
  std::optional<glm::ivec3> destVoxel; //!< destination voxel index for path (output)
  std::optional<path> pathImage; //!< path image (output)

  // Number of intervals between debug print statements (use 0 to disable debug prints)
  unsigned int debugInterval = 10000;

  ImageEdgeWeightType imageEdgeWeightType{ImageEdgeWeightType::AbsoluteDiff};

  // Coefficients that weight the relative contributions of image distance and Euclidean distance
  // when computing the edge distance between two adjacent voxels (nodes in the graph):
  // edge weight = imageWeight * imageDistance + euclidWeight * euclideanDist
  double imageWeight{1.0}; //!< image-based distance weight
  double euclideanWeight{1.0}; //!< Euclidean distance weight

  std::optional<double> stoppingEuclideanDistance; //!< distance at which to stop computations
};

std::ostream& operator<<(std::ostream&, const Params&);

std::optional<Params> parse(int argc, char* argv[]);
