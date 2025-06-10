#pragma once

#include <glm/glm.hpp>

#include <filesystem>
#include <ostream>
#include <optional>

/*
image (optional) input image //
imageEdge (absDiff, squareDiff, inverseSpeed, optional) //
euclideanWeight (1.0, optional) //
stopDist (optional) stopping total distance //
seed (required) source label image //
seedDist (optional) source Euclidean distance image //
mask (in, optional) mask image //
seg (out, optional) segmentation label image //
totalDist (out, optional) total distance image //
euclidDist (out, optional) total euclidean distance image //
imageDist (out, optional) total image distance image //
dest (in, optional) destination voxel //
path (out, optional) path image //
debugPrintInterval (10000, optional)
*/

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

  path seed;
  std::optional<path> seedEuclidDist;
  std::optional<path> image;
  std::optional<path> mask;
  std::optional<path> seg;
  std::optional<path> totalDist;
  std::optional<path> euclidDist;
  std::optional<path> imageDist;
  std::optional<glm::ivec3> destVoxel;
  std::optional<path> pathImage;

  // Number of intervals between debug print statements (use 0 to disable debug prints)
  unsigned int debugInterval = 10000;

  ImageEdgeWeightType imageEdgeWeightType = ImageEdgeWeightType::AbsoluteDiff;

  // Coefficients that weight the relative contributions of image distance and Euclidean distance
  // when computing the edge distance between two adjacent voxels (nodes in the graph):
  // edge weight = imageWeight * imageDistance + euclidWeight * euclideanDist
  double imageWeight = 1.0;
  double euclideanWeight = 1.0;

  std::optional<double> stoppingEuclideanDistance;
};

std::ostream& operator<<(std::ostream&, const Params&);

std::optional<Params> parse(int argc, char* argv[]);
