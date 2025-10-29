#include "Params.h"

#include <argparse/argparse.hpp>

#include <iostream>
#include <unordered_map>

std::string to_string(ImageEdgeWeightType type)
{
  switch (type) {
  case ImageEdgeWeightType::AbsoluteDiff: return "absolute difference";
  case ImageEdgeWeightType::SquaredDiff: return "squared difference";
  case ImageEdgeWeightType::Source: return "source value";
  case ImageEdgeWeightType::Destination: return "destination (neighbor) value";
  case ImageEdgeWeightType::Minimum: return "min value";
  case ImageEdgeWeightType::Mean: return "mean value";
  case ImageEdgeWeightType::Maximum: return "max value";
  }
  return "unknown";
}

ImageEdgeWeightType parseEdgeWeightType(const std::string& s)
{
  static const std::unordered_map<std::string, ImageEdgeWeightType> map = {
    {"AbsDiff", ImageEdgeWeightType::AbsoluteDiff},
    {"SqDiff", ImageEdgeWeightType::SquaredDiff},
    {"Src", ImageEdgeWeightType::Source},
    {"Dst", ImageEdgeWeightType::Destination},
    {"Min", ImageEdgeWeightType::Minimum},
    {"Mean", ImageEdgeWeightType::Mean},
    {"Max", ImageEdgeWeightType::Maximum},
  };

  const auto it = map.find(s);
  if (map.end() != it) {
    return it->second;
  }

  throw std::runtime_error("Invalid ImageEdgeWeightType: " + s);
}

std::ostream& operator<<(std::ostream& os, const Params& p)
{
  auto print_opt = [&os](const auto& label, const auto& val) {
    if (val) {
      os << "  " << label << ": " << *val << "\n";
    }
  };

  os << "Parameters:\n";
  os << "  source: " << p.source << "\n";
  print_opt("image", p.image);
  print_opt("sourceEuclidDist", p.sourceEuclidDist);
  print_opt("mask", p.mask);
  print_opt("seg", p.seg);
  print_opt("totalDist", p.totalDist);
  print_opt("euclidDist", p.euclidDist);
  print_opt("imageDist", p.imageDist);

  if (p.destVoxel) {
    os << "  destVoxel: (" << p.destVoxel->x << ", " << p.destVoxel->y << ", " << p.destVoxel->z << ")\n";
  }

  print_opt("pathImage", p.pathImage);

  os << "  debugInterval: " << p.debugInterval << "\n";
  os << "  imageEdgeWeightType: " << to_string(p.imageEdgeWeightType) << "\n";
  os << "  imageWeight: " << p.imageWeight << "\n";
  os << "  euclideanWeight: " << p.euclideanWeight << "\n";

  print_opt("stoppingEuclideanDistance", p.stoppingEuclideanDistance);

  return os;
}

std::optional<Params> parse(int argc, char* argv[])
{
  argparse::ArgumentParser program("Dijkstra", "1.0.0");
  program.add_description("Compute shortest paths on images using Dijkstra's algorithm");

  program.add_argument("--source").help("Source image").required();
  program.add_argument("--image").help("Input image used for image-based edge weights");
  program.add_argument("--source-dist").help("Input source Euclidean distance image");
  program.add_argument("--mask").help("Input mask image of valid voxels");
  program.add_argument("--seg").help("Output segmentation image");
  program.add_argument("--total-dist").help("Output total distance image");
  program.add_argument("--euclid-dist").help("Output Euclidean distance image");
  program.add_argument("--image-dist").help("Output image-based distance image");
  program.add_argument("--dest-voxel").nargs(3).scan<'i', int>().help("Destination voxel for path (i j k)");
  program.add_argument("--path-image").help("Output shortest path image");
  program.add_argument("--debug-interval").scan<'u', unsigned int>().default_value(10000u).nargs(1).help("Debug print interval");
  program.add_argument("--image-weight-type").default_value(std::string("AbsDiff")).nargs(1)
      .help("Image edge weight type (AbsDiff, SqDiff, Src, Dst, Min, Mean, Max)");
  program.add_argument("--image-weight").scan<'g', double>().default_value(1.0).nargs(1).help("Image-based distance weight");
  program.add_argument("--euclid-weight").scan<'g', double>().default_value(1.0).nargs(1).help("Euclidean distance weight");
  program.add_argument("--stop-euclid-dist").scan<'g', double>().help("Stopping Euclidean distance");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Argument parsing error: " << e.what() << "\n" << program << std::endl;
    return std::nullopt;
  }

  Params params;
  params.source = program.get<std::string>("source");

  if (program.present("--image")) params.image = program.get<std::string>("--image");
  if (program.present("--source-dist")) params.sourceEuclidDist = program.get<std::string>("--source-dist");
  if (program.present("--mask")) params.mask = program.get<std::string>("--mask");
  if (program.present("--seg")) params.seg = program.get<std::string>("--seg");
  if (program.present("--total-dist")) params.totalDist = program.get<std::string>("--total-dist");
  if (program.present("--euclid-dist")) params.euclidDist = program.get<std::string>("--euclid-dist");
  if (program.present("--image-dist")) params.imageDist = program.get<std::string>("--image-dist");

  if (program.is_used("--dest-voxel")) {
    const auto vec = program.get<std::vector<int>>("--dest-voxel");
    if (vec.size() != 3) {
      throw std::runtime_error("Expected exactly 3 integers for --dest-voxel");
    }

    params.destVoxel = glm::ivec3{vec[0], vec[1], vec[2]};
  }

  if (program.present("--path-image")) {
    params.pathImage = program.get<std::string>("--path-image");
  }

  if (program.is_used("--stop-euclid-dist")) {
    params.stoppingEuclideanDistance = program.get<double>("--stop-euclid-dist");
  }

  params.debugInterval = program.get<unsigned int>("--debug-interval");
  params.imageWeight = program.get<double>("--image-weight");
  params.euclideanWeight = program.get<double>("--euclid-weight");
  params.imageEdgeWeightType = parseEdgeWeightType(program.get<std::string>("--image-weight-type"));

  return params;
}
