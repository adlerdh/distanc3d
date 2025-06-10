#include "Util.h"

#include <fast_marching_method/fast_marching_method.hpp>

#include <itkImage.h>
#include <itkImageFileReader.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>

int main(int argc, char* argv[])
{
  namespace fmm = thinks::fast_marching_method;

  using std::chrono::duration_cast;
  using std::chrono::milliseconds;
  using std::chrono::steady_clock;
  using std::chrono::time_point;

  constexpr unsigned int Dim = 3;
  using LabelType = unsigned short;
  using DistanceType = float;

  using LabelImageType = itk::Image<LabelType, Dim>;
  using DistanceImageType = itk::Image<DistanceType, Dim>;

  if (argc != 3)
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " <seed_in> <distance_out>\n"
              << "- seed_in: seed image (input)\n"
              << "- distance_out: FMM distance image (output)\n"
              << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Seed image (in): " << argv[1] << "\nDistance image (out): " << argv[2] << std::endl;

  LabelImageType::Pointer seed = itk::ReadImage<LabelImageType>(argv[1]);

  if (!seed)
  {
    std::cerr << "Null input seed image!" << std::endl;
    return EXIT_FAILURE;
  }

  const LabelImageType::SizeType size = seed->GetLargestPossibleRegion().GetSize();
  const LabelImageType::SpacingType spacing = seed->GetSpacing();

  const std::array<std::size_t, 3> gridSize = {size[0], size[1], size[2]};
  const std::array<DistanceType, 3> gridSpacing = {
    static_cast<DistanceType>(spacing[0]),
    static_cast<DistanceType>(spacing[1]),
    static_cast<DistanceType>(spacing[2])
  };
  const DistanceType uniformSpeed = 1.0f;

  const std::vector<std::array<int32_t, 3>> seedIndices = {{0, 0, 0}};
  const std::vector<DistanceType> seedDistances = {0.0f};

  // Test 1: UniformSpeedEikonalSolver
  time_point<steady_clock> tick = steady_clock::now();
  auto solver = fmm::UniformSpeedEikonalSolver<DistanceType, 3>(gridSpacing, uniformSpeed);
  std::vector<DistanceType> arrivalTimes
    = fmm::SignedArrivalTime(gridSize, seedIndices, seedDistances, solver);
  time_point<steady_clock> tock = steady_clock::now();

  std::cout << "\nFMM (UniformSpeedEikonalSolver) execution time (ms): "
            << duration_cast<milliseconds>(tock - tick).count() << std::endl;

  // Test 2: HighAccuracyUniformSpeedEikonalSolver
  tick = steady_clock::now();
  auto accurateSolver
    = fmm::HighAccuracyUniformSpeedEikonalSolver<DistanceType, 3>(gridSpacing, uniformSpeed);
  std::vector<DistanceType> accurateArrivalTimes
    = fmm::SignedArrivalTime(gridSize, seedIndices, seedDistances, accurateSolver);
  tock = steady_clock::now();

  std::cout << "\nFMM (HighAccuracyUniformSpeedEikonalSolver) execution time (ms): "
            << duration_cast<milliseconds>(tock - tick).count() << std::endl;

  const DistanceImageType::Pointer accurateDistanceImage
    = createImage(seed, accurateArrivalTimes.data(), accurateArrivalTimes.size());

  writeImage<DistanceImageType>(accurateDistanceImage, argv[2]);

  return EXIT_SUCCESS;
}
