#include "Util.h"

#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkSignedMaurerDistanceMapImageFilter.h>

#include <chrono>
#include <iostream>

int main(int argc, char* argv[])
{
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
    std::cerr << argv[0] << " <seed_in> <distance_out> \n"
              << "- seed_in: seed image (input)\n"
              << "- distance_out: distance image (output)\n"
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

  using DistanceFilterType
    = itk::SignedMaurerDistanceMapImageFilter<LabelImageType, DistanceImageType>;

  time_point<steady_clock> tick = steady_clock::now();

  DistanceFilterType::Pointer distanceFilter = DistanceFilterType::New();
  distanceFilter->SetInput(seed);
  distanceFilter->Update();
  DistanceImageType::Pointer distanceImage = distanceFilter->GetOutput();

  time_point<steady_clock> tock = steady_clock::now();

  std::cout << "\nTime (ms): " << duration_cast<milliseconds>(tock - tick).count() << std::endl;

  writeImage<DistanceImageType>(distanceImage, argv[2]);

  return EXIT_SUCCESS;
}
