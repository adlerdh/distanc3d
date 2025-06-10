#pragma once

#include <itkImage.h>
#include <itkImageFileWriter.h>
#include <itkImportImageFilter.h>

#include <filesystem>
#include <iostream>

/**
 * @brief Create a 3D ITK image from raw image data
 *
 * @tparam T Image component type
 * @param[in] ref Reference image
 * @param[in] data Raw image data
 * @param[in] size Number of image voxels
 *
 * @return ITK image aligned with refImage with voxel values from imageData
 */
template<typename T>
typename itk::Image<T, 3>::Pointer createImage(
    const typename itk::ImageBase<3>::Pointer ref, const T* data, std::size_t size)
{
  using ImportFilterType = itk::ImportImageFilter<T, 3>;

  // This filter will not free the memory in its destructor and the application providing the
  // buffer retains the responsibility of freeing the memory for this image data
  constexpr bool filterOwnsBuffer = false;

  if (!ref) {
    std::cerr << "Null data array provided when creating new scalar image!" << std::endl;
    return nullptr;
  }

  if (!data) {
    return nullptr;
  }

  if (size != ref->GetLargestPossibleRegion().GetNumberOfPixels()) {
    return nullptr;
  }

  typename ImportFilterType::Pointer importer = ImportFilterType::New();
  importer->SetRegion(ref->GetLargestPossibleRegion());
  importer->SetOrigin(ref->GetOrigin());
  importer->SetSpacing(ref->GetSpacing());
  importer->SetDirection(ref->GetDirection());
  importer->SetImportPointer(const_cast<T*>(data), size, filterOwnsBuffer);

  try {
    importer->Update();
    return importer->GetOutput();
  }
  catch (const itk::ExceptionObject& e) {
    std::cerr << "Exception creating scalar image from data array: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception creating scalar image from data array: " << e.what() << std::endl;
  }
  return nullptr;
}

/**
 * @brief Write an ITK image to disk
 * @param[in] image ITK image
 * @param[in] outFilename File name
 */
template<class ImageType>
void writeImage(const typename ImageType::Pointer image, const std::filesystem::path& filename)
{
  using WriterType = itk::ImageFileWriter<ImageType>;
  auto writer = WriterType::New();
  writer->SetFileName(filename);
  writer->SetInput(image);

  try {
    writer->Update();
  }
  catch (const itk::ExceptionObject& e) {
    std::cerr << "Exception writing image: " << e.what() << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception writing image: " << e.what() << std::endl;
  }
}
