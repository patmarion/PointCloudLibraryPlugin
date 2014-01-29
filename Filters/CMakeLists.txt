find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


set(sources
  vtkAnnotateOBBs.cxx
  vtkPCDReader.cxx
  vtkPCLConversions.cxx
  vtkPCLEuclideanClusterExtraction.cxx
  vtkPCLNormalEstimation.cxx
  vtkPCLOpenNISource.cxx
  vtkPCLRadiusOutlierRemoval.cxx
  vtkPCLSACSegmentationCylinder.cxx
  vtkPCLSACSegmentationPlane.cxx
  vtkPCLVoxelGrid.cxx
  )


set(deps
  ${PCL_LIBRARIES}
  vtkFiltering vtkIO
  )


set(library_name vtkPCLFilters)

add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${deps})


if(VTK_WRAP_PYTHON)
  include_directories(${VTK_PYTHON_INCLUDE_DIR})
  include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
  wrap_python(${library_name} "${sources}")
endif()
