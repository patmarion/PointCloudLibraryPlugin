find_package(PCL ${PCL_REQUIRED_VERSION} REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


set(sources
  vtkAnnotateOBBs.cxx
  vtkPCDReader.cxx
  vtkPCLConversions.cxx
  vtkPCLEuclideanClusterExtraction.cxx
  vtkPCLNormalEstimation.cxx
  vtkPCLRadiusOutlierRemoval.cxx
  vtkPCLSACSegmentationCircle.cxx
  vtkPCLSACSegmentationCylinder.cxx
  vtkPCLSACSegmentationLine.cxx
  vtkPCLSACSegmentationPlane.cxx
  vtkPCLVoxelGrid.cxx
  )

set(headers
  vtkAnnotateOBBs.h
  vtkPCDReader.h
  vtkPCLConversions.h
  vtkPCLEuclideanClusterExtraction.h
  vtkPCLNormalEstimation.h
  vtkPCLRadiusOutlierRemoval.h
  vtkPCLSACSegmentationCircle.h
  vtkPCLSACSegmentationCylinder.h
  vtkPCLSACSegmentationLine.h
  vtkPCLSACSegmentationPlane.h
  vtkPCLVoxelGrid.h
  )

if(USE_OPENNI)
  list(APPEND sources vtkPCLOpenNISource.cxx)
  list(APPEND headers vtkPCLOpenNISource.h)
endif()

set(deps
  ${PCL_LIBRARIES}
  vtkFiltering vtkIO
  )


set(library_name vtkPCLFilters)

add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${deps})
set_target_properties(${library_name}  PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")

install(TARGETS ${library_name}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

install(FILES
  ${headers}
  DESTINATION include/vtkPCLFilters)

if(VTK_WRAP_PYTHON)
  include_directories(${VTK_PYTHON_INCLUDE_DIR})
  include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
  wrap_python(${library_name} "${sources}")
endif()
