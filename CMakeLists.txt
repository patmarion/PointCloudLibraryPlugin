project(PCLPlugin)
cmake_minimum_required(VERSION 2.8)

option(BUILD_TESTING "Build with tests enabled." ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})


add_subdirectory(Filters)


if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()
