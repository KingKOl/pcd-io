cmake_minimum_required(VERSION 3.0.0)
project(PointCloudIO VERSION 0.1.0)

include(CTest)
enable_testing()

file(GLOB srcs *.cpp *.hpp)

include_directories("/usr/include/eigen3")

add_executable(${PROJECT_NAME} ${srcs})
# add_library(${PROJECT_NAME} SHARED ${srcs})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
