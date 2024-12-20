cmake_minimum_required(VERSION 3.5.0)
project(AutoSentry VERSION 0.1.0 LANGUAGES C CXX)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(./include)
file(GLOB SOURCES src/*.cpp)
add_executable(AutoSentry ${SOURCES})


target_link_libraries(AutoSentry ${PCL_LIBRARIES} ${OpenCV_LIBS} livox_lidar_sdk_static)
