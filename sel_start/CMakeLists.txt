cmake_minimum_required(VERSION 3.13)
project(Main)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(Main main.cpp)
# add_executable(Main filter.cpp)

target_link_libraries (Main ${PCL_LIBRARIES} ${OpenCV_LIBS})


