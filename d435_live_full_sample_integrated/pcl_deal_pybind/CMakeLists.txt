cmake_minimum_required(VERSION 3.22.1)
project(pybeanbox)

# Find packages
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pybind11 REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIR})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add executable
pybind11_add_module(pybeanbox BeanBox2.h BeanBox2.cpp)

# Link libraries
target_link_libraries(pybeanbox  PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

