# 设置最低的CMake版本
cmake_minimum_required(VERSION 3.22.1)

# 项目名称
project(scan2)

# if you couldn't find opencv, you need set opencv includepath
# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 设置编译选项可以使用gdb调试
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 添加头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(scan2 scan2.h scan2.cpp Source.cpp)

# 链接OpenCV库
target_link_libraries(scan2 ${OpenCV_LIBS})
