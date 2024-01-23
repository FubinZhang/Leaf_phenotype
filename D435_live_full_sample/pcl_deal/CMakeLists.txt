cmake_minimum_required(VERSION 3.0)

project(beanbox)

find_package(PCL 1.8 REQUIRED)


find_package(OpenCV REQUIRED)


add_executable(beanbox main.cpp BeanBox2.cpp)

target_include_directories(beanbox PRIVATE 
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(beanbox PRIVATE
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS} 
)
