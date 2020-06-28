cmake_minimum_required(VERSION 3.10)
project(triangulation)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(triangulation main.cpp Frame.cpp Frame.h Marker.cpp Marker.h Camera.cpp Camera.h TriangulationSystem.cpp TriangulationSystem.h)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(triangulation ${OpenCV_LIBS})