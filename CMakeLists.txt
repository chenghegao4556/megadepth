cmake_minimum_required(VERSION 3.10)
project(megadepth)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(Torch_DIR /home/chenghe/3rdparty/pytorch/torch/share/cmake/Torch)

find_package( OpenCV 3.4.1)
find_package(Torch REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${TORCH_INCLUDE_DIRS}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(
        ${PROJECT_NAME} SHARED
        src/megadepth_estimator.cpp
        src/camera.cpp
        src/megadepth_client.cpp
)

target_link_libraries(
        ${PROJECT_NAME}
        ${TORCH_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

add_executable(megadepthclient client.cpp)
target_link_libraries(megadepthclient ${PROJECT_NAME})