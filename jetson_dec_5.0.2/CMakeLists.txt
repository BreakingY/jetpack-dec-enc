cmake_minimum_required(VERSION 3.0)
project(JETSONTEST)
find_package(CUDA REQUIRED)
enable_language(CUDA)

include_directories(${CUDA_INCLUDES} ./ ./include ./include/libjpeg-8b  /usr/include/libdrm ./common/algorithm/cuda /usr/local/cuda-11.4/targets/aarch64-linux/include/)

aux_source_directory(./ TEST)
aux_source_directory(./test DEMO)
aux_source_directory(./common/classes COMMON)
aux_source_directory(./common/algorithm/cuda CUDA)
add_compile_options(-g -std=c++17)#-fpermissive
link_directories(/usr/lib/aarch64-linux-gnu/tegra /lib/aarch64-linux-gnu /usr/local/lib /usr/lib /usr/local/cuda/lib64)
set(EXECUTABLE_OUTPUT_PATH ./)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./)
set(CMAKE_BUILD_TYPE Release)
add_library(JetsonDec SHARED ${TEST} ${COMMON} ${CUDA})
target_link_libraries(JetsonDec ${CUDA_LIBRARIES} pthread v4l2 EGL GLESv2 X11 nvbufsurface nvbufsurftransform nvjpeg nvosd drm cuda cudart)
set(CMAKE_BUILD_TYPE Debug)
add_executable(demo ${DEMO})
target_link_libraries(demo JetsonDec avutil avformat avcodec)

