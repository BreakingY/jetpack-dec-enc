cmake_minimum_required(VERSION 3.0)
project(JETSONTEST)
find_package(CUDA REQUIRED)
enable_language(CUDA)

include_directories(${CUDA_INCLUDES} ./ ./include ./include/libjpeg-8b  /usr/include/libdrm ./common/algorithm/cuda /usr/local/cuda-11.4/targets/aarch64-linux/include/
                    media/MediaCommon media/MediaReader media/MediaReader/FileReader 
                    media/MediaReader/RtspReader  media/MediaReader/RtspReader/rtp  media/MediaReader/RtspReader/3rdparty
                    media/Bitstream/h264/include media/Bitstream/h265/include)

aux_source_directory(./ TEST)
aux_source_directory(./test DEMO)
aux_source_directory(./common/classes COMMON)
aux_source_directory(./common/algorithm/cuda CUDA)

aux_source_directory(./media/MediaCommon MEDIACOMMON)
aux_source_directory(./media/MediaReader/FileReader MEDIAREADER)
aux_source_directory(./media/MediaReader/RtspReader MEDIARTSP)
aux_source_directory(./media/MediaReader/RtspReader/rtp MEDIARTP)
aux_source_directory(./media/MediaReader/RtspReader/3rdparty MEDIA3RD)
aux_source_directory(./media/Bitstream/h264/source H264)
aux_source_directory(./media/Bitstream/h265/source H265)

# 为非 CUDA 代码添加 -fpermissive 选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

# 添加编译选项
add_compile_options(-g -std=c++14)

link_directories(/usr/lib/aarch64-linux-gnu/tegra /lib/aarch64-linux-gnu /usr/local/lib /usr/lib /usr/local/cuda/lib64)
set(EXECUTABLE_OUTPUT_PATH ./)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./)
set(CMAKE_BUILD_TYPE Release)
add_library(JetsonDec SHARED ${TEST} ${COMMON} ${CUDA} ${MEDIACOMMON} ${MEDIAREADER} ${MEDIARTSP} ${MEDIARTP} ${MEDIA3RD} ${H264} ${H265})
target_link_libraries(JetsonDec ${CUDA_LIBRARIES} pthread v4l2 EGL GLESv2 X11 nvbufsurface nvbufsurftransform nvjpeg nvosd drm cuda cudart)
set(CMAKE_BUILD_TYPE Debug)
add_executable(demo ${DEMO})
target_link_libraries(demo JetsonDec avutil avformat avcodec)

