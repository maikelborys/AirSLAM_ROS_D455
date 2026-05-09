cmake_minimum_required(VERSION 3.14)
project(tensorrtbuffer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# AirSLAM only consumes BufferManager (buffers.h, header-only) and logger.
# sample_options.cpp / sample_utils.cpp pull in NvCaffeParser.h / NvUffParser.h
# which were removed in TensorRT 10. We don't need them — keep just logger.cpp.

find_package(CUDAToolkit REQUIRED)

add_library(tensorrtbuffer SHARED src/logger.cpp)

target_include_directories(tensorrtbuffer
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(tensorrtbuffer
    PUBLIC
        CUDA::cudart
)

# Compile-only smoke test for the TRT-10-migrated buffers.h header.
# Builds during `colcon build`; if buffers.h ever stops parsing this fails.
add_executable(buffers_compile_test test/buffers_compile_test.cpp)
target_link_libraries(buffers_compile_test
    PRIVATE
        tensorrtbuffer
        nvinfer
)
