cmake_minimum_required(VERSION 2.6.0)

project(ElasticFusion)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

if(WIN32)
  macro(CANONIFY_BOOL var)
    if(${var})
      set(${var} TRUE)
    else()
      set(${var} FALSE)
    endif()
  endmacro()
endif()

if(WIN32)
  find_package(JPEG REQUIRED)
endif()

find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(ZLIB REQUIRED)

# HACK 为了选择使用支持OpenNI2的Pangolin
add_definitions(-DUSE_OPENNI_PANGOLIN)
set(Pangolin_DIR "/home/guoqing/libraries/Pangolin_ElasticFusion/lib/cmake/Pangolin")
find_package(Pangolin 0.1 REQUIRED)
message(STATUS "==> Pangolin_INCLUDE_DIRS = " ${Pangolin_INCLUDE_DIRS})


find_package(CUDA REQUIRED)
# 下面直接手动包含了
# find_package(OpenNI2 REQUIRED)

find_package(efusion REQUIRED)
find_package(SuiteSparse REQUIRED)

if(WIN32)
  find_package(RealSense QUIET)
  CANONIFY_BOOL(REALSENSE_FOUND)
  message(STATUS "librealsense found: ${REALSENSE_FOUND}")
  option(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})
endif()

if(WIN32)
  include_directories(${JPEG_INCLUDE_DIR})
endif()

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EFUSION_INCLUDE_DIR})
# include_directories(${OPENNI2_INCLUDE_DIR})
include_directories("/usr/include/openni2")


# HACK 
set(WITH_REALSENSE false)

if(WITH_REALSENSE)
# HACK
  find_package(realsense2 REQUIRED)
  if (realsense2_FOUND)
      set("realsense_INCLUDE_DIR"   "/usr/include/librealsense2")
      include_directories("${realsense_INCLUDE_DIR}")
      message(STATUS "==> ElasticFusion Found realsense2 " ${realsense_INCLUDE_DIR})    
  endif (realsense2_FOUND)


  # include_directories(${REALSENSE_INCLUDE_DIR})
  # add_definitions(-DWITH_REALSENSE)
  # set(EXTRA_LIBS ${EXTRA_LIBS} ${REALSENSE_LIBRARY})
  set(EXTRA_LIBS ${EXTRA_LIBS} ${realsense2_LIBRARY})

endif()

file(GLOB srcs *.cpp)
file(GLOB tools_srcs Tools/*.cpp)

if(WIN32)
  file(GLOB hdrs *.h)
  file(GLOB tools_hdrs Tools/*.h)
endif()

if(WIN32)
  add_definitions(-Dlinux=0)
  add_definitions(-DWIN32_LEAN_AND_MEAN)
  add_definitions(-DNOMINMAX)
  set(ADDITIONAL_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  set(EXTRA_WINDOWS_LIBS ${EXTRA_WINDOWS_LIBS} ws2_32 ${JPEG_LIBRARY})
else()
  add_definitions(-Dlinux=1)
endif()

set(CMAKE_CXX_FLAGS ${ADDITIONAL_CMAKE_CXX_FLAGS} "-O3 -msse2 -msse3 -Wall -std=c++11")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-in-bool-context")




add_executable(ElasticFusion 
               ${srcs}
               ${tools_srcs}
               ${hdrs}
               ${tools_hdrs}
)

target_link_libraries(ElasticFusion
					  ${EXTRA_WINDOWS_LIBS}
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${EXTRA_LIBS}
                      ${EFUSION_LIBRARY}
                      # ${OPENNI2_LIBRARY}
                      "/home/guoqing/software/OpenNI2_other/baidwangpan/OpenNI-Linux-x64-2.2/Tools/libOpenNI2.so"
                      ${SUITESPARSE_LIBRARIES}
                      ${BLAS_LIBRARIES}
                      ${LAPACK_LIBRARIES}
)

message(STATUS "==> EXTRA_WINDOWS_LIBS = " ${EXTRA_WINDOWS_LIBS})
message(STATUS "==> ZLIB_LIBRARY = " ${ZLIB_LIBRARY})
message(STATUS "==> Pangolin_LIBRARIES = " ${Pangolin_LIBRARIES})
message(STATUS "==> CUDA_LIBRARIES = " ${CUDA_LIBRARIES})
message(STATUS "==> EXTRA_LIBS = " ${EXTRA_LIBS})
message(STATUS "==> EFUSION_LIBRARY = " ${EFUSION_LIBRARY})
message(STATUS "==> SUITESPARSE_LIBRARIES = " ${SUITESPARSE_LIBRARIES})
message(STATUS "==> BLAS_LIBRARIES = " ${BLAS_LIBRARIES})
message(STATUS "==> LAPACK_LIBRARIES = " ${LAPACK_LIBRARIES})


INSTALL(TARGETS ElasticFusion
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)
