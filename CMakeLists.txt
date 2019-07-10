cmake_minimum_required(VERSION 3.13)
project(hdmap_differ_fusion C CXX)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX d)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(BuildType)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/installed"  CACHE PATH "default install path" FORCE )
endif()

message("project[${PROJECT_NAME}] install path=${CMAKE_INSTALL_PREFIX}")

#ignore the shape lib warning
if ( CMAKE_HOST_APPLE )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-tautological-compare")
    set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wno-tautological-compare")
endif ( CMAKE_HOST_APPLE )
#set msvc /utf-8
if(MSVC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /utf-8")
    set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} /utf-8")
endif(MSVC)


set(NDS_LIBS_DIR ${CMAKE_SOURCE_DIR}/../nds-libs/depends)
set(AUTOHDMAP_LIBS_DIR ${CMAKE_SOURCE_DIR}/../autohdmap-libs)


#link 目录不能省略
if(CMAKE_HOST_WIN32)
elseif(CMAKE_HOST_APPLE)
    link_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/darwin/poco/lib)
    link_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/darwin/geos/lib)
    link_directories(${AUTOHDMAP_LIBS_DIR}/modules/server/lib/darwin)
    link_directories(${AUTOHDMAP_LIBS_DIR}/modules/api/lib/darwin)
    link_directories(${AUTOHDMAP_LIBS_DIR}/modules/core/lib/darwin)
    link_directories(${NDS_LIBS_DIR}/thirdparty/glog/lib)
elseif(CMAKE_HOST_UNIX)
    link_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/linux/poco/lib)
    link_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/linux/geos/lib)
    link_directories(${AUTOHDMAP_LIBS_DIR}/modules/server/lib/linux)
    link_directories(${AUTOHDMAP_LIBS_DIR}/modules/api/lib/linux)
    link_directories(${AUTOHDMAP_LIBS_DIR}/modules/core/lib/linux)
    link_directories(${NDS_LIBS_DIR}/thirdparty/glog/lib)
endif()

#thirdparty include
#include_directories(${HDMAP_LIBS_DIR}/thirdparty/include)
if(CMAKE_HOST_WIN32)
elseif(CMAKE_HOST_APPLE)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/darwin/poco/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/darwin/geos/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/darwin/opencv/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/modules/server/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/modules/api/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/modules/core/include)
    include_directories(${NDS_LIBS_DIR}/thirdparty/glog/include)
elseif(CMAKE_HOST_UNIX)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/linux/poco/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/linux/geos/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/platform/linux/opencv3/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/thirdparty/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/modules/server/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/modules/api/include)
    include_directories(${AUTOHDMAP_LIBS_DIR}/modules/core/include)
    include_directories(${NDS_LIBS_DIR}/thirdparty/glog/include)
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/modules)