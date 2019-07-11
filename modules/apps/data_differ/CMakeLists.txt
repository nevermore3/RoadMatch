project(sdmap_differ_fusion)

#include and src file
file(GLOB_RECURSE project_header *.h)
file(GLOB_RECURSE project_cpp *.cpp)

#exe
add_executable(${PROJECT_NAME} ${project_cpp})

target_link_libraries(${PROJECT_NAME} PocoNet PocoUtil PocoXML PocoJSON PocoFoundation pthread)
target_link_libraries(${PROJECT_NAME} differ_engine glog autohdmap_core autohdmap_api geos)

target_include_directories(${PROJECT_NAME} PRIVATE ../../differ)