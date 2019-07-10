project(differ_engine)

#include and src file
file(GLOB_RECURSE project_files_header *.h)
file(GLOB_RECURSE project_files_cpp *.cpp)

add_library(${PROJECT_NAME} ${project_files_header} ${project_files_cpp})

target_include_directories(${PROJECT_NAME} PRIVATE .)
