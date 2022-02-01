cmake_minimum_required (VERSION 3.1.0 FATAL_ERROR)
project (calcgraph-example)

include_directories("${PROJECT_SOURCE_DIR}/../include")
link_directories("${PROJECT_SOURCE_DIR}/..")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(example "${PROJECT_SOURCE_DIR}/example.cpp")
set_property(TARGET example PROPERTY CXX_STANDARD 14)
set_property(TARGET example PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(example ${GSL_LIBRARIES} calcgraph Threads::Threads)