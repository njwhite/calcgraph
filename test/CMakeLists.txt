cmake_minimum_required (VERSION 3.1.0 FATAL_ERROR)
project (calcgraph-tests)

include_directories("${PROJECT_SOURCE_DIR}/../include")
link_directories("${PROJECT_SOURCE_DIR}/..")

find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPUNIT REQUIRED cppunit)
include_directories(${CPPUNIT_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(unittests "${PROJECT_SOURCE_DIR}/test.cpp")
set_property(TARGET unittests PROPERTY CXX_STANDARD 14)
set_property(TARGET unittests PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(unittests ${CPPUNIT_LIBRARIES} calcgraph Threads::Threads)