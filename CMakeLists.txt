cmake_minimum_required (VERSION 3.1.0 FATAL_ERROR)
project (calcgraph VERSION 1.0.0)

# The library itself
FILE(GLOB SRCFILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(calcgraph STATIC ${SRCFILES})
set_property(TARGET calcgraph PROPERTY CXX_STANDARD 14)
set_property(TARGET calcgraph PROPERTY CXX_STANDARD_REQUIRED ON)
target_include_directories(calcgraph PUBLIC "${PROJECT_SOURCE_DIR}/include")
install(FILES ${PROJECT_SOURCE_DIR}/include/calcgraph.h DESTINATION include)
install(TARGETS calcgraph DESTINATION lib)

# Examples
option(WITH_EXAMPLES "build the example applications " ON)
if(WITH_EXAMPLES)
    add_subdirectory(example)
endif(WITH_EXAMPLES)

# Unit tests
add_subdirectory(test)

# pkg-config
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
  @ONLY
)
install(FILES ${PROJECT_SOURCE_DIR}/calcgraph.pc DESTINATION lib/pkgconfig)


# Man pages, see: https://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf @ONLY)
	add_custom_target(
		doc ALL ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM)
	install(DIRECTORY ${PROJECT_SOURCE_DIR}/man DESTINATION share USE_SOURCE_PERMISSIONS)
endif(DOXYGEN_FOUND)


