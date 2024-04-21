cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

project(lifelong)

file(GLOB SOURCES "./src/*.cpp")

IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


# add_executable(lifelong ${SOURCES} )
# add_library(lifelong STATIC ${SOURCES})
# add_subdirectory(./src/pibt2 ./pibt2)
add_subdirectory(./src/pibt2)

# target_include_directories(lifelong INTERFACE ${PROJECT_SOURCE_DIR}/inc ${Boost_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}/src/pibt2)
# target_include_directories(lifelong INTERFACE ${PROJECT_SOURCE_DIR}/inc ${Boost_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/pibt2)
add_executable(lifelong ${SOURCES} )
target_compile_options(lifelong PUBLIC -O3 -Wall -mtune=native -march=native)

# add_subdirectory(./src/pibt2/grid-pathfinding/ ./graph)
target_link_libraries(lifelong ${Boost_LIBRARIES} lib-mapf)
