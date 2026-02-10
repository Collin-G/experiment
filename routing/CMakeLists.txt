cmake_minimum_required(VERSION 3.10)
project(osm_reader LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find H3 library
find_library(H3_LIBRARY NAMES h3)
find_path(H3_INCLUDE_DIR NAMES h3/h3api.h)

if (NOT H3_LIBRARY OR NOT H3_INCLUDE_DIR)
    message(FATAL_ERROR "H3 library not found. Install it with: brew install h3 (macOS) or apt-get install libh3-dev (Ubuntu)")
endif()

# ========================
# Library
# ========================
add_library(routing STATIC
    src/graph.cpp
    src/osm_parser.cpp
    src/graphbuilder.cpp
    src/astar.cpp
    src/router.cpp
    src/router_api.cpp
    src/matching.cpp
)

target_include_directories(routing
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${H3_INCLUDE_DIR}
        /usr/local/include
)

target_link_libraries(routing
    ${H3_LIBRARY}
    bz2
    boost_system
    boost_iostreams
    boost_date_time
    z
    expat
    pthread
)

# ========================
# Matching Engine Test Executable
# ========================




# ========================
# Original CLI (optional)
# ========================
add_executable(routing_cli
    src/main.cpp
)

target_include_directories(routing_cli
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${H3_INCLUDE_DIR}
)

target_link_libraries(routing_cli routing pthread)