cmake_minimum_required(VERSION 3.10)
project(osm_reader LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ========================
# Library
# ========================
add_library(routing SHARED
    src/graph.cpp
    src/osm_parser.cpp
    src/graphbuilder.cpp
    src/astar.cpp
    src/router.cpp
    src/router_api.cpp
)

target_include_directories(routing
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        /usr/local/include
)

target_link_libraries(routing
    bz2
    boost_system
    boost_iostreams
    boost_date_time
    z
    expat
)

# ========================
# Executable (optional)
# ========================
add_executable(routing_cli
    src/main.cpp
)

target_link_libraries(routing_cli routing)
