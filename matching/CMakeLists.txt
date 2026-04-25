cmake_minimum_required(VERSION 3.10)
project(flux LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── unordered_dense ───────────────────────────────────────────────────────────
set(UNORDERED_DENSE_INCLUDE_DIR 
    ${CMAKE_CURRENT_SOURCE_DIR}/../third-party/unordered_dense/include)

# ── H3 ───────────────────────────────────────────────────────────────────────
find_library(H3_LIBRARY NAMES h3)
find_path(H3_INCLUDE_DIR NAMES h3/h3api.h)
if(NOT H3_LIBRARY OR NOT H3_INCLUDE_DIR)
    message(FATAL_ERROR "H3 not found. Install: brew install h3 or apt install libh3-dev")
endif()

# ── nlohmann_json ─────────────────────────────────────────────────────────────
find_package(nlohmann_json REQUIRED)

# ── librdkafka ────────────────────────────────────────────────────────────────
find_library(RDKAFKA_LIBRARY   NAMES rdkafka)
find_library(RDKAFKACPP_LIBRARY NAMES rdkafka++)
find_path(RDKAFKA_INCLUDE_DIR NAMES librdkafka/rdkafkacpp.h)
if(NOT RDKAFKA_LIBRARY OR NOT RDKAFKACPP_LIBRARY OR NOT RDKAFKA_INCLUDE_DIR)
    message(FATAL_ERROR "librdkafka not found. Install: brew install librdkafka or apt install librdkafka-dev")
endif()

# ── matching library ──────────────────────────────────────────────────────────
add_library(matching STATIC
    src/engine/matching_engine.cpp
    src/threads.cpp
    src/events/json_utils.cpp
)

target_include_directories(matching
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${H3_INCLUDE_DIR}
        ${RDKAFKA_INCLUDE_DIR}
        ${UNORDERED_DENSE_INCLUDE_DIR}
        /usr/local/include
)

target_link_libraries(matching
    ${H3_LIBRARY}
    ${RDKAFKA_LIBRARY}
    ${RDKAFKACPP_LIBRARY}
    pthread
    nlohmann_json::nlohmann_json
)

# ── executable ────────────────────────────────────────────────────────────────
add_executable(matching_cli
    src/main.cpp
)

target_include_directories(matching_cli
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${H3_INCLUDE_DIR}
        ${RDKAFKA_INCLUDE_DIR}
        ${UNORDERED_DENSE_INCLUDE_DIR}
)

target_link_libraries(matching_cli
    matching
    pthread
)