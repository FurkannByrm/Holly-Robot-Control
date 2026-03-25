cmake_minimum_required(VERSION 3.10)
project(HolyBridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ETHERCAT_ROOT "/home/upxtreme/ethercat")

set(COMMON_DIR "${CMAKE_SOURCE_DIR}/../common")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(
    ${INCLUDE_DIR}
    ${COMMON_DIR}
    ${ETHERCAT_ROOT}/include
)

set(SOURCES
    src/main.cpp
    src/rt_loop.cpp
    src/network_server.cpp
)

add_executable(ec_bridge_node ${SOURCES})

target_link_directories(ec_bridge_node PRIVATE ${ETHERCAT_ROOT}/lib/.libs)
target_link_libraries(ec_bridge_node PRIVATE ethercat pthread) 

set_target_properties(ec_bridge_node PROPERTIES 
    BUILD_RPATH "${ETHERCAT_ROOT}/lib/.libs"
    INSTALL_RPATH "${ETHERCAT_ROOT}/lib/.libs"
)

target_compile_options(ec_bridge_node PRIVATE -Wall -Wextra -O3)
