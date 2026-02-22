cmake_minimum_required(VERSION 3.10)
project(io_control)

set(CMAKE_CXX_STANDARD 17)
set(ETHERCAT_ROOT "/home/upxtreme/ethercat")

include_directories(include/io_control)

add_executable(io_log src/io_log.cpp)
target_include_directories(io_log PRIVATE ${ETHERCAT_ROOT}/include)
target_link_directories(io_log PRIVATE ${ETHERCAT_ROOT}/lib/.libs)
target_link_libraries(io_log PRIVATE ethercat)
set_target_properties(io_log PROPERTIES BUILD_RPATH "${ETHERCAT_ROOT}/lib/.libs")

 
   
add_executable(led_blink src/led_blink.cpp)
target_include_directories(led_blink PRIVATE ${ETHERCAT_ROOT}/include)
target_link_directories(led_blink PRIVATE ${ETHERCAT_ROOT}/lib/.libs)
target_link_libraries(led_blink PRIVATE ethercat)
set_target_properties(led_blink PROPERTIES BUILD_RPATH "${ETHERCAT_ROOT}/lib/.libs")
