cmake_minimum_required(VERSION 3.10)
project(HollyBridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# EtherCAT Yolu (Senin sistemine özel)
set(ETHERCAT_ROOT "/home/upxtreme/ethercat")

# 1. Klasör yollarını tanımla
set(COMMON_DIR "${CMAKE_SOURCE_DIR}/../common")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# 2. Header dosyalarını dahil et
include_directories(
    ${INCLUDE_DIR}
    ${COMMON_DIR}
    ${ETHERCAT_ROOT}/include
)

# 3. Kaynak dosyaları
set(SOURCES
    src/main.cpp
    src/rt_loop.cpp
    src/network_server.cpp
)

# 4. Executable oluştur
add_executable(ec_bridge_node ${SOURCES})

# 5. EtherCAT Kütüphanesini bağla
target_link_directories(ec_bridge_node PRIVATE ${ETHERCAT_ROOT}/lib/.libs)
target_link_libraries(ec_bridge_node PRIVATE ethercat pthread) # Real-time için pthread şart

# 6. RPATH ayarı (Kütüphaneyi çalışma anında bulması için)
set_target_properties(ec_bridge_node PROPERTIES 
    BUILD_RPATH "${ETHERCAT_ROOT}/lib/.libs"
    INSTALL_RPATH "${ETHERCAT_ROOT}/lib/.libs"
)

# 7. Debug için ek ayar (Opsiyonel ama önerilir)
target_compile_options(ec_bridge_node PRIVATE -Wall -Wextra -O3)
