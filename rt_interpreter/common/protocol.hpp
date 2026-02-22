#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>

#pragma pack(push, 1) // Memory padding'i kapat

struct RobotState {
    uint64_t seq_id;         // Senkronizasyon ID'si
    uint32_t timestamp;      // Zaman damgası
    uint8_t  inputs;         // EL1008 verisi
    uint8_t  outputs;        // EL2008 mevcut durumu
    uint8_t  is_hardware_emg; // 0 veya 1
    uint8_t  system_ready;    // 0 veya 1
    uint8_t  padding[2];      // 8-byte alignment için opsiyonel
};

struct RobotCommand {
    uint64_t cmd_id;         // Komut ID'si
    uint8_t  set_outputs;    // EL2008'e yazılacak değer
    uint8_t  soft_stops;      // 0 veya 1
    uint8_t  padding[6];     // Alignment
};


#pragma pack(pop)

#endif
