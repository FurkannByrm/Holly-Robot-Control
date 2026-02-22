#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>

#pragma pack(push, 1) 
struct RobotState {
    uint64_t seq_id;         
    uint32_t timestamp;     
    uint8_t  inputs;       
    uint8_t  outputs;        
    uint8_t  is_hardware_emg; 
    uint8_t  system_ready;   
    uint8_t  padding[2];    
};

struct RobotCommand {
    uint64_t cmd_id;       
    uint8_t  set_outputs; 
    uint8_t  soft_stops; 
    uint8_t  padding[6];     // Alignment
};


#pragma pack(pop)

#endif
