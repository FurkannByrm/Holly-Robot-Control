#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>

#pragma pack(push, 1)

// ═══════════════════════════════════════════════════════════════
// Legacy Protocol — EtherCAT I/O only (EL1008 + EL2008)
// ═══════════════════════════════════════════════════════════════

struct RobotState {
    uint64_t seq_id;         
    uint32_t timestamp;     
    uint8_t  inputs;       
    uint8_t  outputs;        
    uint8_t  is_hardware_emg; 
    uint8_t  system_ready;   
    uint8_t  padding[2];    
};
// sizeof(RobotState) = 18

struct RobotCommand {
    uint64_t cmd_id;       
    uint8_t  set_outputs; 
    uint8_t  soft_stops; 
    uint8_t  padding[6];     // Alignment
};
// sizeof(RobotCommand) = 16

// ═══════════════════════════════════════════════════════════════
// Extended Protocol — Full Robot Command (Motion + I/O + Wait)
// GRS Interpreter -> Hardware Controller
// ═══════════════════════════════════════════════════════════════

enum GrsCommandType : uint8_t {
    GRS_CMD_NOP        = 0,
    GRS_CMD_PTP        = 1,
    GRS_CMD_PTP_REL    = 2,
    GRS_CMD_LIN        = 3,
    GRS_CMD_LIN_REL    = 4,
    GRS_CMD_CIRC       = 5,
    GRS_CMD_CIRC_REL   = 6,
    GRS_CMD_SPLINE     = 7,
    GRS_CMD_SPLINE_REL = 8,
    GRS_CMD_WAIT       = 9,
    GRS_CMD_OUTPUT     = 10,
    GRS_CMD_SET_ALL_OUTPUTS = 11,  // Legacy compat: set full output byte
};

// Extended command: carries motion, wait, and I/O data
// Total: 128 bytes (fixed size for easy parsing)
struct GrsRobotCommand {
    uint64_t cmd_id;           // 8   - sequential command ID
    uint8_t  cmd_type;         // 1   - GrsCommandType enum
    uint8_t  io_index;         // 1   - I/O bit index (0-based, for OUTPUT)
    uint8_t  io_value;         // 1   - I/O value (0 or 1)
    uint8_t  set_outputs;      // 1   - full output byte (legacy compat)
    uint8_t  soft_stops;       // 1   - soft emergency stop
    uint8_t  reserved[3];      // 3   - future use
    double   wait_time;        // 8   - wait duration in ms (for WAIT)
    double   coords[6];        // 48  - x,y,z,a,b,c (for PTP/LIN/CIRC)
    double   axes[6];          // 48  - A1,A2,A3,A4,A5,A6 (for axis commands)
    uint8_t  padding[8];       // 8   - alignment to 128 bytes
};
// sizeof(GrsRobotCommand) = 128

// Extended state: carries robot position + I/O state
// Hardware Controller -> GRS Interpreter
// Total: 128 bytes
struct GrsRobotState {
    uint64_t seq_id;           // 8   - senkronizasyon ID
    uint32_t timestamp;        // 4   - zaman damgası (ms)
    uint8_t  inputs;           // 1   - digital input byte (EL1008)
    uint8_t  outputs;          // 1   - digital output byte (EL2008)
    uint8_t  is_hardware_emg;  // 1   - hardware emergency stop
    uint8_t  system_ready;     // 1   - system ready flag
    uint8_t  cmd_ack;          // 1   - last acknowledged cmd_type
    uint8_t  cmd_status;       // 1   - 0=idle, 1=executing, 2=done, 3=error
    uint8_t  reserved[2];      // 2   - future use
    double   current_pos[6];   // 48  - current x,y,z,a,b,c
    double   current_axes[6];  // 48  - current A1,A2,A3,A4,A5,A6
    uint8_t  padding[12];      // 12  - alignment to 128 bytes
};
// sizeof(GrsRobotState) = 128

#pragma pack(pop)

// ═══════════════════════════════════════════════════════════════
// Helper: command type name for logging
// ═══════════════════════════════════════════════════════════════
inline const char* grsCommandTypeName(uint8_t type) {
    switch (type) {
        case GRS_CMD_NOP:        return "NOP";
        case GRS_CMD_PTP:        return "PTP";
        case GRS_CMD_PTP_REL:    return "PTP_REL";
        case GRS_CMD_LIN:        return "LIN";
        case GRS_CMD_LIN_REL:    return "LIN_REL";
        case GRS_CMD_CIRC:       return "CIRC";
        case GRS_CMD_CIRC_REL:   return "CIRC_REL";
        case GRS_CMD_SPLINE:     return "SPLINE";
        case GRS_CMD_SPLINE_REL: return "SPLINE_REL";
        case GRS_CMD_WAIT:       return "WAIT";
        case GRS_CMD_OUTPUT:     return "OUTPUT";
        case GRS_CMD_SET_ALL_OUTPUTS: return "SET_ALL_OUTPUTS";
        default:                 return "UNKNOWN";
    }
}

#endif
