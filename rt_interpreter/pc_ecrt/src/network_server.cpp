#include "pc_ecrt/network_server.hpp"

namespace  {

void logGrsCommand(const GrsRobotCommand& cmd) {
    std::cout << "  [GRS CMD #" << cmd.cmd_id << "] " 
              << grsCommandTypeName(cmd.cmd_type);

    switch (cmd.cmd_type) {
        case GRS_CMD_OUTPUT:
            std::cout << " $OUT[" << (int)(cmd.io_index + 1) << "] = " 
                      << (cmd.io_value ? "TRUE" : "FALSE");
            break;

        case GRS_CMD_SET_ALL_OUTPUTS:
            std::cout << " outputs=" << std::bitset<8>(cmd.set_outputs);
            break;

        case GRS_CMD_WAIT:
            std::cout << " duration=" << cmd.wait_time << "ms";
            break;

        case GRS_CMD_PTP:
        case GRS_CMD_PTP_REL:
        case GRS_CMD_LIN:
        case GRS_CMD_LIN_REL:
        case GRS_CMD_CIRC:
        case GRS_CMD_CIRC_REL:
        case GRS_CMD_SPLINE:
        case GRS_CMD_SPLINE_REL: {
            // Check if coords or axes have non-zero values
            bool hasCoords = false, hasAxes = false;
            for (int i = 0; i < 6; i++) {
                if (cmd.coords[i] != 0.0) hasCoords = true;
                if (cmd.axes[i] != 0.0) hasAxes = true;
            }
            if (hasCoords) {
                static const char* coordNames[] = {"X","Y","Z","A","B","C"};
                std::cout << " pos={";
                bool first = true;
                for (int i = 0; i < 6; i++) {
                    if (cmd.coords[i] != 0.0) {
                        if (!first) std::cout << ", ";
                        std::cout << coordNames[i] << "=" << std::fixed 
                                  << std::setprecision(2) << cmd.coords[i];
                        first = false;
                    }
                }
                std::cout << "}";
            }
            if (hasAxes) {
                std::cout << " axes={";
                bool first = true;
                for (int i = 0; i < 6; i++) {
                    if (cmd.axes[i] != 0.0) {
                        if (!first) std::cout << ", ";
                        std::cout << "A" << (i+1) << "=" << std::fixed 
                                  << std::setprecision(2) << cmd.axes[i];
                        first = false;
                    }
                }
                std::cout << "}";
            }
            break;
        }

        case GRS_CMD_NOP:
        default:
            break;
    }

    if (cmd.soft_stops) std::cout << " [SOFT_STOP]";
    std::cout << std::endl;
}

}


void network_server_func(SPSCQueue<GrsRobotState, 128>& s_q, 
                         SPSCQueue<GrsRobotCommand, 128>& ext_cmd_q,
                         std::atomic<bool>& run) {
    
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) return;

    // (SO_REUSEADDR)
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(12345);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) return;
    listen(server_fd, 3);

    std::cout << "[Network] Server listening on port 12345" << std::endl;
    std::cout << "[Network] Supports legacy (16-byte) and extended (128-byte) protocol" << std::endl;

    while (run) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket < 0) continue;

        // TCP_NODELAY
        int flag = 1;
        setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

        std::cout << "[Network] Client connected!" << std::endl;

        // Auto-detect protocol on first recv
        bool protocol_detected = false;

        while (client_socket > 0 && run) {
            // 1. State — send to client (format depends on detected protocol)
            // Build extended state from legacy queue + simulated position
                auto state = s_q.pop();
                if (state) {
                    GrsRobotState extState{};
                    extState.seq_id = state->seq_id;
                    extState.timestamp = state->timestamp;
                    extState.inputs = state->inputs;
                    extState.outputs = state->outputs;
                    extState.is_hardware_emg = state->is_hardware_emg;
                    extState.system_ready = state->system_ready;
                    // cmd_ack, cmd_status, current_pos, current_axes
                    // currently no actual motion hardware — zeros (simulated)
                    ssize_t sent = send(client_socket, &extState, sizeof(GrsRobotState), MSG_NOSIGNAL);
                    if (sent <= 0) break;  // Client disconnected
                }
             
            // 2. Command — recv from client (MSG_DONTWAIT for non-blocking)
            // Peek first to determine packet size
            uint8_t peekBuf[128];
            int valread = recv(client_socket, peekBuf, sizeof(peekBuf), MSG_DONTWAIT);
            
            if (valread == (int)sizeof(GrsRobotCommand)) {
                // Extended 128-byte command
                if (!protocol_detected) {
                    protocol_detected = true;
                    std::cout << "[Network] Extended protocol detected (128-byte)" << std::endl;
                }
                GrsRobotCommand extCmd;
                std::memcpy(&extCmd, peekBuf, sizeof(GrsRobotCommand));
                
                // Log the command
                logGrsCommand(extCmd);

                // Push to extended queue for rt_loop processing
                ext_cmd_q.push(extCmd);

                // Also create a legacy RobotCommand for I/O operations
                // so rt_loop can apply outputs via EtherCAT
                if (extCmd.cmd_type == GRS_CMD_OUTPUT || extCmd.cmd_type == GRS_CMD_SET_ALL_OUTPUTS) {
                    GrsRobotCommand legacyCmd{};
                    legacyCmd.cmd_id = extCmd.cmd_id;
                    legacyCmd.set_outputs = extCmd.set_outputs;
                    legacyCmd.soft_stops = extCmd.soft_stops;
                    ext_cmd_q.push(legacyCmd);
                }

            } else if (valread == 0) {
                // Client disconnected
                break;
            }
            // valread < 0 means EAGAIN (no data available) — normal for non-blocking
            
            // If queue is empty, relax the processor
            if (!s_q.pop()) usleep(500);
        }
        close(client_socket);
        
        // Clear all outputs when client disconnects (safety)
        GrsRobotCommand clearCmd{};
        clearCmd.set_outputs = 0;
        clearCmd.soft_stops = 0;
        ext_cmd_q.push(clearCmd);
        
        std::cout << "[Network] Client disconnected. Outputs cleared." << std::endl;
    }
    close(server_fd);
}
