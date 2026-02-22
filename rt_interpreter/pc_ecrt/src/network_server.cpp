#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <iostream>
#include <atomic>
#include "protocol.hpp"
#include "spsc_queue.hpp"

void network_server_func(SPSCQueue<RobotState, 128>& s_q, 
                         SPSCQueue<RobotCommand, 128>& c_q, 
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

    while (run) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket < 0) continue;

        // TCP_NODELAY
        int flag = 1;
        setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

        std::cout << "[Network] Client connected!" << std::endl;

        while (client_socket > 0 && run) {
            // 1.State
            auto state = s_q.pop();
            if (state) {
                send(client_socket, &(*state), sizeof(RobotState), 0);
            }

            // 2.Command - MSG_DONTWAIT 
            RobotCommand cmd;
            int valread = recv(client_socket, &cmd, sizeof(RobotCommand), MSG_DONTWAIT);
            if (valread == sizeof(RobotCommand)) {
                c_q.push(cmd);
            } else if (valread == 0) { //not connected
                break;
            }
            
            // if queue is emty,processors is relaxed 
            if (!state) usleep(500); 
        }
        close(client_socket);
        std::cout << "[Network] Client disconnected." << std::endl;
    }
    close(server_fd);
}
