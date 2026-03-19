#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>
#include <sys/mman.h>
#include "protocol.hpp"
#include "spsc_queue.hpp"

std::atomic<bool> running{true};
SPSCQueue<RobotState, 128> state_queue;
SPSCQueue<RobotCommand, 128> cmd_queue;
SPSCQueue<GrsRobotCommand, 128> ext_cmd_queue;
std::atomic<bool> ext_mode{false};

void signal_handler(int) { running = false; }

extern void rt_loop_func(SPSCQueue<RobotState, 128>& s_q, 
                         SPSCQueue<RobotCommand, 128>& c_q, 
                         SPSCQueue<GrsRobotCommand, 128>& ext_q,
                         std::atomic<bool>& run,
                         std::atomic<bool>& ext_mode);

extern void network_server_func(SPSCQueue<RobotState, 128>& s_q, 
                                SPSCQueue<RobotCommand, 128>& c_q, 
                                SPSCQueue<GrsRobotCommand, 128>& ext_q,
                                std::atomic<bool>& run,
                                std::atomic<bool>& ext_mode);

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "mlockall failed! Run with sudo." << std::endl;
    }

    std::cout << "Starting Holly Bridge Node..." << std::endl;
    std::cout << "Supports legacy I/O (16-byte) and extended GRS (128-byte) protocol" << std::endl;

    // RT Thread (Priority 95)
    std::thread rt_thread([&]() {
        struct sched_param param;
        param.sched_priority = 95;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        rt_loop_func(std::ref(state_queue), std::ref(cmd_queue), 
                     std::ref(ext_cmd_queue), std::ref(running), std::ref(ext_mode));
    });

    // Network Thread
    std::thread nw_thread(network_server_func, 
                          std::ref(state_queue), std::ref(cmd_queue), 
                          std::ref(ext_cmd_queue), std::ref(running), std::ref(ext_mode));

    if (rt_thread.joinable()) rt_thread.join();
    if (nw_thread.joinable()) nw_thread.join();

    return 0;
}
