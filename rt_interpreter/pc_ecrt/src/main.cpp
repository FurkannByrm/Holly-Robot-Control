#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>
#include <sys/mman.h>
#include "pc_ecrt/network_server.hpp"
#include "pc_ecrt/rt_loop.hpp"


std::atomic<bool> running{true};
SPSCQueue<GrsRobotState, 128> state_queue;
SPSCQueue<GrsRobotCommand, 128> command_queue;

void signal_handler(int) { running = false; }

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);  // Prevent crash when TCP client disconnects
    
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "mlockall failed! Run with sudo." << std::endl;
    }

    std::cout << "Starting Holly Bridge Node..." << std::endl;
    std::cout << "Protocol: Unified 128-byte (Motion + I/O)" << std::endl;

    // RT Thread (Priority 95)
    std::thread rt_thread([&]() {
        struct sched_param param;
        param.sched_priority = 95;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        rt_loop_func(std::ref(state_queue), std::ref(command_queue), std::ref(running));
    });

    // Network Thread
    std::thread nw_thread(network_server_func, std::ref(state_queue), std::ref(command_queue), std::ref(running));

    if (rt_thread.joinable()) rt_thread.join();
    if (nw_thread.joinable()) nw_thread.join();

    return 0;
}
