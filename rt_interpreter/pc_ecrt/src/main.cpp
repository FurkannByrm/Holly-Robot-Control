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

void signal_handler(int) { running = false; }

extern void rt_loop_func(SPSCQueue<RobotState, 128>& s_q, SPSCQueue<RobotCommand, 128>& c_q, std::atomic<bool>& run);
extern void network_server_func(SPSCQueue<RobotState, 128>& s_q, SPSCQueue<RobotCommand, 128>& c_q, std::atomic<bool>& run);

int main() {
    signal(SIGINT, signal_handler);
    
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "mlockall failed! Run with sudo." << std::endl;
    }

    std::cout << "Starting Holly Bridge Node..." << std::endl;

    // RT Thread (Priority 95)
    std::thread rt_thread([&]() {
        struct sched_param param;
        param.sched_priority = 95;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        rt_loop_func(std::ref(state_queue), std::ref(cmd_queue), std::ref(running));
    });

    // Network Thread
    std::thread nw_thread(network_server_func, std::ref(state_queue), std::ref(cmd_queue), std::ref(running));

    if (rt_thread.joinable()) rt_thread.join();
    if (nw_thread.joinable()) nw_thread.join();

    return 0;
}

/*)#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>
#include <pthread.h>
#include <functional>
#include <sys/mman.h> // mlockall için gerekli
#include "protocol.hpp"
#include "spsc_queue.hpp"

std::atomic<bool> running{true};
SPSCQueue<RobotState, 128> state_queue;
SPSCQueue<RobotCommand, 128> cmd_queue;

void signal_handler(int) { 
    std::cout << "\n[Main] Shutdown signal received..." << std::endl;
    running = false; 
}

extern void rt_loop_func(SPSCQueue<RobotState, 128>& s_q, SPSCQueue<RobotCommand, 128>& c_q, std::atomic<bool>& run);
extern void network_server_func(SPSCQueue<RobotState, 128>& s_q, SPSCQueue<RobotCommand, 128>& c_q, std::atomic<bool>& run);

void set_cpu_affinity(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Error setting affinity for CPU " << cpu_id << std::endl;
    }
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        std::cerr << "Warning: Could not lock memory. Run with sudo!" << std::endl;
    }

    std::cout << "Starting Holly Bridge Node [Production Grade]" << std::endl;

    // 1. RT Thread: EtherCAT ve Kontrol Döngüsü
    std::thread rt_thread([&]() {
        set_cpu_affinity(1); // CPU 1 sadece EtherCAT'e bakar
        
        struct sched_param param;
        param.sched_priority = 95; // 99 yerine 95 tercih edilebilir (kernel işlerine yer bırakmak için)
        if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::cerr << "Priority setting failed! Run with sudo." << std::endl;
        }

        rt_loop_func(std::ref(state_queue), std::ref(cmd_queue), std::ref(running));
    });

    std::thread nw_thread([&]() {
        set_cpu_affinity(2); // CPU 2 Network işlerine bakar
        network_server_func(std::ref(state_queue), std::ref(cmd_queue), std::ref(running));
    });

    std::cout << "Holly Bridge is ACTIVE. Press Ctrl+C to stop." << std::endl;
    std::cout << "RT Thread: CPU 1, Priority: SCHED_FIFO 95" << std::endl;
    std::cout << "NW Thread: CPU 2, Priority: Standard" << std::endl;

    if (rt_thread.joinable()) rt_thread.join();
    if (nw_thread.joinable()) nw_thread.join();

    std::cout << "Holly Bridge Node shutdown cleanly." << std::endl;
    return 0;
}*/
