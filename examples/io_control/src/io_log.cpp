#include <ecrt.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <signal.h>
#include "bit_set.hpp"
#include <atomic>
#include <thread>
#include <chrono>


static ec_master_t* master = nullptr;
static ec_domain_t* domain = nullptr;
static uint8_t* domain_pd = nullptr;

static unsigned int di_offset; 
static unsigned int do_offset;
static bool running = true;


void signal_handler(int){
    running = false;
}



std::atomic<uint8_t> shared_output_state{0};
// --- LOGGING THREAD ---
void logging_thread_func() {
    uint8_t last_state = 255;
    while (running) {
        uint8_t current_state = shared_output_state.load();
        
        if (current_state != last_state) {
            std::cout << "[LOG] Output State Changed: " 
                      << static_cast<int>(current_state) << std::endl;
            last_state = current_state;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(){
    

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);



    master = ecrt_request_master(0);
    if(!master){
        std::cerr<<"Failed to get EtherCAT master\n";
        return -1;
    }
    
    // 2) Domain
    domain = ecrt_master_create_domain(master);
    if (!domain) {
        std::cerr << "Failed to create domain\n";
        return -1;
    }

    // 3) Slave config (EL2008 at position 2)
    ec_slave_config_t* sc =
        ecrt_master_slave_config(master, 0, 2, 0x00000002, 0x07d83052);
    if (!sc) {
        std::cerr << "Failed to get slave config\n";
        return -1;
    }

    // 4) PDO entry registration
static ec_pdo_entry_reg_t domain_regs[] = {
    // Slave 1: EL1008 (Giriş) - 0x6000:01
    {0, 1, 0x00000002, 0x03f03052, 0x6000, 0x01, &di_offset},
    // Slave 2: EL2008 (Çıkış)
    {0, 2, 0x00000002, 0x07d83052, 0x7000, 0x01, &do_offset},
    {}
};
    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        std::cerr << "PDO entry registration failed\n";
        return -1;
    }

    // 5) Activate master
    if (ecrt_master_activate(master)) {
        std::cerr << "Master activation failed\n";
        return -1;
    }

    domain_pd = ecrt_domain_data(domain);
    if (!domain_pd) {
        std::cerr << "Failed to get domain data pointer\n";
        return -1;
    }

    std::cout << "EL2008 blink started. Ctrl+C to stop.\n";
    bool led = false;

    
std::thread logger(logging_thread_func);

    while (running) {
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // 1. INPUT OKUMA (EL1008)
    uint8_t* input_byte = domain_pd + di_offset;
    bool emg_pressed = holly_bitset::test(*input_byte, holly_bitset::Bit::DO0); 

    // 2. MANTIK (Logic)
    uint8_t* output_byte = domain_pd + do_offset;
    if (emg_pressed) {
        holly_bitset::setBit(*output_byte, holly_bitset::Bit::DO0);
    } else {
        holly_bitset::clearBit(*output_byte, holly_bitset::Bit::DO0);
    }

    // 3. LOGGING (Atomic based)
    shared_output_state.store(*output_byte);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    usleep(1000); // 1ms döngü   
 

    }
    }

