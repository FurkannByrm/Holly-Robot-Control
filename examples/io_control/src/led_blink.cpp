#include <cstdint>
#include <ecrt.h>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <signal.h>
#include <chrono>


#include "bit_set.hpp"


static ec_master_t* master = nullptr;
static ec_domain_t* domain =  nullptr;
static uint8_t* domain_pd  = nullptr;

static unsigned int do_offset;

static bool running = true;

void signal_handler(int){
    running = false;
}


int main(){
    
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    //master
    master = ecrt_request_master(0);
    if(!master){

        std::cerr<<"Failed to get EtherCAT master \n";
        return -1;
    }
    //domain
    domain = ecrt_master_create_domain(master);
    if(!domain){
        std::cerr<<"Failed to create domain \n";
        return -1;
    }
    
    //slave config (EL2008 at position 2)
    ec_slave_config_t* sc =
        ecrt_master_slave_config(master,0, 2 ,  0x00000002, 0x07d83052);
    if(!sc){
        std::cerr<<"Failed to get slave config \n";
        return -1;
    }
  

    static ec_pdo_entry_reg_t domain_regs[] = {


        {0,2,0x00000002,0x07d83052,0x7000, 0x01, &do_offset},{}
    };

    if(ecrt_domain_reg_pdo_entry_list(domain,domain_regs)){
        std::cerr << "PDO entry registiration failed \n";
        return -1;
    }
    
    //activate master
    if(ecrt_master_activate(master)){
        std::cerr<<"master activation failed \n";
        return  -1;
    }

    
domain_pd = ecrt_domain_data(domain);
    
    if(!domain_pd){

        std::cerr<< "Failed to get domain data pointer \n";
        return -1;
    }



    std::cout<< "EL2008 blink started. \n";


    while(running)
    {

    ecrt_master_receive(master);
    ecrt_domain_process(domain);
        
    uint8_t* output_byte = domain_pd + do_offset;
    holly_bitset::toggleBit(*output_byte, holly_bitset::Bit::DO0);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);
    
    usleep(100000);


    }
         

} 

