#ifndef NETWORK_SERVER_HPP_
#define NETWORK_SERVER_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <atomic>
#include <bitset>
#include "protocol.hpp"
#include "spsc_queue.hpp"


void network_server_func(SPSCQueue<GrsRobotState, 128>& s_q, 
                         SPSCQueue<GrsRobotCommand, 128>& c_q,
                         std::atomic<bool>& run);



#endif //NETWORK_SERVER_HPP_
