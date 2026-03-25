#ifndef RT_LOOP_HPP_
#define RTLOOP_HPP_
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ecrt.h>
#include "protocol.hpp"
#include "spsc_queue.hpp"
#include "bitset.hpp"
 
void rt_loop_func(SPSCQueue<GrsRobotState, 128>& s_q, 
            SPSCQueue<GrsRobotCommand, 128>& c_q,
            std::atomic<bool>& run);

#endif //RT_LOOP_HPP
