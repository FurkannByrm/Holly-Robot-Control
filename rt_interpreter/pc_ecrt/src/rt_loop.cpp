#include <iostream>
#include <chrono>
#include <ecrt.h>
#include "protocol.hpp"
#include "spsc_queue.hpp"
#include "bitset.hpp"


static ec_master_t *master = NULL;
static ec_domain_t *domain = NULL;
static uint8_t *domain_pd = NULL;

static unsigned int off_el1008_in;
static unsigned int off_el2008_out;

static ec_pdo_entry_reg_t domain_regs[] = {
    {0, 1, 0x00000002, 0x03f03052, 0x6000, 0x01, &off_el1008_in},
    {0, 2, 0x00000002, 0x07d83052, 0x7000, 0x01, &off_el2008_out},
    {}
};

void rt_loop_func(SPSCQueue<RobotState, 128>& s_q, 
                  SPSCQueue<RobotCommand, 128>& c_q,
                  SPSCQueue<GrsRobotCommand, 128>& ext_q,
                  std::atomic<bool>& run,
                  std::atomic<bool>& ext_mode) 
{
    master = ecrt_request_master(0);
    if (!master) return;
    domain = ecrt_master_create_domain(master);
    if (!domain || ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) return;
    if (ecrt_master_activate(master)) return;
    domain_pd = ecrt_domain_data(domain);

    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    const uint64_t PERIOD_NS = 1000000; // 1ms

    uint8_t last_client_cmd = 0; 
    uint64_t last_id = 0;
    uint8_t last_cmd_ack = 0;     // For extended: last acknowledged cmd_type
    uint8_t last_cmd_status = 0;  // 0=idle, 2=done

    std::cout << "[RT] Bridge Active. Mode: Hardware Interlock (EMG Priority)" << std::endl;

    while (run) {
        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= 1000000000) {
            wakeup_time.tv_sec++;
            wakeup_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        // --- 1. Input (EL1008) ---
        uint8_t raw_input = *(domain_pd + off_el1008_in);
        
        // NC Button logic: 1 = (SAFE), 0 = (EMG)
        bool is_system_safe = holly_bitset::test(raw_input, holly_bitset::Bit::DO0);

        // --- 2. Process Legacy Commands (RobotCommand) ---
        auto cmd = c_q.pop();
        if (cmd) {
            last_client_cmd = cmd->set_outputs;
            last_id = cmd->cmd_id;
        }

        // --- 3. Process Extended Commands (GrsRobotCommand) ---
        // Extended commands carry motion/wait/output data
        // Motion & WAIT: simulated (logged by network_server), acknowledged immediately
        // OUTPUT: applied to last_client_cmd for EtherCAT output
        auto extCmd = ext_q.pop();
        if (extCmd) {
            last_id = extCmd->cmd_id;
            ext_mode.store(true);

            switch (extCmd->cmd_type) {
                case GRS_CMD_OUTPUT:
                    // Apply single bit to output byte
                    if (extCmd->io_value)
                        last_client_cmd |= (1u << extCmd->io_index);
                    else
                        last_client_cmd &= ~(1u << extCmd->io_index);
                    last_cmd_ack = GRS_CMD_OUTPUT;
                    last_cmd_status = 2; // done
                    break;

                case GRS_CMD_SET_ALL_OUTPUTS:
                    last_client_cmd = extCmd->set_outputs;
                    last_cmd_ack = GRS_CMD_SET_ALL_OUTPUTS;
                    last_cmd_status = 2;
                    break;

                case GRS_CMD_PTP:
                case GRS_CMD_PTP_REL:
                case GRS_CMD_LIN:
                case GRS_CMD_LIN_REL:
                case GRS_CMD_CIRC:
                case GRS_CMD_CIRC_REL:
                case GRS_CMD_SPLINE:
                case GRS_CMD_SPLINE_REL:
                    // No actual motion hardware — simulate as done immediately
                    // Command details already logged by network_server
                    last_cmd_ack = extCmd->cmd_type;
                    last_cmd_status = 2; // done
                    break;

                case GRS_CMD_WAIT:
                    // WAIT is handled on the interpreter side (sleep)
                    // Just acknowledge it
                    last_cmd_ack = GRS_CMD_WAIT;
                    last_cmd_status = 2;
                    break;

                default:
                    last_cmd_ack = extCmd->cmd_type;
                    last_cmd_status = 0; // idle
                    break;
            }
        }

        // --- 4. Safety priority ---
        uint8_t final_output = 0;

        if (is_system_safe) {
            final_output = last_client_cmd;
        } else {
            // Emergency: kill all outputs
            final_output = 0;
        }

        // --- 5. Write output (EL2008) ---
        *(domain_pd + off_el2008_out) = final_output;

        // --- 6. State feedback ---
        RobotState st{};
        st.seq_id = last_id;
        st.timestamp = (uint32_t)wakeup_time.tv_nsec;
        st.inputs = raw_input;
        st.outputs = final_output;
        st.is_hardware_emg = !is_system_safe; 
        st.system_ready = is_system_safe; 
        
        s_q.push(st);

        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }
    ecrt_release_master(master);
}
