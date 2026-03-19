#include <iostream>
#include <iomanip>
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

// ═══════════════════════════════════════════════════════════════
// Log motion command positions (simulated — no actual servo yet)
// ═══════════════════════════════════════════════════════════════
static void logMotionTarget(const GrsRobotCommand& cmd) {
    static const char* coordNames[] = {"X","Y","Z","A","B","C"};
    static const char* axisNames[]  = {"A1","A2","A3","A4","A5","A6"};

    std::cout << "  [RT MOTION] " << grsCommandTypeName(cmd.cmd_type) << " → ";
    
    // Check for cartesian coordinates
    bool hasCoords = false;
    for (int i = 0; i < 6; i++) if (cmd.coords[i] != 0.0) { hasCoords = true; break; }
    
    if (hasCoords) {
        std::cout << "pos={";
        bool first = true;
        for (int i = 0; i < 6; i++) {
            if (cmd.coords[i] != 0.0) {
                if (!first) std::cout << ", ";
                std::cout << coordNames[i] << "=" << std::fixed << std::setprecision(2) << cmd.coords[i];
                first = false;
            }
        }
        std::cout << "}";
    }
    
    // Check for axis values
    bool hasAxes = false;
    for (int i = 0; i < 6; i++) if (cmd.axes[i] != 0.0) { hasAxes = true; break; }
    
    if (hasAxes) {
        if (hasCoords) std::cout << " ";
        std::cout << "axes={";
        bool first = true;
        for (int i = 0; i < 6; i++) {
            if (cmd.axes[i] != 0.0) {
                if (!first) std::cout << ", ";
                std::cout << axisNames[i] << "=" << std::fixed << std::setprecision(2) << cmd.axes[i];
                first = false;
            }
        }
        std::cout << "}";
    }
    
    std::cout << std::endl;
}

void rt_loop_func(SPSCQueue<RobotState, 128>& s_q, 
                  SPSCQueue<RobotCommand, 128>& c_q,
                  SPSCQueue<GrsRobotCommand, 128>& ext_cmd_q,
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

        // --- 1. (EL1008) ---
        uint8_t raw_input = *(domain_pd + off_el1008_in);
        
        // Log IN: if see 00000001 (not pressed), that is NC button.
        // NC Button logic: 1 = (SAFE), 0 = (EMG)
        bool is_system_safe = holly_bitset::test(raw_input, holly_bitset::Bit::DO0);

        // --- 2. CLIENT (RobotCommand) ---
        auto cmd = c_q.pop();
        if (cmd) {
            last_client_cmd = cmd->set_outputs;
            last_id = cmd->cmd_id;
        }

        // --- 2b. Extended GRS Commands (Motion/Wait) ---
        auto ext_cmd = ext_cmd_q.pop();
        if (ext_cmd) {
            last_id = ext_cmd->cmd_id;
            
            // Handle I/O commands via set_outputs
            if (ext_cmd->cmd_type == GRS_CMD_OUTPUT || 
                ext_cmd->cmd_type == GRS_CMD_SET_ALL_OUTPUTS) {
                // Update output byte from extended command
                if (ext_cmd->cmd_type == GRS_CMD_OUTPUT) {
                    if (ext_cmd->io_value)
                        last_client_cmd |= (1u << ext_cmd->io_index);
                    else
                        last_client_cmd &= ~(1u << ext_cmd->io_index);
                } else {
                    last_client_cmd = ext_cmd->set_outputs;
                }
            }
            
            // Log motion commands (PTP/LIN/CIRC etc.)
            if (ext_cmd->cmd_type >= GRS_CMD_PTP && 
                ext_cmd->cmd_type <= GRS_CMD_SPLINE_REL) {
                logMotionTarget(*ext_cmd);
            }
        }

        // --- 3. priority ---
        uint8_t final_output = 0;

        if (is_system_safe) {
            // if it is just safety, client said it happened
            final_output = last_client_cmd;
        } else {
            // if button  is pressed, output: 0!
            final_output = 0;
            // Not: last_client_cmd  is not zero, when safety is fixed, and client says ON, it can continues 
        }

        // --- 4. (EL2008) ---
        *(domain_pd + off_el2008_out) = final_output;

        // --- 5. feedback ---
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

/*
#include <iostream>
#include <chrono>
#include <ecrt.h>
#include "protocol.hpp"
#include "spsc_queue.hpp"

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
                  std::atomic<bool>& run) 
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

    uint8_t client_requested_val = 0; 
    uint64_t last_id = 0;

    std::cout << "[RT] Bridge Active. Monitoring Hardware Interrupts..." << std::endl;

    while (run) {
        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= 1000000000) {
            wakeup_time.tv_sec++;
            wakeup_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        uint8_t raw_input = *(domain_pd + off_el1008_in);
        
        // --- KRİTİK NOKTA: Butonun Durumu ---
        // Eğer butona basmadığında IN:1 görüyorsan, 'is_button_pressed' (0. bitin 0 olmasıdır)
        // Aşağıdaki test fonksiyonu 0. bitin 1 olup olmadığına bakar.
        bool bit0_is_high = holly_bitset::test(raw_input, holly_bitset::Bit::DO0);

        auto cmd = c_q.pop();
        if (cmd) {
            client_requested_val = cmd->set_outputs;
            last_id = cmd->cmd_id;
        }

        uint8_t final_output = 0;

        bool is_safe = bit0_is_high; 
        

        if (is_safe) {
            final_output = client_requested_val;
        } else {
            final_output = 0;
        }

        *(domain_pd + off_el2008_out) = final_output;

        RobotState st{};
        st.seq_id = last_id;
        st.timestamp = (uint32_t)wakeup_time.tv_nsec;
        st.inputs = raw_input;
        st.outputs = final_output;
        st.is_hardware_emg = !is_safe;
        st.system_ready = is_safe;
        
        s_q.push(st);

        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }
    ecrt_release_master(master);
}*/
