#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

using namespace unitree::robot;

using LowState = unitree_hg::msg::dds_::LowState_;

std::mutex state_mutex;

LowState latest_state;

bool state_received = false;

// =====================================================
// DDS callback
// =====================================================

void StateCallback(const void* message)
{
    if (message == nullptr) {
        return;
    }

    const auto* state =
        static_cast<const LowState*>(message);

    std::lock_guard<std::mutex> lock(state_mutex);

    latest_state = *state;
    state_received = true;
}

// =====================================================
// Main
// =====================================================

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr
            << "Usage: " << argv[0]
            << " <network_interface>\n";

        return 1;
    }

    std::cout << "\nG1 Robot Mode Reader\n";
    std::cout << "============================\n";
    std::cout << "READ-ONLY MODE\n";
    std::cout << "No motor commands will be sent.\n";
    std::cout << "============================\n\n";

    ChannelFactory::Instance()->Init(0, argv[1]);

    auto subscriber =
        std::make_shared<ChannelSubscriber<LowState>>(
            "rt/lowstate"
        );

    subscriber->InitChannel(StateCallback, 1);

    std::cout << "Waiting for G1 state...\n";

    bool connected = false;

    for (int i = 0; i < 100; ++i) {

        {
            std::lock_guard<std::mutex> lock(state_mutex);

            if (state_received) {
                connected = true;
            }
        }

        if (connected) {
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    if (!connected) {
        std::cerr << "ERROR: No G1 state received.\n";
        return 1;
    }

    std::cout << "\nReading robot mode for 5 seconds...\n\n";

    for (int i = 0; i < 5; ++i) {

        LowState state;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state = latest_state;
        }

        std::cout
            << "Time: " << i << " s\n"
            << "mode_pr: "
            << static_cast<int>(state.mode_pr()) << "\n"
            << "mode_machine: "
            << static_cast<int>(state.mode_machine()) << "\n"
            << "----------------------------\n";

        std::this_thread::sleep_for(
            std::chrono::seconds(1)
        );
    }

    std::cout << "\nMode reading finished.\n";

    return 0;
}
