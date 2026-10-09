#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;

// Shared robot state
std::mutex state_mutex;
LowState latest_state;
std::atomic<bool> received_state{false};

// DDS callback: receive robot state
void StateCallback(const void* message)
{
    if (message == nullptr) {
        return;
    }

    const auto* state =
        static_cast<const LowState*>(message);

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        latest_state = *state;
    }

    received_state.store(true);
}

// Print joint positions
void PrintArm(
    const LowState& state,
    const std::string& arm_name,
    const std::vector<int>& indices,
    const std::vector<std::string>& names)
{
    std::cout << "\n" << arm_name << ":\n";

    for (size_t i = 0; i < indices.size(); ++i) {

        int index = indices[i];

        float q = state.motor_state().at(index).q();

        std::cout
            << std::left << std::setw(18) << names[i]
            << " : "
            << std::fixed << std::setprecision(4)
            << q << " rad\n";
    }
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr
            << "Usage: " << argv[0]
            << " <network_interface> <arm_dof: 5|7>\n";
        return 1;
    }

    std::string network_interface = argv[1];

    int arm_dof = 0;

    try {
        arm_dof = std::stoi(argv[2]);
    } catch (...) {
        std::cerr << "Invalid arm_dof.\n";
        return 1;
    }

    if (arm_dof != 5 && arm_dof != 7) {
        std::cerr << "arm_dof must be 5 or 7.\n";
        return 1;
    }

    std::cout << "G1 Arm State Reader\n";
    std::cout << "Network: " << network_interface << "\n";
    std::cout << "Arm DOF: " << arm_dof << "\n";

    // Initialize DDS communication
    ChannelFactory::Instance()->Init(
        0, network_interface
    );

    // Subscribe to robot state
    auto subscriber =
        std::make_shared<ChannelSubscriber<LowState>>(
            "rt/lowstate"
        );

    subscriber->InitChannel(StateCallback, 1);

    // Joint names
    std::vector<std::string> names;

    if (arm_dof == 5) {
        names = {
            "Shoulder Pitch",
            "Shoulder Roll",
            "Shoulder Yaw",
            "Elbow Pitch",
            "Elbow Roll"
        };
    } else {
        names = {
            "Shoulder Pitch",
            "Shoulder Roll",
            "Shoulder Yaw",
            "Elbow",
            "Wrist Roll",
            "Wrist Pitch",
            "Wrist Yaw"
        };
    }

    // Joint indices from Unitree G1 SDK examples
    std::vector<int> left_indices;
    std::vector<int> right_indices;

    for (int i = 0; i < arm_dof; ++i) {
        left_indices.push_back(15 + i);
        right_indices.push_back(22 + i);
    }

    std::cout << "\nWaiting for robot state...\n";

    // Wait up to 10 seconds for first message
    for (int i = 0; i < 100; ++i) {
        if (received_state.load()) {
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    if (!received_state.load()) {
        std::cerr
            << "ERROR: No robot state received.\n";
        return 1;
    }

    std::cout << "Robot connected!\n";

    // Read and display 10 snapshots
    for (int count = 0; count < 10; ++count) {

        LowState snapshot;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            snapshot = latest_state;
        }

        std::cout
            << "\n========== G1 ARM STATUS ==========\n";

        PrintArm(
            snapshot,
            "LEFT ARM",
            left_indices,
            names
        );

        PrintArm(
            snapshot,
            "RIGHT ARM",
            right_indices,
            names
        );

        std::cout
            << "===================================\n";

        std::this_thread::sleep_for(
            std::chrono::seconds(1)
        );
    }

    std::cout << "\nState reading finished.\n";

    return 0;
}
