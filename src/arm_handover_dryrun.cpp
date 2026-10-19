#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace unitree::robot;

using LowState = unitree_hg::msg::dds_::LowState_;

constexpr int ARM_START = 15;
constexpr int ARM_COUNT = 14;

// Simulation parameters
constexpr double CONTROL_DT = 0.02;
constexpr double ACQUIRE_TIME = 3.0;
constexpr double HOLD_TIME = 3.0;
constexpr double RELEASE_TIME = 3.0;

// Diagnostic parameters
constexpr double MAX_STATE_AGE = 0.25;
constexpr double MAX_PASSIVE_DEVIATION = 0.02;

std::mutex state_mutex;
LowState latest_state;

bool received_state = false;

std::chrono::steady_clock::time_point last_state_time;

std::atomic<bool> stop_requested{false};

const std::array<std::string, ARM_COUNT> joint_names = {
    "Left Shoulder Pitch",
    "Left Shoulder Roll",
    "Left Shoulder Yaw",
    "Left Elbow",
    "Left Wrist Roll",
    "Left Wrist Pitch",
    "Left Wrist Yaw",
    "Right Shoulder Pitch",
    "Right Shoulder Roll",
    "Right Shoulder Yaw",
    "Right Elbow",
    "Right Wrist Roll",
    "Right Wrist Pitch",
    "Right Wrist Yaw"
};

// =====================================================
// Signal handler
// =====================================================

void SignalHandler(int)
{
    stop_requested.store(true);
}

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

    last_state_time =
        std::chrono::steady_clock::now();

    received_state = true;
}

// =====================================================
// Read robot state
// =====================================================

bool ReadState(LowState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex);

    if (!received_state) {
        return false;
    }

    double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() -
        last_state_time
    ).count();

    if (age > MAX_STATE_AGE) {
        return false;
    }

    state = latest_state;

    return true;
}

// =====================================================
// Weight trajectory - simulation only
// =====================================================

double CalculateWeight(double t)
{
    if (t < ACQUIRE_TIME) {
        return t / ACQUIRE_TIME;
    }

    if (t < ACQUIRE_TIME + HOLD_TIME) {
        return 1.0;
    }

    if (t < ACQUIRE_TIME + HOLD_TIME +
                RELEASE_TIME) {

        double release_t =
            t - ACQUIRE_TIME - HOLD_TIME;

        return 1.0 -
               release_t / RELEASE_TIME;
    }

    return 0.0;
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

    std::signal(SIGINT, SignalHandler);

    std::cout << "\nG1 ArmSDK Handover Dry Run\n";
    std::cout << "================================\n";
    std::cout << "Mode: READ-ONLY\n";
    std::cout << "No motor commands will be sent.\n";
    std::cout << "================================\n\n";

    // DDS initialization
    ChannelFactory::Instance()->Init(0, argv[1]);

    auto subscriber =
        std::make_shared<ChannelSubscriber<LowState>>(
            "rt/lowstate"
        );

    subscriber->InitChannel(StateCallback, 1);

    std::cout << "Waiting for G1 state...\n";

    LowState state;

    bool connected = false;

    for (int i = 0; i < 100; ++i) {

        if (stop_requested.load()) {
            return 1;
        }

        if (ReadState(state)) {
            connected = true;
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    if (!connected) {
        std::cerr << "ERROR: No fresh G1 state.\n";
        return 1;
    }

    // Save initial joint positions
    std::array<double, ARM_COUNT> initial_q{};

    for (int i = 0; i < ARM_COUNT; ++i) {

        double q =
            state.motor_state().at(ARM_START + i).q();

        if (!std::isfinite(q)) {
            std::cerr << "Invalid initial joint.\n";
            return 1;
        }

        initial_q[i] = q;
    }

    std::cout << "Initial joint positions:\n";

    for (int i = 0; i < ARM_COUNT; ++i) {
        std::cout
            << ARM_START + i << " "
            << joint_names[i] << ": "
            << initial_q[i] << " rad\n";
    }

    std::cout
        << "\nStarting simulated handover...\n";

    const double total_time =
        ACQUIRE_TIME + HOLD_TIME + RELEASE_TIME;

    const int total_steps =
        static_cast<int>(total_time / CONTROL_DT);

    double overall_max_error = 0.0;
    int overall_max_motor = -1;

    auto next_tick = std::chrono::steady_clock::now();

    for (int step = 0; step <= total_steps; ++step) {

        if (stop_requested.load()) {
            std::cout << "\nStopped by user.\n";
            return 1;
        }

        if (!ReadState(state)) {
            std::cerr
                << "\nERROR: State communication timeout.\n";
            return 1;
        }

        double t = step * CONTROL_DT;

        // This weight is calculated but NOT published
        double simulated_weight =
            std::clamp(CalculateWeight(t), 0.0, 1.0);

        double max_error = 0.0;
        int max_motor = -1;
        double actual_q = 0.0;
        double actual_dq = 0.0;

        for (int i = 0; i < ARM_COUNT; ++i) {

            int motor_id = ARM_START + i;

            double q =
                state.motor_state().at(motor_id).q();

            double dq =
                state.motor_state().at(motor_id).dq();

            if (!std::isfinite(q) ||
                !std::isfinite(dq)) {

                std::cerr
                    << "\nERROR: Invalid motor state: "
                    << motor_id << "\n";

                return 1;
            }

            double error =
                std::abs(q - initial_q[i]);

            if (error > max_error) {

                max_error = error;
                max_motor = motor_id;
                actual_q = q;
                actual_dq = dq;
            }
        }

        if (max_error > overall_max_error) {

            overall_max_error = max_error;
            overall_max_motor = max_motor;
        }

        // Display every 0.5 seconds
        if (step % 25 == 0) {

            std::string phase;

            if (t < ACQUIRE_TIME)
                phase = "SIMULATED ACQUIRE";
            else if (t < ACQUIRE_TIME + HOLD_TIME)
                phase = "SIMULATED HOLD";
            else
                phase = "SIMULATED RELEASE";

            std::cout
                << std::fixed
                << std::setprecision(5)
                << "t=" << t
                << " s | " << phase
                << " | weight=" << simulated_weight
                << " | max_error=" << max_error
                << " rad | motor=" << max_motor
                << " | q=" << actual_q
                << " | dq=" << actual_dq
                << "\n";
        }

        // Diagnostic-only threshold
        if (max_error > MAX_PASSIVE_DEVIATION) {

            std::cerr
                << "\nWARNING: Unexpected passive motion.\n"
                << "Motor: " << max_motor << "\n"
                << "Deviation: " << max_error
                << " rad\n";

            return 1;
        }

        next_tick += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(next_tick);
    }

    std::cout
        << "\n========== DRY RUN SUMMARY ==========\n"
        << "Simulated final weight: "
        << CalculateWeight(total_time) << "\n"
        << "Maximum passive deviation: "
        << overall_max_error << " rad\n"
        << "Motor: "
        << overall_max_motor << "\n"
        << "Commands published: 0\n"
        << "=====================================\n";

    std::cout << "\nDry run finished.\n";

    return 0;
}
