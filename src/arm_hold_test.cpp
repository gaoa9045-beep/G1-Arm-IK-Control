#include <unitree/dds_wrapper/common/crc.h>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

using namespace unitree::robot;

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;

// =====================================================
// G1 configuration
// =====================================================

constexpr int MOTOR_COUNT = 35;

constexpr int ARM_START = 15;
constexpr int ARM_COUNT = 14;

constexpr int WEIGHT_INDEX = 29;

// =====================================================
// Simulation parameters
// =====================================================

constexpr double CONTROL_DT = 0.004;  // 250 Hz

constexpr double ACQUIRE_TIME = 3.0;
constexpr double HOLD_TIME = 3.0;
constexpr double RELEASE_TIME = 3.0;

constexpr double TOTAL_TIME =
    ACQUIRE_TIME + HOLD_TIME + RELEASE_TIME;

// =====================================================
// Diagnostic thresholds
// =====================================================

constexpr double MAX_STATE_AGE = 0.25;

// Passive monitoring threshold.
// Not a validated hardware safety limit.
constexpr double MAX_PASSIVE_DEVIATION = 0.02;

// =====================================================
// Shared robot state
// =====================================================

std::mutex state_mutex;

LowState latest_state{};

bool state_received = false;

std::chrono::steady_clock::time_point last_state_time;

// Signal-safe stop flag
volatile std::sig_atomic_t stop_requested = 0;

// =====================================================
// Ctrl+C handler
// =====================================================

void SignalHandler(int)
{
    stop_requested = 1;
}

// =====================================================
// DDS state callback
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

    state_received = true;
}

// =====================================================
// Read fresh robot state
// =====================================================

bool GetState(LowState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex);

    if (!state_received) {
        return false;
    }

    const double age =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now()
            - last_state_time
        ).count();

    if (age > MAX_STATE_AGE) {
        return false;
    }

    state = latest_state;

    return true;
}

// =====================================================
// G1 joint classification
// =====================================================

bool IsWristJoint(int id)
{
    return (id >= 19 && id <= 21) ||
           (id >= 26 && id <= 28);
}

bool IsWeakMotor(int id)
{
    return id == 4 ||
           id == 10 ||
           (id >= 15 && id <= 18) ||
           (id >= 22 && id <= 25);
}

// =====================================================
// Initialize offline LowCmd
// =====================================================

bool InitializeCommand(
    LowCmd& command,
    const LowState& state)
{
    command.mode_pr(0);

    command.mode_machine(
        state.mode_machine()
    );

    // Construct candidate command fields
    // for the G1's 29 physical joints.
    //
    // This message must NOT be published.
    // Reserved slots remain at default values.

    for (int id = 0; id < 29; ++id) {

        const double q =
            state.motor_state().at(id).q();

        if (!std::isfinite(q)) {
            std::cerr
                << "Invalid motor position: "
                << id << "\n";

            return false;
        }

        auto& motor =
            command.motor_cmd().at(id);

        motor.mode(1);

        motor.q(static_cast<float>(q));

        motor.dq(0.0f);

        motor.tau(0.0f);

        if (IsWristJoint(id)) {

            motor.kp(40.0f);
            motor.kd(1.5f);

        } else if (IsWeakMotor(id)) {

            motor.kp(80.0f);
            motor.kd(3.0f);

        } else {

            motor.kp(300.0f);
            motor.kd(3.0f);
        }
    }

    // G1 ArmSDK weight slot
    // Only an offline simulation variable

    command.motor_cmd()
        .at(WEIGHT_INDEX)
        .q(0.0f);

    return true;
}

// =====================================================
// CRC32 calculation
//
// Explicit packing matches Python HG LowCmd:
//
// '<2B2x' + 'B3x5fI' * 35 + '5I'
//
// Offline verification only.
// =====================================================

void AppendU32(
    std::array<uint8_t, 1004>& bytes,
    std::size_t& pos,
    uint32_t value)
{
    for (int i = 0; i < 4; ++i) {

        bytes.at(pos++) =
            static_cast<uint8_t>(
                (value >> (8 * i)) & 0xFF
            );
    }
}

void AppendFloat(
    std::array<uint8_t, 1004>& bytes,
    std::size_t& pos,
    float value)
{
    uint32_t bits = 0;

    static_assert(sizeof(float) == 4);

    std::memcpy(
        &bits,
        &value,
        sizeof(float)
    );

    AppendU32(bytes, pos, bits);
}

uint32_t CalculateCRC(const LowCmd& command)
{
    std::array<uint8_t, 1004> bytes{};

    std::size_t pos = 0;

    // mode_pr, mode_machine and padding

    bytes.at(pos++) = command.mode_pr();

    bytes.at(pos++) = command.mode_machine();

    bytes.at(pos++) = 0;
    bytes.at(pos++) = 0;

    // 35 motor commands

    for (int id = 0; id < MOTOR_COUNT; ++id) {

        const auto& motor =
            command.motor_cmd().at(id);

        bytes.at(pos++) = motor.mode();

        bytes.at(pos++) = 0;
        bytes.at(pos++) = 0;
        bytes.at(pos++) = 0;

        AppendFloat(bytes, pos, motor.q());
        AppendFloat(bytes, pos, motor.dq());
        AppendFloat(bytes, pos, motor.tau());
        AppendFloat(bytes, pos, motor.kp());
        AppendFloat(bytes, pos, motor.kd());

        AppendU32(
            bytes,
            pos,
            motor.reserve()
        );
    }

    // Reserved uint32 fields

    for (const auto value : command.reserve()) {

        AppendU32(
            bytes,
            pos,
            value
        );
    }

    // Final CRC field is not part of CRC input
    AppendU32(bytes, pos, command.crc());

    if (pos != bytes.size()) {

        std::cerr
            << "ERROR: Incorrect command packing.\n";

        return 0;
    }

    constexpr std::size_t WORD_COUNT = 250;

    std::array<uint32_t, WORD_COUNT> words{};

    for (std::size_t i = 0; i < WORD_COUNT; ++i) {

        const std::size_t p = i * 4;

        words[i] =
            static_cast<uint32_t>(bytes[p])
            | (static_cast<uint32_t>(bytes[p + 1]) << 8)
            | (static_cast<uint32_t>(bytes[p + 2]) << 16)
            | (static_cast<uint32_t>(bytes[p + 3]) << 24);
    }

    return crc32_core(
        words.data(),
        static_cast<uint32_t>(WORD_COUNT)
    );
}

// =====================================================
// Simulated control weight
// =====================================================

double CalculateWeight(double t)
{
    if (t < ACQUIRE_TIME) {

        return std::clamp(
            t / ACQUIRE_TIME,
            0.0,
            1.0
        );
    }

    if (t < ACQUIRE_TIME + HOLD_TIME) {

        return 1.0;
    }

    if (t < TOTAL_TIME) {

        const double release_t =
            t - ACQUIRE_TIME - HOLD_TIME;

        return std::clamp(
            1.0 - release_t / RELEASE_TIME,
            0.0,
            1.0
        );
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
            << "Usage: "
            << argv[0]
            << " <network_interface>\n";

        return 1;
    }

    std::signal(SIGINT, SignalHandler);

    std::cout
        << "\n====================================\n"
        << " G1 ArmSDK Hold Test V2\n"
        << "====================================\n"
        << "READ-ONLY DRY RUN\n"
        << "No motor commands will be sent.\n"
        << "No control publisher is created.\n"
        << "====================================\n";

    // Initialize DDS for state subscription only

    ChannelFactory::Instance()->Init(
        0,
        argv[1]
    );

    auto subscriber =
        std::make_shared<ChannelSubscriber<LowState>>(
            "rt/lowstate"
        );

    subscriber->InitChannel(StateCallback, 1);

    std::cout << "\nWaiting for G1 state...\n";

    LowState state{};

    bool connected = false;

    for (int i = 0; i < 100; ++i) {

        if (stop_requested) {
            return 1;
        }

        if (GetState(state)) {

            connected = true;
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    if (!connected) {

        std::cerr
            << "ERROR: Robot communication failed.\n";

        return 1;
    }

    // Build offline command

    LowCmd command{};

    if (!InitializeCommand(command, state)) {

        return 1;
    }

    // Save initial arm angles

    std::array<double, ARM_COUNT> initial_q{};

    for (int i = 0; i < ARM_COUNT; ++i) {

        initial_q[i] =
            state.motor_state()
                .at(ARM_START + i).q();
    }

    const int initial_mode_machine =
        static_cast<int>(state.mode_machine());

    std::cout
        << "\nRobot mode_machine: "
        << initial_mode_machine
        << "\n";

    std::cout
        << "Command mode_machine: "
        << static_cast<int>(command.mode_machine())
        << "\n";

    std::cout
        << "\nStarting simulated control sequence...\n\n";

    const int total_steps =
        static_cast<int>(TOTAL_TIME / CONTROL_DT);

    double overall_max_error = 0.0;
    int overall_max_motor = -1;

    auto next_tick =
        std::chrono::steady_clock::now();

    // =================================================
    // Main diagnostic loop
    // =================================================

    for (int step = 0;
         step <= total_steps;
         ++step) {

        if (stop_requested) {

            std::cout
                << "\nStopped by user.\n"
                << "No motor commands were sent.\n";

            return 1;
        }

        if (!GetState(state)) {

            std::cerr
                << "\nERROR: State timeout.\n";

            return 1;
        }

        if (static_cast<int>(state.mode_machine()) !=
            initial_mode_machine) {

            std::cerr
                << "\nERROR: mode_machine changed.\n";

            return 1;
        }

        const double t =
            step * CONTROL_DT;

        // Simulated weight; never transmitted

        const double weight =
            CalculateWeight(t);

        command.motor_cmd()
            .at(WEIGHT_INDEX)
            .q(static_cast<float>(weight));

        // Recalculate offline CRC

        command.crc(0);

        const uint32_t crc =
            CalculateCRC(command);

        command.crc(crc);

        // Monitor arm joints

        double maximum_error = 0.0;
        double maximum_velocity = 0.0;

        int maximum_error_motor = -1;

        for (int i = 0; i < ARM_COUNT; ++i) {

            const int motor_id =
                ARM_START + i;

            const double actual_q =
                state.motor_state()
                    .at(motor_id).q();

            const double actual_dq =
                state.motor_state()
                    .at(motor_id).dq();

            if (!std::isfinite(actual_q) ||
                !std::isfinite(actual_dq)) {

                std::cerr
                    << "\nERROR: Invalid motor state.\n"
                    << "Motor: "
                    << motor_id << "\n";

                return 1;
            }

            const double error =
                std::abs(
                    actual_q - initial_q[i]
                );

            maximum_velocity =
                std::max(
                    maximum_velocity,
                    std::abs(actual_dq)
                );

            if (error > maximum_error) {

                maximum_error = error;

                maximum_error_motor = motor_id;
            }
        }

        if (maximum_error > overall_max_error) {

            overall_max_error =
                maximum_error;

            overall_max_motor =
                maximum_error_motor;
        }

        // Print every 0.5 seconds

        if (step % 125 == 0 ||
            step == total_steps) {

            const char* phase = "RELEASE";

            if (t < ACQUIRE_TIME) {

                phase = "ACQUIRE";

            } else if (
                t < ACQUIRE_TIME + HOLD_TIME) {

                phase = "HOLD";
            }

            std::cout
                << std::fixed
                << std::setprecision(5)
                << "t=" << t
                << " | phase=" << phase
                << " | simulated_weight=" << weight
                << " | max_error=" << maximum_error
                << " rad"
                << " | motor=" << maximum_error_motor
                << " | max_dq=" << maximum_velocity
                << " rad/s"
                << "\n";
        }

        // Passive motion diagnostic

        if (maximum_error >
            MAX_PASSIVE_DEVIATION) {

            std::cerr
                << "\nWARNING: Passive arm deviation.\n"
                << "Motor: "
                << maximum_error_motor << "\n"
                << "Deviation: "
                << maximum_error << " rad\n";

            return 1;
        }

        next_tick +=
            std::chrono::milliseconds(4);

        std::this_thread::sleep_until(
            next_tick
        );
    }

    // =================================================
    // Summary
    // =================================================

    std::cout
        << "\n========== V2 DRY RUN SUMMARY ==========\n"
        << "Mode machine: "
        << initial_mode_machine << "\n"
        << "Simulated final weight: "
        << CalculateWeight(TOTAL_TIME) << "\n"
        << "Maximum passive deviation: "
        << overall_max_error << " rad\n"
        << "Motor: "
        << overall_max_motor << "\n"
        << "Control frequency: 250 Hz\n"
        << "Offline CRC calculation: COMPLETED\n"
        << "CRC protocol compatibility: NOT PROVEN\n"
        << "Commands published: 0\n"
        << "========================================\n";

    std::cout
        << "\nG1 ArmSDK V2 dry run finished.\n";

    return 0;
}
