#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <chrono>
#include <cmath>
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

constexpr int MOTOR_COUNT = 35;

constexpr int LEFT_ARM_START = 15;
constexpr int RIGHT_ARM_END = 28;

constexpr int WEIGHT_INDEX = 29;

constexpr double MAX_STATE_AGE = 0.25;

std::mutex state_mutex;

LowState latest_state{};

bool state_received = false;

std::chrono::steady_clock::time_point last_state_time;

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
// Read a fresh robot state
// =====================================================

bool GetState(LowState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex);

    if (!state_received) {
        return false;
    }

    double age =
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
// Classify arm joint
// =====================================================

bool IsArmJoint(int id)
{
    return id >= LEFT_ARM_START &&
           id <= RIGHT_ARM_END;
}

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
// CRC calculation candidate
//
// Based on the installed H2 SDK example.
// Offline comparison only.
// Not yet validated for G1 arm_sdk firmware.
// =====================================================

uint32_t CalculateCRC32Candidate(const LowCmd& command)
{
    static_assert(
        sizeof(LowCmd) % sizeof(uint32_t) == 0,
        "LowCmd size must be divisible by 4"
    );

    constexpr std::size_t WORD_COUNT =
        sizeof(LowCmd) / sizeof(uint32_t);

    static_assert(
        WORD_COUNT > 1,
        "LowCmd is unexpectedly small"
    );

    std::array<uint32_t, WORD_COUNT> words{};

    std::memcpy(
        words.data(),
        &command,
        sizeof(LowCmd)
    );

    return crc32_core(
        words.data(),
        static_cast<uint32_t>(WORD_COUNT - 1)
    );
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

    std::cout
        << "\n====================================\n"
        << " G1 ArmSDK Command Dry Run\n"
        << "====================================\n"
        << "READ-ONLY / OFFLINE COMMAND CHECK\n"
        << "No control publisher is created.\n"
        << "No motor commands will be sent.\n"
        << "====================================\n\n";

    // Initialize DDS for subscribing only
    ChannelFactory::Instance()->Init(0, argv[1]);

    auto subscriber =
        std::make_shared<ChannelSubscriber<LowState>>(
            "rt/lowstate"
        );

    subscriber->InitChannel(StateCallback, 1);

    std::cout << "Waiting for G1 state...\n";

    LowState state{};

    bool connected = false;

    for (int i = 0; i < 100; ++i) {

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
            << "ERROR: No fresh robot state.\n";

        return 1;
    }

    // =================================================
    // Read robot information
    // =================================================

    std::cout
        << "\nRobot status:\n"
        << "mode_pr = "
        << static_cast<int>(state.mode_pr())
        << "\n"
        << "mode_machine = "
        << static_cast<int>(state.mode_machine())
        << "\n";

    // =================================================
    // Construct an offline LowCmd object
    // =================================================

    LowCmd command{};

    command.mode_pr(0);

    command.mode_machine(
        state.mode_machine()
    );

    // Initialize motor fields from measured state.
    // These are reference values for offline checking.
    // Do not publish this command to a real robot.

    for (int id = 0; id < MOTOR_COUNT; ++id) {

        const double q =
            state.motor_state().at(id).q();

        if (!std::isfinite(q)) {

            std::cerr
                << "ERROR: Invalid motor q at index "
                << id << "\n";

            return 1;
        }

        auto& motor =
            command.motor_cmd().at(id);

        motor.mode(1);
        motor.q(static_cast<float>(q));
        motor.dq(0.0f);
        motor.tau(0.0f);

        // Reference gains from G1_29_ArmController
        // For offline command-layout inspection only.

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

    // ArmSDK weight slot for G1.
    // Set to zero because this is an offline check.

    command.motor_cmd()
        .at(WEIGHT_INDEX)
        .q(0.0f);

    // =================================================
    // Print command fields
    // =================================================

    std::cout
        << "\nConstructed command:\n"
        << "mode_pr = "
        << static_cast<int>(command.mode_pr())
        << "\n"
        << "mode_machine = "
        << static_cast<int>(command.mode_machine())
        << "\n"
        << "weight index = "
        << WEIGHT_INDEX
        << "\n"
        << "weight = "
        << command.motor_cmd().at(WEIGHT_INDEX).q()
        << "\n";

    std::cout
        << "\nArm joint command values:\n";

    std::cout
        << std::fixed
        << std::setprecision(6);

    for (int id = LEFT_ARM_START;
         id <= RIGHT_ARM_END;
         ++id) {

        const auto& motor =
            command.motor_cmd().at(id);

        std::cout
            << "Motor " << id
            << " | mode="
            << static_cast<int>(motor.mode())
            << " | q=" << motor.q()
            << " | kp=" << motor.kp()
            << " | kd=" << motor.kd()
            << "\n";
    }

    // =================================================
    // CRC candidate calculation
    // =================================================

    command.crc(0);

    uint32_t crc_candidate =
        CalculateCRC32Candidate(command);

    command.crc(crc_candidate);

    std::cout
        << "\nCRC calculation:\n"
        << "LowCmd size = "
        << sizeof(LowCmd)
        << " bytes\n"
        << "CRC32 candidate = 0x"
        << std::hex
        << std::uppercase
        << crc_candidate
        << std::dec
        << "\n";

    // =================================================
    // Safety confirmation
    // =================================================

    std::cout
        << "\n====================================\n"
        << " DRY RUN SUMMARY\n"
        << "====================================\n"
        << "G1 state received: YES\n"
        << "Offline LowCmd constructed: YES\n"
        << "CRC candidate calculated: YES\n"
        << "CRC compatibility verified: NO\n"
        << "ArmSDK publisher created: NO\n"
        << "Motor commands published: 0\n"
        << "====================================\n";

    std::cout
        << "\nDry run finished.\n";

    return 0;
}
