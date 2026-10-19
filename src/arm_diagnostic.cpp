#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

using namespace unitree::robot;

using LowState = unitree_hg::msg::dds_::LowState_;

constexpr int ARM_START = 15;
constexpr int ARM_COUNT = 14;

std::mutex state_mutex;
LowState latest_state;
bool received = false;

std::chrono::steady_clock::time_point last_update;

void StateCallback(const void* msg)
{
    if (!msg) return;

    std::lock_guard<std::mutex> lock(state_mutex);

    latest_state = *static_cast<const LowState*>(msg);
    last_update = std::chrono::steady_clock::now();

    received = true;
}

bool ReadState(LowState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex);

    if (!received) return false;

    auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_update
    ).count();

    if (age > 0.25) return false;

    state = latest_state;

    return true;
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr
            << "Usage: "
            << argv[0]
            << " <network_interface>\n";
        return 1;
    }

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

        if (ReadState(state)) {
            connected = true;
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    if (!connected) {
        std::cerr << "No fresh state received.\n";
        return 1;
    }

    std::array<double, ARM_COUNT> initial_q{};

    for (int i = 0; i < ARM_COUNT; ++i) {
        initial_q[i] =
            state.motor_state().at(ARM_START + i).q();
    }

    std::cout << "\nG1 Passive Arm Diagnostic\n";
    std::cout << "No motor commands will be sent.\n";
    std::cout << "Monitoring for 10 seconds.\n\n";

    double overall_max_error = 0.0;
    int overall_max_motor = -1;

    for (int step = 0; step < 100; ++step) {

        if (!ReadState(state)) {
            std::cerr << "State timeout.\n";
            return 1;
        }

        double max_error = 0.0;
        double max_velocity = 0.0;

        int max_motor = -1;
        double max_motor_q = 0.0;
        double max_motor_dq = 0.0;

        for (int i = 0; i < ARM_COUNT; ++i) {

            int id = ARM_START + i;

            double q =
                state.motor_state().at(id).q();

            double dq =
                state.motor_state().at(id).dq();

            if (!std::isfinite(q) ||
                !std::isfinite(dq)) {

                std::cerr << "Invalid joint state.\n";
                return 1;
            }

            double error =
                std::abs(q - initial_q[i]);

            if (error > max_error) {
                max_error = error;
                max_motor = id;
                max_motor_q = q;
                max_motor_dq = dq;
            }

            max_velocity =
                std::max(max_velocity, std::abs(dq));
        }

        if (max_error > overall_max_error) {
            overall_max_error = max_error;
            overall_max_motor = max_motor;
        }

        if (step % 10 == 0) {

            std::cout << std::fixed
                      << std::setprecision(5);

            std::cout
                << "Time: " << step * 0.1 << " s"
                << " | Max deviation: "
                << max_error << " rad"
                << " | Motor: " << max_motor
                << " | q: " << max_motor_q
                << " | dq: " << max_motor_dq
                << " | Max |dq|: " << max_velocity
                << " rad/s\n";
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    std::cout
        << "\n========== SUMMARY ==========\n"
        << "Maximum deviation: "
        << overall_max_error << " rad\n"
        << "Motor index: "
        << overall_max_motor << "\n"
        << "=============================\n";

    return 0;
}
