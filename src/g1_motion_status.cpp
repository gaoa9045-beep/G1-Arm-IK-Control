#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <cstdint>
#include <iostream>
#include <string>

using namespace unitree::robot;
using namespace unitree::robot::b2;

int main(int argc, char** argv)
{
    // =====================================================
    // Check arguments
    // =====================================================

    if (argc != 2) {

        std::cerr
            << "Usage: "
            << argv[0]
            << " <network_interface>\n";

        return 1;
    }

    std::cout << "\n";
    std::cout << "====================================\n";
    std::cout << " G1 Motion Service Status Reader\n";
    std::cout << "====================================\n";
    std::cout << "QUERY ONLY\n";
    std::cout << "No motor commands will be sent.\n";
    std::cout << "No motion service will be changed.\n";
    std::cout << "====================================\n\n";

    // =====================================================
    // Initialize DDS
    // =====================================================

    ChannelFactory::Instance()->Init(0, argv[1]);

    // =====================================================
    // Initialize MotionSwitcherClient
    // =====================================================

    MotionSwitcherClient switcher;

    switcher.SetTimeout(5.0F);

    switcher.Init();

    // =====================================================
    // Query motion control service
    // =====================================================

    std::string robotForm;

    std::string motionName;

    std::cout << "Querying G1 motion service...\n\n";

    int32_t ret = switcher.CheckMode(
        robotForm,
        motionName
    );

    // =====================================================
    // Check result
    // =====================================================

    if (ret != 0) {

        std::cerr
            << "CheckMode FAILED.\n"
            << "Error code: "
            << ret << "\n";

        return 1;
    }

    std::cout << "CheckMode succeeded.\n\n";

    std::cout << "Robot form: "
              << robotForm << "\n";

    std::cout << "Motion name: "
              << (motionName.empty() ?
                  "<empty>" : motionName)
              << "\n\n";

    // =====================================================
    // Interpret result
    // =====================================================

    if (motionName.empty()) {

        std::cout
            << "Motion control service: "
            << "DEACTIVATED\n";

    } else {

        std::cout
            << "Motion control service: "
            << "ACTIVE\n";

        if (robotForm == "0") {

            if (motionName == "normal") {

                std::cout
                    << "Service name: sport_mode\n";

            } else if (motionName == "ai") {

                std::cout
                    << "Service name: ai_sport\n";

            } else if (motionName == "advanced") {

                std::cout
                    << "Service name: advanced_sport\n";

            } else {

                std::cout
                    << "Service name: "
                    << "Unknown mapping\n";
            }

        } else {

            std::cout
                << "Robot form: "
                << robotForm
                << "\n";
        }
    }

    std::cout << "\n";
    std::cout << "====================================\n";
    std::cout << "QUERY COMPLETED\n";
    std::cout << "No motion commands sent.\n";
    std::cout << "====================================\n";

    return 0;
}
