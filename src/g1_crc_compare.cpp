#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using LowCmd = unitree_hg::msg::dds_::LowCmd_;

// Python reference result
constexpr uint32_t EXPECTED_CRC = 0x4E5447CF;

// G1 HG LowCmd contains 35 motor commands
constexpr int MOTOR_COUNT = 35;

// HG LowCmd packed size
constexpr std::size_t PACKED_SIZE = 1004;

// =====================================================
// Append one byte
// =====================================================

void AppendU8(
    std::vector<uint8_t>& data,
    uint8_t value)
{
    data.push_back(value);
}

// =====================================================
// Append 32-bit integer in little-endian order
// =====================================================

void AppendU32(
    std::vector<uint8_t>& data,
    uint32_t value)
{
    for (int i = 0; i < 4; ++i) {
        data.push_back(
            static_cast<uint8_t>(
                (value >> (8 * i)) & 0xFF
            )
        );
    }
}

// =====================================================
// Append float32 in little-endian order
// =====================================================

void AppendFloat(
    std::vector<uint8_t>& data,
    float value)
{
    static_assert(sizeof(float) == 4);

    uint32_t bits = 0;

    std::memcpy(
        &bits,
        &value,
        sizeof(float)
    );

    AppendU32(data, bits);
}

// =====================================================
// Pack HG LowCmd
//
// Matches Python:
// '<2B2x' + 'B3x5fI' * 35 + '5I'
// =====================================================

std::vector<uint8_t> PackLowCmd(
    const LowCmd& command)
{
    std::vector<uint8_t> data;

    data.reserve(PACKED_SIZE);

    AppendU8(data, command.mode_pr());
    AppendU8(data, command.mode_machine());

    // 2-byte padding
    AppendU8(data, 0);
    AppendU8(data, 0);

    for (int i = 0; i < MOTOR_COUNT; ++i) {

        const auto& motor =
            command.motor_cmd().at(i);

        AppendU8(data, motor.mode());

        // 3-byte padding
        AppendU8(data, 0);
        AppendU8(data, 0);
        AppendU8(data, 0);

        AppendFloat(data, motor.q());
        AppendFloat(data, motor.dq());
        AppendFloat(data, motor.tau());
        AppendFloat(data, motor.kp());
        AppendFloat(data, motor.kd());

        AppendU32(data, motor.reserve());
    }

    // 4 reserve uint32 fields
    for (const auto value : command.reserve()) {
        AppendU32(data, value);
    }

    // CRC field
    AppendU32(data, command.crc());

    return data;
}

// =====================================================
// Calculate CRC32 over all words except CRC field
// =====================================================

uint32_t CalculateCRC(
    const std::vector<uint8_t>& bytes)
{
    if (bytes.size() != PACKED_SIZE) {
        throw std::runtime_error(
            "Unexpected LowCmd packed size"
        );
    }

    constexpr std::size_t WORD_COUNT =
        PACKED_SIZE / 4 - 1;

    std::array<uint32_t, WORD_COUNT> words{};

    for (std::size_t i = 0; i < WORD_COUNT; ++i) {

        std::size_t p = 4 * i;

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
// Main: offline test only
// =====================================================

int main()
{
    std::cout
        << "\n====================================\n"
        << " G1 Python / C++ CRC32 Comparison\n"
        << "====================================\n"
        << "OFFLINE TEST ONLY\n"
        << "No robot connection.\n"
        << "No motor commands.\n"
        << "====================================\n\n";

    // Create LowCmd with default values
    LowCmd command{};

    // Exactly match the Python test
    command.mode_pr(0);
    command.mode_machine(5);

    auto& motor15 =
        command.motor_cmd().at(15);

    motor15.mode(1);
    motor15.q(0.25f);
    motor15.dq(0.0f);
    motor15.tau(0.0f);
    motor15.kp(80.0f);
    motor15.kd(3.0f);

    // G1 ArmSDK test weight
    command.motor_cmd().at(29).q(0.5f);

    // Initial CRC = 0
    command.crc(0);

    // Explicitly pack all fields
    const auto bytes = PackLowCmd(command);

    // Calculate CRC
    const uint32_t cpp_crc =
        CalculateCRC(bytes);

    std::cout << std::fixed
              << std::setprecision(6);

    std::cout
        << "mode_pr: "
        << static_cast<int>(command.mode_pr())
        << "\n"
        << "mode_machine: "
        << static_cast<int>(command.mode_machine())
        << "\n"
        << "Motor 15 mode: "
        << static_cast<int>(motor15.mode())
        << "\n"
        << "Motor 15 q: "
        << motor15.q()
        << "\n"
        << "Motor 15 kp: "
        << motor15.kp()
        << "\n"
        << "Motor 15 kd: "
        << motor15.kd()
        << "\n"
        << "Weight: "
        << command.motor_cmd().at(29).q()
        << "\n"
        << "Packed size: "
        << bytes.size()
        << " bytes\n";

    std::cout
        << "\nPython CRC32: 0x"
        << std::uppercase
        << std::hex
        << std::setw(8)
        << std::setfill('0')
        << EXPECTED_CRC
        << "\n";

    std::cout
        << "C++ CRC32:    0x"
        << std::setw(8)
        << cpp_crc
        << std::dec
        << "\n";

    const bool match =
        (cpp_crc == EXPECTED_CRC);

    std::cout
        << "\n====================================\n"
        << "CRC MATCH: "
        << (match ? "YES - PASS" : "NO - FAIL")
        << "\n"
        << "====================================\n";

    return match ? 0 : 1;
}
