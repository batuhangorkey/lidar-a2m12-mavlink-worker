#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "mavsdk/mavlink_include.h"
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"
#include "mavsdk/system.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace mavsdk;
using namespace sl;

namespace {
constexpr size_t kObstacleBinCount = 72;
constexpr uint8_t kAngleIncrementDeg = static_cast<uint8_t>(360u / kObstacleBinCount);
constexpr float kDistanceScale = 1.0f / 40.0f;
const std::chrono::milliseconds kSystemPollInterval{500};

std::atomic_bool g_should_exit{false};

struct WorkerConfig {
    std::string mavlink_uri = "udpout://127.0.0.1:14550";
    uint8_t system_id = 1;
    uint8_t component_id = MAV_COMP_ID_USER1;
    uint8_t target_system_id = 255;
    std::string lidar_port = "/dev/ttyUSB0";
    uint32_t lidar_baudrate = 256000;
    uint16_t min_distance_cm = 0;
    uint16_t max_distance_cm = 500;
};

void print_usage(const std::string &binary) {
    std::cout
        << "Usage: " << binary << " [options]\n\n"
        << "Options:\n"
        << "  --mavlink-uri <uri>       MAVLink connection string (default "
           "udpout://127.0.0.1:14550)\n"
        << "  --system-id <id>          System ID advertised to the MAVLink network (default 1)\n"
        << "  --component-id <id>       Component ID advertised to the MAVLink network (default "
           "MAV_COMP_ID_USER1)\n"
        << "  --target-system-id <id>   System ID of the target GCS/autopilot (default 255)\n"
        << "  --lidar-port <path>       Serial device path for the RPLIDAR (default /dev/ttyUSB0)\n"
        << "  --baudrate <value>        Serial baudrate for the RPLIDAR (default 256000)\n"
        << "  --min-distance <cm>       Minimum distance (cm) encoded into obstacle_distance "
           "(default 0)\n"
        << "  --max-distance <cm>       Maximum distance (cm) encoded into obstacle_distance "
           "(default 500)\n"
        << "  -h, --help                Show this help message and exit\n";
}

WorkerConfig parse_arguments(int argc, char *argv[]) {
    WorkerConfig config;
    auto require_value = [&](int &index, const std::string &flag) -> std::string {
        if (index + 1 >= argc) {
            std::ostringstream oss;
            oss << "Flag " << flag << " requires a value";
            throw std::invalid_argument(oss.str());
        }
        return argv[++index];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--mavlink-uri") {
            config.mavlink_uri = require_value(i, arg);
        } else if (arg == "--system-id") {
            config.system_id = static_cast<uint8_t>(std::stoi(require_value(i, arg)));
        } else if (arg == "--component-id") {
            config.component_id = static_cast<uint8_t>(std::stoi(require_value(i, arg)));
        } else if (arg == "--target-system-id") {
            config.target_system_id = static_cast<uint8_t>(std::stoi(require_value(i, arg)));
        } else if (arg == "--lidar-port") {
            config.lidar_port = require_value(i, arg);
        } else if (arg == "--baudrate") {
            config.lidar_baudrate = static_cast<uint32_t>(std::stoul(require_value(i, arg)));
        } else if (arg == "--min-distance") {
            config.min_distance_cm = static_cast<uint16_t>(std::stoi(require_value(i, arg)));
        } else if (arg == "--max-distance") {
            config.max_distance_cm = static_cast<uint16_t>(std::stoi(require_value(i, arg)));
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            std::ostringstream oss;
            oss << "Unknown argument: " << arg;
            throw std::invalid_argument(oss.str());
        }
    }

    if (config.min_distance_cm > config.max_distance_cm) {
        std::swap(config.min_distance_cm, config.max_distance_cm);
    }

    return config;
}

void handle_signal(int) {
    std::cerr << "\nSignal received. Shutting down...\n";
    g_should_exit.store(true);
}

uint64_t steady_time_us() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

std::shared_ptr<System>
wait_for_system(Mavsdk &client, uint8_t target_system_id, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!g_should_exit.load()) {
        for (auto &system : client.systems()) {
            if (system == nullptr) {
                continue;
            }
            if (target_system_id == 0 || system->get_system_id() == target_system_id) {
                return system;
            }
        }

        if (timeout.count() >= 0 && std::chrono::steady_clock::now() >= deadline) {
            break;
        }

        std::this_thread::sleep_for(kSystemPollInterval);
    }

    return nullptr;
}

std::array<uint16_t, kObstacleBinCount> compress_scan(
    const sl_lidar_response_measurement_node_hq_t *nodes, size_t count, uint16_t min_distance,
    uint16_t max_distance) {
    std::array<uint16_t, kObstacleBinCount> distances{};
    if (count == 0) {
        distances.fill(max_distance);
        return distances;
    }

    const size_t stride = std::max<size_t>(count / kObstacleBinCount, static_cast<size_t>(1));
    for (size_t i = 0; i < kObstacleBinCount; ++i) {
        const size_t index = std::min(i * stride, count - 1);
        const auto reading_cm = static_cast<uint16_t>(nodes[index].dist_mm_q2 * kDistanceScale);
        distances[i] = std::clamp<uint16_t>(reading_cm, min_distance, max_distance);
    }

    return distances;
}

void log_config(const WorkerConfig &config) {
    std::cout << "----- Worker configuration -----\n"
              << "MAVLink URI       : " << config.mavlink_uri << '\n'
              << "System ID         : " << static_cast<int>(config.system_id) << '\n'
              << "Component ID      : " << static_cast<int>(config.component_id) << '\n'
              << "Target system ID  : " << static_cast<int>(config.target_system_id) << '\n'
              << "LIDAR port        : " << config.lidar_port << '\n'
              << "LIDAR baudrate    : " << config.lidar_baudrate << '\n'
              << "Distance clamp    : [" << config.min_distance_cm << ", " << config.max_distance_cm
              << "] cm\n"
              << "--------------------------------\n";
}

} // namespace

int main(int argc, char *argv[]) {
    try {
        const WorkerConfig config = parse_arguments(argc, argv);

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
#if defined(SIGQUIT)
        std::signal(SIGQUIT, handle_signal);
#endif

        auto sdk_config = Mavsdk::Configuration(ComponentType::CompanionComputer);
        sdk_config.set_always_send_heartbeats(true);
        sdk_config.set_system_id(config.system_id);
        sdk_config.set_component_id(config.component_id);

        Mavsdk mavlink_client(sdk_config);

        const ConnectionResult connection_result =
            mavlink_client.add_any_connection(config.mavlink_uri);
        if (connection_result != ConnectionResult::Success) {
            std::ostringstream oss;
            oss << "Connection failed: " << connection_result_str(connection_result);
            throw std::runtime_error(oss.str());
        }

        log_config(config);

        auto gcs =
            wait_for_system(mavlink_client, config.target_system_id, std::chrono::seconds(5));
        if (!gcs) {
            std::cerr << "Warning: no system " << static_cast<int>(config.target_system_id)
                      << " detected yet. Will keep trying while scanning.\n";
        } else {
            std::cout << "Found system with ID " << static_cast<int>(gcs->get_system_id()) << '\n';
        }

        std::unique_ptr<MavlinkPassthrough> mavlink_passthrough;
        if (gcs) {
            mavlink_passthrough = std::make_unique<MavlinkPassthrough>(gcs);
        }

        auto lidar_result = createLidarDriver();
        auto lidar = *lidar_result;
        if (lidar == nullptr) {
            throw std::runtime_error("Failed to allocate LIDAR driver");
        }

        auto channel_result =
            createSerialPortChannel(config.lidar_port.c_str(), config.lidar_baudrate);
        auto channel = *channel_result;
        if (channel == nullptr) {
            std::ostringstream oss;
            oss << "Failed to open serial channel on " << config.lidar_port;
            throw std::runtime_error(oss.str());
        }

        std::cout << "Connecting to LIDAR on " << config.lidar_port << '\n';
        sl_result lidar_connect_result = lidar->connect(channel);
        if (!SL_IS_OK(lidar_connect_result)) {
            std::ostringstream oss;
            oss << "Failed to connect to LIDAR, error: 0x" << std::hex << lidar_connect_result;
            throw std::runtime_error(oss.str());
        }

        lidar->stop();
        lidar->setMotorSpeed(0);

        sl_lidar_response_device_info_t device_info{};
        lidar_connect_result = lidar->getDeviceInfo(device_info);
        if (!SL_IS_OK(lidar_connect_result)) {
            std::ostringstream oss;
            oss << "Failed to read device info, error: 0x" << std::hex << lidar_connect_result;
            throw std::runtime_error(oss.str());
        }

        std::cout << "Device info -> Model: " << static_cast<int>(device_info.model)
                  << " Firmware: " << (device_info.firmware_version >> 8) << '.'
                  << (device_info.firmware_version & 0xffu)
                  << " Hardware: " << static_cast<int>(device_info.hardware_version) << '\n';

        LidarScanMode scan_mode{};
        lidar->setMotorSpeed();
        lidar_connect_result = lidar->startScan(false, true, 0, &scan_mode);
        if (!SL_IS_OK(lidar_connect_result)) {
            throw std::runtime_error("Failed to start scan");
        }

        while (!g_should_exit.load()) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = sizeof(nodes) / sizeof(nodes[0]);
            sl_result grab_result = lidar->grabScanDataHq(nodes, count);
            if (!SL_IS_OK(grab_result)) {
                std::cerr << "grabScanDataHq failed with error 0x" << std::hex << grab_result
                          << std::dec << '\n';
                continue;
            }

            lidar->ascendScanData(nodes, count);
            const auto distances =
                compress_scan(nodes, count, config.min_distance_cm, config.max_distance_cm);

            if (!mavlink_passthrough) {
                gcs = wait_for_system(
                    mavlink_client, config.target_system_id, std::chrono::seconds(0));
                if (gcs) {
                    std::cout << "GCS discovered (system ID "
                              << static_cast<int>(gcs->get_system_id()) << ")\n";
                    mavlink_passthrough = std::make_unique<MavlinkPassthrough>(gcs);
                } else {
                    continue;
                }
            }

            const uint64_t timestamp_us = steady_time_us();
            const auto passthrough_result = mavlink_passthrough->queue_message(
                [&distances, timestamp_us, &config](MavlinkAddress address, uint8_t channel_id) {
                    mavlink_message_t message;
                    mavlink_msg_obstacle_distance_pack_chan(
                        address.system_id,
                        address.component_id,
                        channel_id,
                        &message,
                        timestamp_us,
                        MAV_DISTANCE_SENSOR_LASER,
                        distances.data(),
                        kAngleIncrementDeg,
                        config.min_distance_cm,
                        config.max_distance_cm,
                        0,
                        0,
                        MAV_FRAME_BODY_FRD);
                    return message;
                });

            if (passthrough_result != MavlinkPassthrough::Result::Success) {
                std::cerr << "Failed to queue obstacle_distance message. Result code: "
                          << static_cast<int>(passthrough_result) << '\n';
            }
        }

        std::cout << "Stopping scan and shutting down.\n";
        lidar->stop();
        lidar->setMotorSpeed(0);

        delete lidar;
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return -1;
    }
}