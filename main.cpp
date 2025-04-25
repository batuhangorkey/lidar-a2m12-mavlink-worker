#include <rplidar.h>
#include <signal.h>
#include <stdio.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "mavsdk/mavlink_include.h"
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"
#include "mavsdk/system.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "utils.h"

using namespace mavsdk;
using namespace sl;

static auto lidar = *createLidarDriver();
static bool ctrl_c_pressed = false;

void ctrlc(int sig) {
    fprintf(stderr, "\nctrl-c pressed, exiting...\n");
    ctrl_c_pressed = true;
}

int main(int argc, char *argv[]) {
    // first arg is lidar, second is mavlink
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <lidar_port> <mavlink_port>\n", argv[0]);
        return -1;
    }

    std::string lidar_port = argv[1];
    std::string mavlink_port = argv[2];

    fprintf(stderr, "Lidar port: %s\n", lidar_port.c_str());
    fprintf(stderr, "Mavlink port: %s\n", mavlink_port.c_str());

    if (signal(SIGTERM, ctrlc) == SIG_ERR) {
        fprintf(stderr, "Error setting signal handler\n");
        return -1;
    }

    auto config = Mavsdk::Configuration(ComponentType::CompanionComputer);
    config.set_always_send_heartbeats(true);
    config.set_system_id(1);
    config.set_component_id(MAV_COMP_ID_USER1);
    Mavsdk mavlinkClient(config);
    // auto result = mavlinkClient.add_any_connection("udpout://172.20.128.1:14550");
    auto result = mavlinkClient.add_any_connection(mavlink_port);
    if (result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << result << std::endl;
        return -1;
    }

    sleep_s(3);
    fprintf(stderr, "Found %zu system(s)\n", mavlinkClient.systems().size());

    std::shared_ptr<System> gcs;
    for (auto &system : mavlinkClient.systems()) {
        std::cout << "System found with ID: " << static_cast<int>(system->get_system_id())
                  << std::endl;
        if (system->get_system_id() == 255) {
            gcs = system;
            break;
        }
    }

    if (gcs == nullptr) {
        std::cerr << "No GCS found" << std::endl;
        return -1;
    }
    std::cout << "Found GCS with ID: " << static_cast<int>(gcs->get_system_id()) << std::endl;

    auto mavlinkPassthrough = MavlinkPassthrough(*gcs);

    ///  Create a communication channel instance
    // auto channel = createSerialPortChannel("/dev/ttyUSB0", 256000);
    auto channel = createSerialPortChannel(lidar_port.c_str(), 256000);

    ///  Create a LIDAR driver instance
    auto res = lidar->connect(*channel);
    if (!SL_IS_OK(res)) {
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
        return -1;
    }

    sl_lidar_response_device_info_t deviceInfo;
    res = lidar->getDeviceInfo(deviceInfo);
    if (SL_IS_OK(res)) {
        printf(
            "Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            deviceInfo.model,
            deviceInfo.firmware_version >> 8,
            deviceInfo.firmware_version & 0xffu,
            deviceInfo.hardware_version);
    } else {
        fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        return -1;
    }

    LidarScanMode scanMode;
    fprintf(stderr, "Starting scan...\n");

    lidar->setMotorSpeed();
    lidar->startScan(false, true, 0, &scanMode);

    sl_result op_result;

    const int max_samples = 72;
    const uint8_t increment = 360 / max_samples;

    while (!ctrl_c_pressed) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        op_result = lidar->grabScanDataHq(nodes, count);

        const int step = count / 72;
        uint16_t distances[max_samples] = {0};

        if (SL_IS_OK(op_result)) {
            lidar->ascendScanData(nodes, count);

            for (int pos = 0; pos < max_samples; pos++) {
                distances[pos] = nodes[pos * step].dist_mm_q2 / 10.0f / 4.0f;
            }
            auto time_usec =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
            auto res = mavlinkPassthrough.queue_message(
                [&distances, &time_usec](MavlinkAddress address, auto channel) {
                    mavlink_message_t message;
                    mavlink_msg_obstacle_distance_pack_chan(
                        address.system_id,
                        address.component_id,
                        channel,
                        &message,
                        time_usec,
                        MAV_DISTANCE_SENSOR_LASER,
                        distances,
                        increment,
                        0,
                        500,
                        0,
                        0,
                        MAV_FRAME_BODY_FRD);
                    return message;
                });
            std::cout << "Sending message: " << res << std::endl;
            // for (int pos = 0; pos < (int)count; ++pos) {
            //   auto dist_cm = nodes[pos].dist_mm_q2 * 10.0f / 4.0f;

            //   printf("%s theta: %03.2f Dist (cm): %08.2f Q: %d \n",
            //          (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "
            //          ", (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
            //          nodes[pos].dist_mm_q2 * 10.0f / 4.0f,
            //          nodes[pos].quality >>
            //          SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            // }
        }
    }

    fprintf(stderr, "Scan stopped.\n");

    lidar->stop();
    lidar->setMotorSpeed(0);

    delete lidar;
    return 0;
}