# RPLIDAR A2M12 MAVLink Worker

Companion-computer worker that turns raw scans from a Slamtec RPLIDAR A2M12 into MAVLink `OBSTACLE_DISTANCE` messages for PX4/ArduPilot object avoidance pipelines. The refactor focuses on production-ready ergonomics: configurable runtime, deterministic signal handling, and clear operational docs for your portfolio.

## Highlights
- **Pure MAVSDK flow** – runs as `ComponentType::CompanionComputer`, auto-discovers the target system, and streams via MAVLink passthrough.
- **Deterministic scan compression** – converts up to 8192 HQ samples into 72 evenly spaced bins with angle increment metadata.
- **Runtime configuration** – tune connection URIs, IDs, serial ports, baud rates, and distance clamps without touching code.
- **Graceful shutdown** – POSIX signals flip an atomic flag, stop the motor, and close the serial session without undefined behavior.

## Architecture
```
┌─────────────┐    ┌────────────────────┐    ┌────────────────────────┐    ┌─────────────────────────┐
│ RPLIDAR     │ -> │ Slamtec SDK driver │ -> │ Worker (72-bin compressor) │ -> │ MAVSDK passthrough (GCS) │
└─────────────┘    └────────────────────┘    └────────────────────────┘    └─────────────────────────┘
```
1. `sl_lidar` yields HQ scan nodes.
2. Worker converts Slamtec's Q2 millimetres to centimetres via $d_{cm} = \frac{d_{mm\_q2}}{40}$ and clamps them.
3. MAVSDK queues `OBSTACLE_DISTANCE` using `queue_message`, so PX4/ArduPilot can consume it alongside companion telemetry.

## Getting Started

### 1. Clone (with submodule)
```bash
git clone https://github.com/batuhangorkey/lidar-a2m12-mavlink-worker.git
cd lidar-a2m12-mavlink-worker
git submodule update --init --recursive   # pulls Slamtec SDK into rplidar_sdk/
```

### 2. Install prerequisites
- CMake ≥ 3.10
- A C++17 toolchain (GCC, Clang, or MSVC)
- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html) with development headers
- Slamtec RPLIDAR A2M12 + USB/UART adapter

### 3. Configure & build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### 4. Run
```bash
./build/a2m12_mavlink_worker \
	--mavlink-uri udpout://127.0.0.1:14550 \
	--lidar-port /dev/ttyUSB0 \
	--system-id 1 --component-id 196 --target-system-id 255 \
	--min-distance 20 --max-distance 500
```

Keep MAVSDK's backend (usually `mavsdk_server`) reachable at the URI you provide. When running against SITL on the same machine, `udpout://127.0.0.1:14550` is enough; for Wi-Fi companion links, point to the autopilot's IP.

## Configuration Flags

| Flag | Description | Default |
| --- | --- | --- |
| `--mavlink-uri` | MAVLink connection string passed to MAVSDK | `udpout://127.0.0.1:14550` |
| `--system-id` / `--component-id` | IDs advertised by the worker | `1` / `MAV_COMP_ID_USER1` |
| `--target-system-id` | System ID to look for (set to autopilot or GCS) | `255` |
| `--lidar-port` / `--baudrate` | Serial port path and baudrate for RPLIDAR | `/dev/ttyUSB0` / `256000` |
| `--min-distance` / `--max-distance` | Clamp values for `OBSTACLE_DISTANCE` (cm) | `0` / `500` |
| `-h`, `--help` | Show CLI usage | — |

Flags are validated (e.g., `min_distance` automatically swaps with `max_distance` if provided backwards) and are safe to script for different vehicles.

## Runtime Behavior
- **Signal-aware loop** – `SIGINT`, `SIGTERM`, and `SIGQUIT` set an atomic flag. The worker finishes the current iteration, sends a final stop command to the motor, and releases the driver cleanly.
- **Adaptive GCS discovery** – it patiently retries `wait_for_system()` until a system with the requested ID appears, so launching before PX4 boots no longer drops data.
- **Measurement compression** – the loop snapshots 8192 HQ nodes, sorts them via `ascendScanData`, and slices them into 72 equal-angle bins with `kAngleIncrement = 5°` before publishing.
- **Error reporting** – driver, serial, and MAVLink errors surface through human-readable `std::runtime_error` messages that bubble up to `stderr`.

## Development Notes
- `.clang-format` (LLVM style) is included—`clang-format -i main.cpp` keeps patches tidy.
- `CMakeLists.txt` already links against `MAVSDK::mavsdk` plus the static Slamtec library located at `rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a`. Adjust the path if you build the SDK elsewhere.
- `utils.h` keeps tiny sleep/count helpers for future modules; feel free to replace with `<chrono>` helpers if you expand the codebase.

## Portfolio-Friendly Roadmap
1. **Unit-test the compression logic** using captured node dumps to prove determinism.
2. **Add telemetry publishing** (e.g., `STATUSTEXT` heartbeat) so the autopilot can report worker faults to the GCS HUD.
3. **Expose health metrics** over REST/gRPC for fleet observability.
4. **Containerize** the worker with the Slamtec SDK and MAVSDK baked in for quick redeploys on companion computers.

These extensions make for great talking points when showcasing the repo—each maps cleanly to a real-world deliverable (better validation, observability, deployability).
