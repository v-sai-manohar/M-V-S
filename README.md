# EdgeAnalytics

Real-time edge analytics system for Raspberry Pi 4B on QNX Neutrino (aarch64le).
The project reads environmental and motion data, computes 2-second window analytics, and serves live telemetry through an HTTP dashboard and API.

## Highlights

- QNX multi-threaded architecture with deterministic priorities
- DHT22 sensor support (temperature and humidity via GPIO17)
- MPU6050 sensor support (accelerometer and gyroscope via I2C1)
- Lock-free SPSC ring buffers between sensor and analytics tasks
- 2-second window analytics with threshold-based alert flags
- HTTP dashboard and telemetry API on port 8080
- Automatic fallback to simulation mode when hardware preflight fails

## Runtime Pipeline

1. Sensor threads acquire DHT22 and MPU6050 data.
2. Analytics thread drains buffers every 2 seconds and computes stats.
3. Transmission thread receives analytics messages via QNX IPC.
4. HTTP server exposes dashboard, telemetry JSON, and health endpoint.

Thread spawn order:

1. HTTP server
2. Transmission
3. Analytics
4. DHT22 sensor
5. MPU6050 sensor

## Endpoints

- Dashboard: /
- Telemetry API: /api/telemetry
- Health: /health

Default URL:

- http://<PI_IP>:8080/

## Hardware Setup

Target board:

- Raspberry Pi 4B

Sensor wiring:

- DHT22 DATA -> GPIO17 (physical pin 11) with 10k pull-up to 3.3V
- MPU6050 SDA -> GPIO2 (physical pin 3)
- MPU6050 SCL -> GPIO3 (physical pin 5)
- Sensor VCC -> 3.3V
- Sensor GND -> GND
- MPU6050 address: 0x68 (default) or 0x69 (AD0 high)

## Build

Prerequisites:

- QNX SDP 8.0 toolchain (qcc)
- QNX Momentics or shell with QNX environment loaded

From repository root:

    make clean
    make -j8 all

Output binary:

- edge_analytics_aarch64le

## Run

Hardware mode:

    ./edge_analytics_aarch64le

Simulation mode:

    ./edge_analytics_aarch64le --simulate

Note:

- If /dev/i2c1 is inaccessible or I/O privilege is missing, the app logs a warning and switches to simulation mode.

## Deployment to Raspberry Pi

Keep binary and web assets together:

    scp edge_analytics_aarch64le root@<PI_IP>:/tmp/edge_analytics/
    scp -r web root@<PI_IP>:/tmp/edge_analytics/
    ssh root@<PI_IP> "cd /tmp/edge_analytics && chmod +x edge_analytics_aarch64le && ./edge_analytics_aarch64le"

## Repository Structure

- src/ : Core C source files
- web/ : Dashboard HTML assets
- objs/ : Build object output by architecture
- Submission_Package/ : Submission bundle (source_code, supporting_files, presentation)
- Makefile : QNX build configuration
- README_PS34.md : Hardware-focused build and diagnostic notes
- EdgeSense_Report_v2.md : Detailed project report

## Troubleshooting

- MPU6050 not detected:
  - Verify I2C manager is started and /dev/i2c1 exists.
  - Check AD0 wiring (0x68 vs 0x69).

- DHT22 read failures:
  - Confirm GPIO17 wiring and pull-up resistor.
  - Ensure stable 3.3V and common ground.

- Permission errors:
  - Run as root or with io_priv to allow ThreadCtl(_NTO_TCTL_IO).

## License

This repository is intended for academic and educational use unless otherwise specified by the project owner.
