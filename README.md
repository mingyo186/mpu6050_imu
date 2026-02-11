# mpu6050_imu

ROS 2 Jazzy driver for the InvenSense MPU6050 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) over I2C.

## Features

- Publishes `sensor_msgs/Imu` on the `imu/data_raw` topic
- **Fake mode** — generates random IMU data without physical hardware (great for development and testing)
- Configurable I2C bus, device address, publish rate, full-scale ranges, and covariance values
- Burst-read of all 6 axes in a single I2C transaction for consistency
- Proper SI unit output: acceleration in m/s², angular velocity in rad/s

## Prerequisites

- ROS 2 Jazzy
- Python 3
- `smbus2` (only required when `fake_mode` is `false`)

```bash
pip3 install smbus2
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select mpu6050_imu
source install/setup.bash
```

## Usage

### Launch with default parameters (fake mode)

```bash
ros2 launch mpu6050_imu mpu6050_launch.py
```

### Run the node directly

```bash
ros2 run mpu6050_imu mpu6050_node.py --ros-args -p fake_mode:=true
```

### Run with real hardware

```bash
ros2 run mpu6050_imu mpu6050_node.py --ros-args -p fake_mode:=false
```

### Override parameters via YAML

```bash
ros2 launch mpu6050_imu mpu6050_launch.py params_file:=/path/to/your_params.yaml
```

### Verify output

```bash
ros2 topic echo /imu/data_raw
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fake_mode` | bool | `true` | `true`: generate random IMU data, `false`: read from real I2C device |
| `i2c_bus` | int | `1` | I2C bus number (`/dev/i2c-N`) |
| `device_address` | int | `0x68` | MPU6050 I2C address (`0x68` or `0x69`) |
| `publish_rate` | double | `50.0` | Publishing rate in Hz |
| `frame_id` | string | `imu_link` | TF frame ID in the Imu message header |
| `accel_range` | int | `0` | Accelerometer full-scale: `0`=±2g, `1`=±4g, `2`=±8g, `3`=±16g |
| `gyro_range` | int | `0` | Gyroscope full-scale: `0`=±250, `1`=±500, `2`=±1000, `3`=±2000 °/s |
| `accel_covariance` | double | `0.04` | Diagonal acceleration covariance (m²/s⁴) |
| `gyro_covariance` | double | `0.02` | Diagonal angular velocity covariance (rad²/s²) |

## Services

| Service | Type | Description |
|---|---|---|
| `imu/calibrate` | `std_srvs/Trigger` | Collect gyro data for 2 seconds and compute bias offset |
| `imu/reset` | `std_srvs/Trigger` | Clear bias and reinitialize the sensor |

## Package Structure

```
mpu6050_imu/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── mpu6050_params.yaml
├── launch/
│   └── mpu6050_launch.py
├── mpu6050_imu/
│   ├── __init__.py
│   └── mpu6050_driver.py
├── nodes/
│   └── mpu6050_node.py
└── test/
    └── test_mpu6050_node.py
```

## Test Results

Tested on Ubuntu 24.04 (WSL2) with `fake_mode: true`.

```
$ colcon test --packages-select mpu6050_imu
$ colcon test-result --verbose
Summary: 27 tests, 0 errors, 0 failures, 0 skipped
```

| Test Category | Test | Result |
|---|---|---|
| **Topics** | `imu/data_raw` publishes `sensor_msgs/Imu` | PASS |
| **Topics** | `frame_id == "imu_link"` | PASS |
| **Topics** | `orientation_covariance[0] == -1.0` (no fusion) | PASS |
| **Services** | `imu/calibrate` returns `success=True` | PASS |
| **Services** | `imu/reset` returns `success=True` | PASS |
| **Parameters** | `publish_rate` runtime change to 20.0 Hz | PASS |
| **Shutdown** | Clean exit (code 0, -2, or -15) | PASS |
| **Linting** | pep257, flake8, copyright, xmllint | PASS |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
