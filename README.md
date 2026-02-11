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
└── nodes/
    └── mpu6050_node.py
```

## TODO

- [x] BMP280 — barometric pressure + temperature sensor
- [x] BNO055 — 9-axis absolute orientation IMU
- [x] HMC5883L — 3-axis digital compass
- [ ] VL53L0X — ToF (Time-of-Flight) distance sensor
- [ ] ADS1115 — 16-bit ADC for analog sensors

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
