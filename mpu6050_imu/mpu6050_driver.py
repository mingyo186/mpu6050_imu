# Copyright 2025 The mpu6050_imu Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""MPU6050 I2C Driver - 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)."""

import math
import random
import struct


class FakeMPU6050Driver:
    """Fake driver that generates random IMU data without I2C hardware."""

    GRAVITY = 9.80665

    def __init__(self, **kwargs):
        """Initialize fake driver (no hardware needed)."""

    def read_all(self):
        """Read fake accelerometer, gyroscope, and temperature data."""
        ax = random.gauss(0.0, 0.3)
        ay = random.gauss(0.0, 0.3)
        az = self.GRAVITY + random.gauss(0.0, 0.3)

        gx = random.gauss(0.0, 0.01)
        gy = random.gauss(0.0, 0.01)
        gz = random.gauss(0.0, 0.01)

        temp = 25.0 + random.gauss(0.0, 0.5)

        return (ax, ay, az), (gx, gy, gz), temp

    def who_am_i(self) -> int:
        """Return the default MPU6050 chip ID."""
        return 0x68

    def close(self):
        """Close fake driver (no-op)."""


class MPU6050Driver:
    """Low-level I2C driver for InvenSense MPU6050."""

    # Register Map
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B  # 6 bytes: AX, AY, AZ (H/L pairs)
    TEMP_OUT_H = 0x41  # 2 bytes
    GYRO_XOUT_H = 0x43  # 6 bytes: GX, GY, GZ (H/L pairs)
    PWR_MGMT_1 = 0x6B
    WHO_AM_I = 0x75

    # Full-Scale Range Tables
    #  range setting -> (register bits, LSB-per-unit)
    ACCEL_RANGES = {
        0: (0x00, 16384.0),  # +/-2g
        1: (0x08, 8192.0),   # +/-4g
        2: (0x10, 4096.0),   # +/-8g
        3: (0x18, 2048.0),   # +/-16g
    }
    GYRO_RANGES = {
        0: (0x00, 131.0),    # +/-250 deg/s
        1: (0x08, 65.5),     # +/-500 deg/s
        2: (0x10, 32.8),     # +/-1000 deg/s
        3: (0x18, 16.4),     # +/-2000 deg/s
    }

    GRAVITY = 9.80665            # m/s^2
    DEG_TO_RAD = math.pi / 180.0

    def __init__(self, bus: int = 1, address: int = 0x68,
                 accel_range: int = 0, gyro_range: int = 0):
        """Initialize MPU6050 driver on the given I2C bus and address."""
        from smbus2 import SMBus

        self.address = address
        self.bus = SMBus(bus)

        self._accel_scale = self.ACCEL_RANGES[accel_range][1]
        self._gyro_scale = self.GYRO_RANGES[gyro_range][1]

        self._init_device(accel_range, gyro_range)

    def _init_device(self, accel_range: int, gyro_range: int):
        """Configure MPU6050 registers for sampling and full-scale ranges."""
        # Wake up (clear SLEEP bit), use internal 8 MHz oscillator
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        # Sample-rate divider: Fs / (1 + SMPLRT_DIV) -> 1 kHz / 8 = 125 Hz
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x07)

        # DLPF bandwidth ~44 Hz (CONFIG register bits [2:0] = 3)
        self.bus.write_byte_data(self.address, self.CONFIG, 0x03)

        # Full-scale ranges
        self.bus.write_byte_data(
            self.address, self.ACCEL_CONFIG, self.ACCEL_RANGES[accel_range][0])
        self.bus.write_byte_data(
            self.address, self.GYRO_CONFIG, self.GYRO_RANGES[gyro_range][0])

    def _read_raw_block(self, reg: int, length: int) -> bytes:
        """Read *length* bytes starting at *reg* in a single I2C transaction."""
        return bytes(self.bus.read_i2c_block_data(self.address, reg, length))

    @staticmethod
    def _unpack_int16(data: bytes, offset: int) -> int:
        """Unpack a big-endian signed 16-bit integer."""
        return struct.unpack_from('>h', data, offset)[0]

    def read_all(self):
        """Read accel, temp, and gyro in one 14-byte burst from 0x3B."""
        buf = self._read_raw_block(self.ACCEL_XOUT_H, 14)

        ax = self._unpack_int16(buf, 0) / self._accel_scale * self.GRAVITY
        ay = self._unpack_int16(buf, 2) / self._accel_scale * self.GRAVITY
        az = self._unpack_int16(buf, 4) / self._accel_scale * self.GRAVITY

        temp_raw = self._unpack_int16(buf, 6)
        temp = temp_raw / 340.0 + 36.53

        gx = self._unpack_int16(buf, 8) / self._gyro_scale * self.DEG_TO_RAD
        gy = self._unpack_int16(buf, 10) / self._gyro_scale * self.DEG_TO_RAD
        gz = self._unpack_int16(buf, 12) / self._gyro_scale * self.DEG_TO_RAD

        return (ax, ay, az), (gx, gy, gz), temp

    def who_am_i(self) -> int:
        """Read WHO_AM_I register (should return 0x68 for genuine MPU6050)."""
        return self.bus.read_byte_data(self.address, self.WHO_AM_I)

    def close(self):
        """Close the underlying I2C bus."""
        self.bus.close()
