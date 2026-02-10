#!/usr/bin/env python3
"""ROS2 node that reads MPU6050 over I2C and publishes sensor_msgs/Imu."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from mpu6050_imu.mpu6050_driver import MPU6050Driver, FakeMPU6050Driver


class MPU6050ImuNode(Node):
    def __init__(self):
        super().__init__('mpu6050_imu_node')

        # ── Declare parameters ────────────────────────────────────
        self.declare_parameter('fake_mode', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('accel_range', 0)      # 0=±2g  1=±4g  2=±8g  3=±16g
        self.declare_parameter('gyro_range', 0)        # 0=±250  1=±500  2=±1000  3=±2000 °/s
        self.declare_parameter('accel_covariance', 0.04)
        self.declare_parameter('gyro_covariance', 0.02)

        # ── Read parameters ───────────────────────────────────────
        fake_mode   = self.get_parameter('fake_mode').value
        bus         = self.get_parameter('i2c_bus').value
        address     = self.get_parameter('device_address').value
        rate        = self.get_parameter('publish_rate').value
        self.frame_id     = self.get_parameter('frame_id').value
        accel_range = self.get_parameter('accel_range').value
        gyro_range  = self.get_parameter('gyro_range').value
        self.accel_cov = self.get_parameter('accel_covariance').value
        self.gyro_cov  = self.get_parameter('gyro_covariance').value

        # ── Initialise driver ─────────────────────────────────────
        if fake_mode:
            self.driver = FakeMPU6050Driver()
            self.get_logger().info('FAKE MODE enabled — generating random IMU data')
        else:
            try:
                self.driver = MPU6050Driver(bus, address, accel_range, gyro_range)
                chip_id = self.driver.who_am_i()
                self.get_logger().info(
                    f'MPU6050 initialised  bus={bus}  addr=0x{address:02X}  '
                    f'WHO_AM_I=0x{chip_id:02X}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open MPU6050: {e}')
                raise

        # ── Publisher + timer ─────────────────────────────────────
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing sensor_msgs/Imu on "imu/data_raw" @ {rate} Hz')

    # ──────────────────────────────────────────────────────────────
    def _timer_cb(self):
        try:
            (ax, ay, az), (gx, gy, gz), _ = self.driver.read_all()
        except OSError as e:
            self.get_logger().warn(f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation is not estimated by the raw MPU6050 → mark unknown
        msg.orientation_covariance[0] = -1.0

        # Angular velocity (rad/s)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        gc = self.gyro_cov
        msg.angular_velocity_covariance = [
            gc,  0.0, 0.0,
            0.0, gc,  0.0,
            0.0, 0.0, gc,
        ]

        # Linear acceleration (m/s²)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        ac = self.accel_cov
        msg.linear_acceleration_covariance = [
            ac,  0.0, 0.0,
            0.0, ac,  0.0,
            0.0, 0.0, ac,
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
