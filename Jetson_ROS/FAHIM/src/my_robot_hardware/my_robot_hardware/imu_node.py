"""
FAHIM — MPU6050 IMU node.

Reads the MPU6050 over Jetson I2C and publishes sensor_msgs/Imu.
Orientation is NOT computed here (MPU6050 has no magnetometer). The EKF in
robot_localization will fuse angular velocity + linear acceleration with
wheel odometry to produce a filtered pose.

On Jetson Orin Nano the 40-pin header I2C is bus 7 by default. If your
module is wired to the board camera connector, use bus 1. Override via
the `i2c_bus` parameter.

Install the prerequisite on the Jetson:
    sudo apt install -y python3-smbus2
    sudo usermod -aG i2c $USER   # logout/login after this
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError as e:
    raise ImportError(
        'smbus2 not found. Install with: sudo apt install python3-smbus2'
    ) from e


# ---- MPU6050 register map ----
REG_PWR_MGMT_1   = 0x6B
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_XOUT_H  = 0x43

# Scale factors for ±2 g / ±250 °/s
ACCEL_SCALE = 9.80665 / 16384.0              # m/s² per LSB
GYRO_SCALE  = (math.pi / 180.0) / 131.0      # rad/s per LSB


class Mpu6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('calibrate_samples', 500)

        bus_id = int(self.get_parameter('i2c_bus').value)
        self.addr = int(self.get_parameter('i2c_addr').value)
        self.frame = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('rate_hz').value)
        self.n_cal = int(self.get_parameter('calibrate_samples').value)

        try:
            self.bus = SMBus(bus_id)
        except Exception as e:
            self.get_logger().fatal(
                f'Could not open i2c bus {bus_id}: {e}'
            )
            raise

        self._init_sensor()

        # Gyro bias (rad/s) — measured once at startup
        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0
        self._calibrate()

        self.pub = self.create_publisher(Imu, 'imu/data_raw', 30)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'mpu6050 up — bus={bus_id} addr=0x{self.addr:02X} '
            f'rate={rate:.0f}Hz frame={self.frame}'
        )

    # ------------------------------------------------------------
    # Sensor setup
    # ------------------------------------------------------------
    def _init_sensor(self):
        # wake up (clear SLEEP bit, use PLL with X gyro as clock src)
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # sample-rate divider = 7 -> 1kHz / (1+7) = 125 Hz internal
        self.bus.write_byte_data(self.addr, REG_SMPLRT_DIV,  0x07)
        # DLPF = 3 -> 44 Hz bandwidth (decent for 50 Hz publish)
        self.bus.write_byte_data(self.addr, REG_CONFIG,      0x03)
        # ±250 °/s
        self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG,  0x00)
        # ±2 g
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, 0x00)
        time.sleep(0.05)

    def _read_i16(self, reg):
        hi = self.bus.read_byte_data(self.addr, reg)
        lo = self.bus.read_byte_data(self.addr, reg + 1)
        v = (hi << 8) | lo
        if v & 0x8000:
            v = -((~v & 0xFFFF) + 1)
        return v

    def _read_all(self):
        ax = self._read_i16(REG_ACCEL_XOUT_H)      * ACCEL_SCALE
        ay = self._read_i16(REG_ACCEL_XOUT_H + 2)  * ACCEL_SCALE
        az = self._read_i16(REG_ACCEL_XOUT_H + 4)  * ACCEL_SCALE
        gx = self._read_i16(REG_GYRO_XOUT_H)       * GYRO_SCALE
        gy = self._read_i16(REG_GYRO_XOUT_H  + 2)  * GYRO_SCALE
        gz = self._read_i16(REG_GYRO_XOUT_H  + 4)  * GYRO_SCALE
        return ax, ay, az, gx, gy, gz

    def _calibrate(self):
        """Estimate gyro zero-rate bias. Keep the robot perfectly still."""
        self.get_logger().info(
            f'Calibrating gyro bias — KEEP ROBOT PERFECTLY STILL '
            f'({self.n_cal} samples, ~{self.n_cal * 0.005:.1f}s) ...'
        )
        sx = sy = sz = 0.0
        ok = 0
        for _ in range(self.n_cal):
            try:
                _, _, _, gx, gy, gz = self._read_all()
                sx += gx
                sy += gy
                sz += gz
                ok += 1
            except Exception:
                pass
            time.sleep(0.005)
        if ok == 0:
            self.get_logger().warn('Calibration failed — no samples read')
            return
        self.gx_bias = sx / ok
        self.gy_bias = sy / ok
        self.gz_bias = sz / ok
        self.get_logger().info(
            f'Gyro bias (rad/s): x={self.gx_bias:+.5f} '
            f'y={self.gy_bias:+.5f} z={self.gz_bias:+.5f}'
        )
        self.get_logger().info('IMU calibration done — robot may now move.')

    # ------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------
    def tick(self):
        try:
            ax, ay, az, gx, gy, gz = self._read_all()
        except Exception as e:
            self.get_logger().warning(f'i2c read error: {e}')
            return

        gx -= self.gx_bias
        gy -= self.gy_bias
        gz -= self.gz_bias

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        # Orientation: not provided by this driver -> cov[0] = -1 per REP-145
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Noise covariances (diagonal only)
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04
        msg.angular_velocity_covariance[0]    = 0.02
        msg.angular_velocity_covariance[4]    = 0.02
        msg.angular_velocity_covariance[8]    = 0.02

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
