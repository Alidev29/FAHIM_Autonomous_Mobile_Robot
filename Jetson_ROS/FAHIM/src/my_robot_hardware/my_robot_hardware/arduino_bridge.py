"""
FAHIM — Arduino bridge node.

Single node that owns /dev/arduino (serial is exclusive).
  - Subscribes : /cmd_vel        (geometry_msgs/Twist)
  - Publishes  : /odom           (nav_msgs/Odometry)
  - Broadcasts : odom -> base_link TF (optional — disable when EKF is active)

Protocol (must match Arduino firmware):
  TX  L<rpm>\\n  R<rpm>\\n  S\\n
  RX  ENC,<lc>,<rc>,<lrpm>,<rrpm>\\n    (ticks + rpm at 50 Hz)
      DBG,...                            (logged at DEBUG level)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ---- Parameters (overridable from launch) ----
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.235)
        self.declare_parameter('encoder_cpr', 660)
        self.declare_parameter('max_rpm', 333.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        # REP-105: the odom TF child is base_footprint (the 2-D projection
        # of the robot on the ground). The URDF joint base_footprint->base_link
        # carries the 2.5 mm lift. The EKF is configured with
        # base_link_frame: base_footprint, so odom.child_frame_id must match.
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        # Per-wheel command calibration. 1.0 means no change. If the
        # robot veers right on a straight forward command, left is
        # running faster than right — drop left_motor_scale (e.g. 0.95)
        # or bump right_motor_scale. Opposite for veer-left.
        self.declare_parameter('left_motor_scale',  1.0)
        self.declare_parameter('right_motor_scale', 1.0)
        # Per-wheel odometry calibration. If driving a measured 1.00 m
        # shows x = 1.03 on one side, set that side's odom_scale to
        # 1.00 / 1.03 = 0.971.
        self.declare_parameter('left_odom_scale',  1.0)
        self.declare_parameter('right_odom_scale', 1.0)
        # Minimum motor RPM applied when a non-zero speed is commanded.
        # JGB37-520 motors stall below ~20-30 PWM due to static friction.
        # At max_rpm=333, a 26 RPM pure-rotation command maps to only
        # ~20 PWM — below the stiction threshold, so the motors don't spin.
        # Setting min_motor_rpm=50 (~38 PWM) ensures in-place rotation works.
        # Tune upward if the motors still stall; tune downward if the robot
        # jerks at low-speed starts.
        self.declare_parameter('min_motor_rpm', 50.0)

        port          = self.get_parameter('port').value
        baud          = int(self.get_parameter('baud').value)
        self.wheel_r  = float(self.get_parameter('wheel_radius').value)
        self.wheel_s  = float(self.get_parameter('wheel_separation').value)
        self.cpr      = int(self.get_parameter('encoder_cpr').value)
        self.max_rpm  = float(self.get_parameter('max_rpm').value)
        self.pub_tf   = bool(self.get_parameter('publish_tf').value)
        self.odom_fr  = self.get_parameter('odom_frame').value
        self.base_fr  = self.get_parameter('base_frame').value
        self.cmd_to   = float(self.get_parameter('cmd_vel_timeout').value)
        self.l_cmd_k  = float(self.get_parameter('left_motor_scale').value)
        self.r_cmd_k  = float(self.get_parameter('right_motor_scale').value)
        self.l_odo_k  = float(self.get_parameter('left_odom_scale').value)
        self.r_odo_k  = float(self.get_parameter('right_odom_scale').value)
        self.min_rpm  = float(self.get_parameter('min_motor_rpm').value)

        # meters of travel per encoder tick
        self.m_per_tick = 2.0 * math.pi * self.wheel_r / float(self.cpr)

        # ---- Serial port (exclusive) ----
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open {port}: {e}')
            raise
        time.sleep(2.0)                 # let Arduino reset on DTR
        self.ser.reset_input_buffer()
        self.ser_lock = threading.Lock()

        # ---- Odometry state ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_lc = None
        self.last_rc = None
        self.last_odom_time = None
        # Last valid velocity (held when dt is too small to compute a new one)
        self.last_v = 0.0
        self.last_w = 0.0

        # ---- ROS I/O ----
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_bc = TransformBroadcaster(self) if self.pub_tf else None

        # ---- Watchdog on /cmd_vel ----
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog)

        # ---- Serial RX thread ----
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(
            f'arduino_bridge up — port={port} baud={baud} '
            f'r={self.wheel_r} sep={self.wheel_s} cpr={self.cpr} '
            f'publish_tf={self.pub_tf}'
        )

    # ============================================================
    # /cmd_vel -> L<rpm>\n R<rpm>\n
    # ============================================================
    def cmd_vel_cb(self, msg: Twist):
        v = float(msg.linear.x)    # m/s
        w = float(msg.angular.z)   # rad/s

        v_l = v - w * self.wheel_s / 2.0
        v_r = v + w * self.wheel_s / 2.0

        rpm_l = (v_l / (2.0 * math.pi * self.wheel_r)) * 60.0
        rpm_r = (v_r / (2.0 * math.pi * self.wheel_r)) * 60.0

        # Per-wheel command calibration — compensates for motor / tire
        # asymmetry so a straight cmd_vel actually drives straight.
        rpm_l *= self.l_cmd_k
        rpm_r *= self.r_cmd_k

        # Dead-zone compensation: if a non-zero RPM is commanded but it
        # falls below min_motor_rpm, raise it to the minimum so the motor
        # actually overcomes static friction.  Zero stays zero (stop command).
        if 0.0 < abs(rpm_l) < self.min_rpm:
            rpm_l = math.copysign(self.min_rpm, rpm_l)
        if 0.0 < abs(rpm_r) < self.min_rpm:
            rpm_r = math.copysign(self.min_rpm, rpm_r)

        rpm_l = max(-self.max_rpm, min(self.max_rpm, rpm_l))
        rpm_r = max(-self.max_rpm, min(self.max_rpm, rpm_r))

        with self.ser_lock:
            self.ser.write(f'L{rpm_l:.1f}\n'.encode('ascii'))
            self.ser.write(f'R{rpm_r:.1f}\n'.encode('ascii'))

        self.last_cmd_time = self.get_clock().now()

    def watchdog(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_to:
            with self.ser_lock:
                self.ser.write(b'S\n')

    # ============================================================
    # Serial RX loop
    # ============================================================
    def rx_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.get_logger().warning(f'serial read error: {e}')
                time.sleep(0.1)
                continue
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if line.startswith('ENC,'):
                self._handle_enc(line)
            elif line.startswith('DBG,'):
                self.get_logger().debug(line)

    def _handle_enc(self, line: str):
        parts = line.split(',')
        # ENC,<lc>,<rc>,<lrpm>,<rrpm>
        if len(parts) < 5:
            return
        try:
            lc   = int(parts[1])
            rc   = int(parts[2])
            lrpm = float(parts[3])
            rrpm = float(parts[4])
        except ValueError:
            return

        now = self.get_clock().now()

        if self.last_lc is None:
            self.last_lc = lc
            self.last_rc = rc
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        d_l = (lc - self.last_lc) * self.m_per_tick * self.l_odo_k
        d_r = (rc - self.last_rc) * self.m_per_tick * self.r_odo_k

        d_center = (d_l + d_r) / 2.0
        d_theta  = (d_r - d_l) / self.wheel_s

        # second-order integration (midpoint) — uses tick counts (immune to dt jitter)
        mid_theta = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta = self._wrap(self.theta + d_theta)

        # ---- Velocity: Arduino hardware RPM (not d_tick/dt) ----
        # The tick-based d_center/dt calculation suffers from Python GIL + USB
        # burst-read timing jitter: the RX thread sometimes processes two ENC
        # messages in rapid succession, producing an inflated dt on the follow-
        # on message and a velocity that oscillates between the correct value
        # and ~0.5× the correct value, biasing the EKF output by ~25 %.
        #
        # The Arduino measures RPM via period-timing in the encoder ISR
        # (hardware-accurate, independent of the host scheduler).  The firmware
        # fix (processCmd no longer resets the 8-sample RPM buffer on every
        # L/R command) means actualRPM now tracks the true wheel speed during
        # steady-state driving.  Using it here gives a clean, stable velocity
        # signal with no Python-side timing dependency.
        #
        # Position integration above still uses tick counts (cumulative,
        # immune to dt jitter) so the pose remains correct.
        l_vel = lrpm * (2.0 * math.pi * self.wheel_r) / 60.0 * self.l_odo_k
        r_vel = rrpm * (2.0 * math.pi * self.wheel_r) / 60.0 * self.r_odo_k
        v     = (l_vel + r_vel) / 2.0
        w     = (r_vel - l_vel) / self.wheel_s
        max_v = self.max_rpm * 2.0 * math.pi * self.wheel_r / 60.0
        v     = max(-max_v, min(max_v, v))

        self.last_lc = lc
        self.last_rc = rc
        self.last_odom_time = now

        self._publish(now, v, w)

    @staticmethod
    def _wrap(a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    # ============================================================
    # Publish /odom + TF
    # ============================================================
    def _publish(self, stamp, v, w):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        od = Odometry()
        od.header.stamp = stamp.to_msg()
        od.header.frame_id = self.odom_fr
        od.child_frame_id  = self.base_fr
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x  = v
        od.twist.twist.angular.z = w

        # Covariances — pose is not fused (odom0_config all-false for pose);
        # twist vx and vyaw are fused; everything else is unobserved.
        # IMPORTANT: unobserved states must use 1e6 (not 0.0) so that the EKF
        # does not accidentally interpret zero as "infinite certainty = exactly 0".
        for i in range(36):
            od.pose.covariance[i]  = 0.0
            od.twist.covariance[i] = 0.0
        # Pose — all disabled in odom0_config but fill in reasonable values anyway
        od.pose.covariance[0]   = 0.01   # x
        od.pose.covariance[7]   = 0.01   # y
        od.pose.covariance[14]  = 1e6    # z  (not observed)
        od.pose.covariance[21]  = 1e6    # roll  (not observed)
        od.pose.covariance[28]  = 1e6    # pitch (not observed)
        od.pose.covariance[35]  = 0.05   # yaw
        # Twist — vx and vyaw are the only fused states (odom0_config[6], [11])
        od.twist.covariance[0]  = 0.01   # vx   (observed)
        od.twist.covariance[7]  = 1e6    # vy   (not observed — diff drive)
        od.twist.covariance[14] = 1e6    # vz   (not observed)
        od.twist.covariance[21] = 1e6    # vroll  (not observed)
        od.twist.covariance[28] = 1e6    # vpitch (not observed)
        od.twist.covariance[35] = 0.05   # vyaw (observed)

        self.odom_pub.publish(od)

        if self.tf_bc is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_fr
            t.child_frame_id  = self.base_fr
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_bc.sendTransform(t)

    # ============================================================
    def destroy_node(self):
        self.running = False
        try:
            with self.ser_lock:
                self.ser.write(b'S\n')
                self.ser.flush()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
