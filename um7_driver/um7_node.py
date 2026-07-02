"""
ROS2 node that publishes UM7 IMU data using the ROS-free parser.

Responsibilities kept here (not in the parser): parameters, serial I/O with
reconnect, REP-103 unit conversion, message assembly, event-driven publishing,
frame convention, and an optional debug TF broadcast.

Orientation source is the Euler registers (112-116); the UM7 boots in Euler
mode so no configuration write is needed (see CLAUDE.md, "A 모드").
"""

import math
import threading

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import UInt32
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:  # pragma: no cover - exercised only without pyserial
    serial = None

from um7_driver.um7_parser import Um7Parser

DEG_TO_RAD = math.pi / 180.0

# Constant reframing quaternions (x, y, z, w). See CLAUDE.md "프레임 규약":
# the UM7 output frame is NOT specified in the datasheet, so the enu transform
# below is an *assumption* (NED world / FRD body) that must be validated on
# hardware. Fix sign/axis issues here in the node, never in the parser.
_Q_NED_TO_ENU = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)  # 180 deg about (1,1,0)
_Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)                        # 180 deg about x


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert ZYX Euler angles (rad) to a quaternion ``(x, y, z, w)``."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(a: tuple, b: tuple) -> tuple:
    """Return the Hamilton product ``a * b`` of two ``(x, y, z, w)`` quats."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class Um7Node(Node):
    """Read a UM7 over serial and publish IMU, magnetometer and temperature."""

    def __init__(self) -> None:
        """Declare parameters, create publishers, and start the serial reader."""
        super().__init__('um7_node')

        self.port = self.declare_parameter('port', '').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        convention = self.declare_parameter('frame_convention', 'enu').value
        self.publish_tf = self.declare_parameter('publish_tf', False).value
        self.publish_health = self.declare_parameter('publish_health', False).value

        self._to_enu = str(convention).lower() == 'enu'
        if self._to_enu:
            self.get_logger().warn(
                'frame_convention=enu applies an ASSUMED NED->ENU transform; '
                'the UM7 output frame is not in the datasheet -- verify axis '
                'signs on hardware (see CLAUDE.md 프레임 규약).')

        self._parser = Um7Parser()
        self._state: 'dict[str, float]' = {}

        self._imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self._mag_pub = self.create_publisher(
            MagneticField, 'imu/mag', qos_profile_sensor_data)
        self._temp_pub = self.create_publisher(
            Temperature, 'imu/temperature', qos_profile_sensor_data)
        self._health_pub = None
        if self.publish_health:
            self._health_pub = self.create_publisher(UInt32, 'imu/health', 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._stop = threading.Event()
        self._thread = None
        if serial is None:
            self.get_logger().error('pyserial not installed; cannot open the port.')
        elif not self.port:
            self.get_logger().error(
                "Parameter 'port' is empty; set it to a /dev/serial/by-id/... path.")
        else:
            self._thread = threading.Thread(target=self._serial_loop, daemon=True)
            self._thread.start()

    # --- serial I/O ---------------------------------------------------------

    def _serial_loop(self) -> None:
        """Read the serial port forever, reconnecting with backoff on error."""
        backoff, backoff_max = 0.5, 5.0
        while rclpy.ok() and not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.1) as port:
                    self.get_logger().info(f'Opened {self.port} @ {self.baud} baud')
                    backoff = 0.5
                    self._read_until_error(port)
            except (OSError, serial.SerialException) as exc:
                self.get_logger().warn(
                    f'Serial error: {exc}; reconnecting in {backoff:.1f}s')
                self._stop.wait(backoff)
                backoff = min(backoff * 2.0, backoff_max)

    def _read_until_error(self, port) -> None:
        """Pump bytes from an open serial ``port`` into the parser."""
        while rclpy.ok() and not self._stop.is_set():
            data = port.read(256)
            if data:
                for packet in self._parser.feed(data):
                    self._handle_packet(packet)

    def shutdown(self) -> None:
        """Signal the serial thread to stop and wait for it to finish."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # --- packet handling ----------------------------------------------------

    def _handle_packet(self, packet) -> None:
        """Update cached state and publish messages driven by this packet."""
        self._state.update(packet.values)
        stamp = self.get_clock().now().to_msg()
        keys = packet.values.keys()

        if keys & {'roll', 'pitch', 'yaw', 'gyro_proc_x', 'gyro_proc_y',
                   'gyro_proc_z', 'accel_proc_x', 'accel_proc_y', 'accel_proc_z'}:
            self._publish_imu(stamp)
        if keys & {'mag_proc_x', 'mag_proc_y', 'mag_proc_z'}:
            self._publish_mag(stamp)
        if 'temperature' in keys:
            self._publish_temperature(stamp)
        if self._health_pub is not None and 'health' in keys:
            self._health_pub.publish(UInt32(data=int(self._state['health'])))

    def _publish_imu(self, stamp) -> None:
        """Assemble and publish a sensor_msgs/Imu from cached state."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        if {'roll', 'pitch', 'yaw'} <= self._state.keys():
            quat = quaternion_from_euler(
                self._state['roll'] * DEG_TO_RAD,
                self._state['pitch'] * DEG_TO_RAD,
                self._state['yaw'] * DEG_TO_RAD)
            if self._to_enu:
                quat = quaternion_multiply(
                    _Q_NED_TO_ENU, quaternion_multiply(quat, _Q_FRD_TO_FLU))
            msg.orientation.x, msg.orientation.y, msg.orientation.z, \
                msg.orientation.w = quat
        else:
            msg.orientation_covariance[0] = -1.0  # orientation unavailable

        if {'gyro_proc_x', 'gyro_proc_y', 'gyro_proc_z'} <= self._state.keys():
            gx, gy, gz = self._to_body(
                self._state['gyro_proc_x'] * DEG_TO_RAD,
                self._state['gyro_proc_y'] * DEG_TO_RAD,
                self._state['gyro_proc_z'] * DEG_TO_RAD)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = \
                gx, gy, gz
        else:
            msg.angular_velocity_covariance[0] = -1.0

        if {'accel_proc_x', 'accel_proc_y', 'accel_proc_z'} <= self._state.keys():
            # DREG_ACCEL_PROC is already m/s^2 -- no scaling (see CLAUDE.md).
            ax, ay, az = self._to_body(
                self._state['accel_proc_x'],
                self._state['accel_proc_y'],
                self._state['accel_proc_z'])
            msg.linear_acceleration.x, msg.linear_acceleration.y, \
                msg.linear_acceleration.z = ax, ay, az
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        # Covariance TODO (REP-145): the UM7 does not report covariance; the
        # matrices are left as unknown (all zeros) for available fields.
        self._imu_pub.publish(msg)

        if self._tf_broadcaster is not None and {'roll', 'pitch', 'yaw'} <= \
                self._state.keys():
            self._broadcast_tf(stamp, msg.orientation)

    def _publish_mag(self, stamp) -> None:
        """
        Publish sensor_msgs/MagneticField from cached mag state.

        NOTE: DREG_MAG_PROC is unit-norm (dimensionless), not Tesla. The
        vector is published as-is for heading; treat magnitude with caution
        until a raw->Tesla scale is decided (see CLAUDE.md TODO).
        """
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        mx, my, mz = self._to_body(
            self._state['mag_proc_x'],
            self._state['mag_proc_y'],
            self._state['mag_proc_z'])
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = mx, my, mz
        self._mag_pub.publish(msg)

    def _publish_temperature(self, stamp) -> None:
        """Publish sensor_msgs/Temperature (already degrees Celsius)."""
        msg = Temperature()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.temperature = float(self._state['temperature'])
        msg.variance = 0.0
        self._temp_pub.publish(msg)

    def _broadcast_tf(self, stamp, orientation) -> None:
        """Broadcast a world->frame_id transform for RViz debugging."""
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = self.frame_id
        transform.transform.rotation = orientation
        self._tf_broadcaster.sendTransform(transform)

    def _to_body(self, x: float, y: float, z: float) -> tuple:
        """
        Map a body-frame vector to ENU (FLU) axes when enabled.

        FRD->FLU is a 180-degree rotation about the x axis: (x, -y, -z).
        """
        if self._to_enu:
            return (x, -y, -z)
        return (x, y, z)


def main(args=None) -> None:
    """Spin the UM7 node until shutdown."""
    rclpy.init(args=args)
    node = Um7Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
