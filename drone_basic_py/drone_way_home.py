#!/usr/bin/env python3

from __future__ import annotations
import math, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import threading

class GoHome(Node):
    # ───────── initialization ────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__('go_home')
        # Parameters
        self.declare_parameter('odom_topic',       'odometry/filtered')
        self.declare_parameter('cmd_vel_topic',    'drone/cmd_vel')
        self.declare_parameter('waypoint_topic',   'drone/waypoint')
        self.declare_parameter('takeoff_height',   3.0)
        self.declare_parameter('ascend_speed',     0.6)
        self.declare_parameter('cruise_speed',     0.5)
        self.declare_parameter('descend_speed',    0.5)
        self.declare_parameter('position_tolerance', 0.3)
        self.declare_parameter('vertical_tolerance', 0.05)
        self.declare_parameter('landing_height',     0.15)
        self.declare_parameter('control_rate',       20.0)  # Hz
        # Topics / services
        odom_t = self.get_parameter('odom_topic').value
        cmd_t  = self.get_parameter('cmd_vel_topic').value
        wpt_t  = self.get_parameter('waypoint_topic').value
        self.create_subscription(Odometry, odom_t, self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_t, 10)
        self.create_subscription(Twist, cmd_t, self._cmd_watch_cb, 10)
        self.create_service(Trigger, 'go_home', self._srv_home_cb)
        self.create_subscription(String, wpt_t, self._waypoint_cb, 10)
        # State
        self.current:  tuple[float, float, float] | None = None
        self.home:     tuple[float, float, float] | None = None
        self.wpts:     list[tuple[float, float, float] | None] = [None] * 5
        self.target:   tuple[float, float, float] | None = None
        self.state: str = 'IDLE'
        self._last_autonomous_pub = self.get_clock().now()  # only when FSM != IDLE
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._loop)
        self.get_logger().info('go_home node up -awaiting odometry…')

    # ───────── subscribers / service handlers ────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.current = (p.x, p.y, p.z)
        if self.home is None:
            self.home = self.current
            self.get_logger().info(f'Home set at {self.home}')

    def _cmd_watch_cb(self, msg: Twist) -> None:
        """Abort autonomous flight on any external cmd_vel."""
        if self.state == 'IDLE':
            return  # no autonomous mission running
        # magnitude of incoming command
        mag = sum(map(abs, (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z)))
        if mag < 1e-4:
            return  # essentially zero – ignore
        # Ignore our own messages (published <150 ms ago) while autonomous
        if (self.get_clock().now() - self._last_autonomous_pub).nanoseconds < 1.5e8:
            return
        self.state = 'IDLE'
        self.get_logger().warn('cmd_vel detected - mission aborted.')

    def _srv_home_cb(self, _, res):
        if not self.home or not self.current:
            res.success, res.message = False, 'No odometry yet'
            return res
        if self.state != 'IDLE':
            res.success, res.message = False, f'Busy ({self.state})'
            return res
        self.target = self.home
        self.state = 'HOME_ASCEND'
        self.get_logger().info('Home sequence initiated')
        res.success, res.message = True, 'Going home'
        return res

    def _waypoint_cb(self, msg: String) -> None:
        key = msg.data.strip()
        # Home keyword over topic ------------------------------------------------
        if key.lower() == 'home':
            if self.state == 'IDLE':
                self.target = self.home
                self.state = 'HOME_ASCEND'
                self.get_logger().info('Home command received')
            else:
                self.get_logger().warn(f'Cannot go home (busy: {self.state})')
            return
        # Waypoints 1‑5 ---------------------------------------------------------
        if key in '12345':
            idx = int(key) - 1
            if self.current is None:
                self.get_logger().warn('No odometry - cannot store/fly waypoint')
                return
            if self.wpts[idx] is None:
                self.wpts[idx] = self.current
                self.get_logger().info(f'Waypoint {key} saved at {self.current}')
            else:
                if self.state != 'IDLE':
                    self.get_logger().warn(f'Busy ({self.state}) ')
                    return
                self.target = self.wpts[idx]
                self.state = 'WP_ASCEND_CRUISE'
                self.get_logger().info(f'Flying to waypoint {key}: {self.target}')
            return
        # Any other key → cancel -------------------------------------------------
        if self.state != 'IDLE':
            self.state = 'IDLE'
            self.get_logger().info('Mission cancelled by user input')

    # ───────── control loop ───────────────────────────────────────────────────
    def _loop(self):
        if self.current is None:
            return
        x, y, z = self.current
        tol_xy  = self.get_parameter('position_tolerance').value
        tol_z   = self.get_parameter('vertical_tolerance').value
        asc     = self.get_parameter('ascend_speed').value
        dsc     = self.get_parameter('descend_speed').value
        crs     = self.get_parameter('cruise_speed').value
        land_z  = self.get_parameter('landing_height').value
        cruise_z = self.get_parameter('takeoff_height').value
        tw = Twist()  # all zeros

        # ── FSM ----------------------------------------------------------------
        if self.state == 'HOME_ASCEND':
            diff = cruise_z - z
            if abs(diff) > tol_z:
                tw.linear.z = asc if diff > 0 else -dsc
            else:
                self.state = 'HOME_CRUISE'; self.get_logger().info('→ HOME_CRUISE')
        elif self.state == 'HOME_CRUISE':
            tx, ty, _ = self.target
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist > tol_xy:
                tw.linear.x = crs * dx / dist
                tw.linear.y = crs * dy / dist
            else:
                self.state = 'HOME_DESCEND'; self.get_logger().info('→ HOME_DESCEND')
        elif self.state == 'HOME_DESCEND':
            if z > land_z + tol_z:
                tw.linear.z = -dsc
            else:
                self.state = 'IDLE'; self.get_logger().info('Home landed')
        elif self.state == 'WP_ASCEND_CRUISE':
            diff = cruise_z - z
            if abs(diff) > tol_z:
                tw.linear.z = asc if diff > 0 else -dsc
            else:
                self.state = 'WP_CRUISE_XY'; self.get_logger().info('WP_CRUISE_XY')
        elif self.state == 'WP_CRUISE_XY':
            tx, ty, _ = self.target
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist > tol_xy:
                tw.linear.x = crs * dx / dist
                tw.linear.y = crs * dy / dist
            else:
                self.state = 'WP_ADJUST_Z'; self.get_logger().info('WP_ADJUST_Z')
        elif self.state == 'WP_ADJUST_Z':
            _, _, tz = self.target
            diff = tz - z
            if abs(diff) > tol_z:
                tw.linear.z = asc if diff > 0 else -dsc
            else:
                self.state = 'IDLE'; self.get_logger().info('Waypoint reached')
        else:  # IDLE – publish NOTHING to avoid masking user cmd_vel
            return

        # Publish and remember time (for cmd_vel filtering) -------------------
        self.cmd_pub.publish(tw)
        self._last_autonomous_pub = self.get_clock().now()

# ───────── main ───────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = GoHome()

    def spin_node():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=spin_node)
    spin_thread.start()

    try:
        spin_thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
