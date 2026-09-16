import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

OCCUPIED_THRESHOLD = 50     # Catches obstacles and their immediate inflation area
SAMPLE_STEP = 0.15        # Check every 15 cm along the corridor
WIDTH_BUFFER = 0.35       # Check 35 cm on both sides of center-line
ENTRANCE_LOOKBACK = 0.60  # Check 60 cm into the entrance approach
ROBOTS = ['robot_1', 'robot_2']

HOME_POSITIONS = {
    'robot_1': {'x': 47.5, 'y': 12.3,    'yaw': math.pi},
    'robot_2': {'x': 47.5, 'y': 15.0475, 'yaw': math.pi}
}

SAFE_SEGMENTS = [
    # Horizontal
    {'zone': '8', 'type': 'H', 'x': -49.54, 'y_min': -16.47, 'y_max': 12.29},
    {'zone': '9', 'type': 'H', 'x': -44.0,  'y_min': -6.69,  'y_max': -3.67},
    {'zone': '10', 'type': 'H', 'x': -24.0, 'y_min': 1.8,    'y_max': 8.0},
    {'zone': '13', 'type': 'H', 'x': -12.8, 'y_min': 15.68,  'y_max': 21.41},

    # Vertical
    {'zone': '1', 'type': 'V', 'y': 11.46, 'x_min': 34.73, 'x_max': 38.54},
    {'zone': '2', 'type': 'V', 'y': 11.46, 'x_min': 21.82, 'x_max': 26.4},
    {'zone': '3', 'type': 'V', 'y': 8.15,  'x_min': 34.86, 'x_max': 43.56},
    {'zone': '4', 'type': 'V', 'y': 8.15,  'x_min': 21.8,  'x_max': 30.8},
    {'zone': '5', 'type': 'V', 'y': 1.4,   'x_min': 35.26, 'x_max': 39.81},
    {'zone': '6', 'type': 'V', 'y': 1.4,   'x_min': 22.0,  'x_max': 27.39},

    {'zone': '7', 'type': 'V', 'y': -11.77, 'x_min': -33.41, 'x_max': -32.09},
    {'zone': '11', 'type': 'V', 'y': 11.3,  'x_min': -19.27, 'x_max': -16.27},
    {'zone': '12', 'type': 'V', 'y': 24.5,  'x_min': -9.45,  'x_max': -5.24},
]

RED_ZONES = [
    {'id': 1, 'x_min': 35.0,  'x_max': 43.9,  'y_min': 11.38, 'y_max': 13.36},
    {'id': 2, 'x_min': 21.77, 'x_max': 30.57, 'y_min': 11.17, 'y_max': 13.29},
    {'id': 3, 'x_min': 31.48, 'x_max': 34.32, 'y_min': 2.13,  'y_max': 10.54},
    {'id': 4, 'x_min': 32.9,  'x_max': 44.92, 'y_min': -2.22, 'y_max': 1.47},
    {'id': 5, 'x_min': 18.75, 'x_max': 32.85, 'y_min': -2.21, 'y_max': 1.76},
]

station_zone = [
    {'id': 1, 'x': 20.5,   'y': 5.08  }, # P1
    {'id': 2, 'x': 7.0,    'y': -1.63 }, # P2
    {'id': 3, 'x': -19.16, 'y': -1.44 }, # P3
    {'id': 4, 'x': -23.41, 'y': 2.04  }, # P4
    {'id': 5, 'x': -48.87, 'y': -12.62}, # P5
    {'id': 6, 'x': -32.6,  'y': -12.06}, # P6
    {'id': 7, 'x': -16.15, 'y': 24.02 }, # P7
    {'id': 8, 'x': -15.22, 'y': 11.39 }, # P8
]


def get_red_zone(rx: float, ry: float) -> dict | None:
    for zone in RED_ZONES:
        if zone['x_min'] <= rx <= zone['x_max'] and zone['y_min'] <= ry <= zone['y_max']:
            return zone
    return None


def get_segment_target_pose(rx: float, ry: float, seg: dict) -> tuple[float, float]:
    offset = 1.5

    if seg['type'] == 'V':
        y = seg['y']
        x_min, x_max = min(seg['x_min'], seg['x_max']), max(seg['x_min'], seg['x_max'])

        if rx <= x_min:
            x = min(x_min + offset, x_max)
        elif rx >= x_max:
            x = max(x_max - offset, x_min)
        else:
            center_x = (x_min + x_max) / 2.0
            x = min(x_min + offset, x_max) if rx < center_x else max(x_max - offset, x_min)

        return x, y
    else:
        x = seg['x']
        y_min, y_max = min(seg['y_min'], seg['y_max']), max(seg['y_min'], seg['y_max'])

        if ry <= y_min:
            y = min(y_min + offset, y_max)
        elif ry >= y_max:
            y = max(y_max - offset, y_min)
        else:
            center_y = (y_min + y_max) / 2.0
            y = min(y_min + offset, y_max) if ry < center_y else max(y_max - offset, y_min)

        return x, y


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self._lock            = threading.Lock()
        self._is_alarm_active = False

        self._robot_x        = {r: HOME_POSITIONS[r]['x'] for r in ROBOTS}
        self._robot_y        = {r: HOME_POSITIONS[r]['y'] for r in ROBOTS}
        self._robot_yaw      = {r: HOME_POSITIONS[r]['yaw'] for r in ROBOTS}
        self._status         = {r: 'idle' for r in ROBOTS} # idle, working, returning, evacuating
        self._goal_id        = {r: 0 for r in ROBOTS}
        self._costmap        = {r: None for r in ROBOTS}
        self._current_handle = {r: None for r in ROBOTS}
        self._stop_timer     = {r: None for r in ROBOTS}

        self._nav_client    = {}
        self._cancel_client = {}
        self._cmd_vel_pub   = {}

        for r in ROBOTS:
            self._nav_client[r] = ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            self._cancel_client[r] = self.create_client(CancelGoal, f'/{r}/navigate_to_pose/_action/cancel_goal')
            self._cmd_vel_pub[r] = self.create_publisher(Twist, f'/{r}/cmd_vel', 10)

            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{r}/amcl_pose',
                lambda msg, r_name=r: self._amcl_cb(msg, r_name),
                10
            )
            self.create_subscription(
                OccupancyGrid,
                f'/{r}/global_costmap/costmap',
                lambda msg, r_name=r: self._costmap_cb(msg, r_name),
                10
            )

        self.create_subscription(String, '/alarm', self._alarm_cb, 10)
        self.get_logger().info('Multi-Robot Station Controller initialized.')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped, r_name: str):
        with self._lock:
            self._robot_x[r_name] = msg.pose.pose.position.x
            self._robot_y[r_name] = msg.pose.pose.position.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self._robot_yaw[r_name] = 2.0 * math.atan2(qz, qw)

    def _costmap_cb(self, msg: OccupancyGrid, r_name: str):
        self._costmap[r_name] = msg

    def _is_point_occupied_any(self, x: float, y: float) -> bool:
        for r_name in ROBOTS:
            costmap = self._costmap[r_name]
            if costmap is None:
                continue

            info = costmap.info
            gx = int((x - info.origin.position.x) / info.resolution)
            gy = int((y - info.origin.position.y) / info.resolution)

            if 0 <= gx < info.width and 0 <= gy < info.height:
                idx = gy * info.width + gx
                cost = costmap.data[idx]
                if cost >= OCCUPIED_THRESHOLD:
                    return True
        return False

    def _is_segment_3_blocked(self) -> bool:
        seg3 = next(s for s in SAFE_SEGMENTS if s['zone'] == '3')
        y_center = seg3['y']
        x_min = min(seg3['x_min'], seg3['x_max']) - ENTRANCE_LOOKBACK
        x_max = max(seg3['x_min'], seg3['x_max']) + ENTRANCE_LOOKBACK

        x_curr = x_min
        while x_curr <= x_max:
            for dy in [-WIDTH_BUFFER, -WIDTH_BUFFER / 2.0, 0.0, WIDTH_BUFFER / 2.0, WIDTH_BUFFER]:
                if self._is_point_occupied_any(x_curr, y_center + dy):
                    return True
            x_curr += SAMPLE_STEP

        return False

    def dispatch_station_goal(self, st_x: float, st_y: float, station_id: int):
        with self._lock:
            if self._is_alarm_active:
                self.get_logger().error('Cannot dispatch goal while Alarm is active!')
                return

            s1 = self._status['robot_1']
            s2 = self._status['robot_2']

            # Filter available robots (idle or returning)
            available = [r for r in ROBOTS if self._status[r] in ['idle', 'returning']]

            if not available:
                self.get_logger().warn('Both robots are currently WORKING. Station request rejected.')
                return

            selected_robot = None

            if len(available) == 1:
                selected_robot = available[0]
            else:
                if s1 == 'idle' and s2 == 'idle':
                    selected_robot = 'robot_1'
                else:
                    dist1 = math.hypot(st_x - self._robot_x['robot_1'], st_y - self._robot_y['robot_1'])
                    dist2 = math.hypot(st_x - self._robot_x['robot_2'], st_y - self._robot_y['robot_2'])

                    if dist1 <= dist2:
                        selected_robot = 'robot_1'
                    else:
                        selected_robot = 'robot_2'

        rx, ry = self._robot_x[selected_robot], self._robot_y[selected_robot]
        dx, dy = st_x - rx, st_y - ry
        yaw_target = math.atan2(dy, dx)

        self.get_logger().info(
            f'Dispatched [{selected_robot}] to Station P{station_id} ({st_x}, {st_y}).'
        )
        self._send_goal(selected_robot, st_x, st_y, yaw=yaw_target, task_type='station')

    def _alarm_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'alarm':
            with self._lock:
                if self._is_alarm_active:
                    return
                self._is_alarm_active = True

            self.get_logger().error('ALARM TRIGGERED!')
            self._cancel_all_nav_goals()

            seg3 = next(s for s in SAFE_SEGMENTS if s['zone'] == '3')
            seg4 = next(s for s in SAFE_SEGMENTS if s['zone'] == '4')

            assigned_zones = {
                'robot_1': seg4,
                'robot_2': seg3
            }

            for r_name in ROBOTS:
                rx, ry = self._robot_x[r_name], self._robot_y[r_name]
                target_seg = assigned_zones.get(r_name)

                if target_seg is None:
                    self.get_logger().error(f'[{r_name}] No assigned segment. Stopping.')
                    self._hold_stop(r_name, duration_sec=2.0)
                    continue

                sx, sy = get_segment_target_pose(rx, ry, target_seg)
                seg_type = target_seg['type']
                seg_zone = target_seg['zone']

                dx, dy = sx - rx, sy - ry
                move_angle = math.atan2(dy, dx)

                with self._lock:
                    current_yaw = self._robot_yaw[r_name]

                angle_diff = math.atan2(
                    math.sin(move_angle - current_yaw),
                    math.cos(move_angle - current_yaw)
                )

                raw_yaw = move_angle if abs(angle_diff) <= (math.pi / 2.0) else (move_angle + math.pi)

                if seg_type == 'H':
                    yaw_target = (math.pi / 2.0) if math.sin(raw_yaw) >= 0 else (-math.pi / 2.0)
                else:
                    yaw_target = 0.0 if math.cos(raw_yaw) >= 0 else math.pi

                self.get_logger().warn(
                    f'[{r_name}] Evacuating to safe segment {seg_zone} at ({sx:.2f}, {sy:.2f}) '
                    f'yaw={math.degrees(yaw_target):.1f}°'
                )
                self._send_goal(r_name, sx, sy, yaw=yaw_target, task_type='evac')

        elif cmd == 'clear':
            with self._lock:
                self._is_alarm_active = False
                for r in ROBOTS:
                    if self._status[r] == 'evacuating':
                        self._status[r] = 'idle'
            self.get_logger().info('Alarm cleared.')


    def _cancel_all_nav_goals(self):
        for r_name in ROBOTS:
            if self._cancel_client[r_name].service_is_ready():
                req = CancelGoal.Request()
                self._cancel_client[r_name].call_async(req)

    def _hold_stop(self, r_name: str, duration_sec: float = 2.0, hz: float = 20.0):
        if self._stop_timer[r_name] is not None:
            self._stop_timer[r_name].cancel()

        ticks_left = int(duration_sec * hz)
        period = 1.0 / hz

        def _tick():
            nonlocal ticks_left
            self._cmd_vel_pub[r_name].publish(Twist())
            ticks_left -= 1
            if ticks_left <= 0 and self._stop_timer[r_name] is not None:
                self._stop_timer[r_name].cancel()
                self._stop_timer[r_name] = None

        self._stop_timer[r_name] = self.create_timer(period, _tick)

    def _send_goal(self, r_name: str, gx: float, gy: float, yaw: float = 0.0, task_type: str = 'station'):
        self._nav_client[r_name].wait_for_server()

        with self._lock:
            self._goal_id[r_name] += 1
            current_gid = self._goal_id[r_name]

            if task_type == 'station':
                self._status[r_name] = 'working'
            elif task_type == 'home':
                self._status[r_name] = 'returning'
            elif task_type == 'evac':
                self._status[r_name] = 'evacuating'

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = gx
        pose.pose.position.y    = gy
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        fut = self._nav_client[r_name].send_goal_async(
            goal, feedback_callback=lambda fb, r=r_name: self._feedback_cb(fb, r))
        fut.add_done_callback(
            lambda f, r=r_name, gid=current_gid, t_type=task_type: self._goal_response_cb(f, r, gid, t_type))

    def _goal_response_cb(self, future, r_name: str, goal_id: int, task_type: str):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'[{r_name}] Goal REJECTED.')
            with self._lock:
                if self._goal_id[r_name] == goal_id:
                    self._status[r_name] = 'idle'
                    self._current_handle[r_name] = None
            return

        with self._lock:
            if self._goal_id[r_name] == goal_id:
                self._current_handle[r_name] = handle

        handle.get_result_async().add_done_callback(
            lambda f, r=r_name, gid=goal_id, t_type=task_type: self._result_cb(f, r, gid, t_type))

    def _result_cb(self, future, r_name: str, goal_id: int, task_type: str):
        with self._lock:
            # Preemption check: Ignore if goal ID is outdated
            if self._goal_id[r_name] != goal_id:
                self.get_logger().info(f'[{r_name}] Task ({task_type}) preempted by newer command.')
                return

            self._current_handle[r_name] = None

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            if task_type == 'station':
                self.get_logger().info(f'[{r_name}] Arrived at Station! Returning home...')
                home = HOME_POSITIONS[r_name]
                self._send_goal(r_name, home['x'], home['y'], yaw=home['yaw'], task_type='home')

            elif task_type == 'home':
                self.get_logger().info(f'[{r_name}] Arrived Home safely.')
                with self._lock:
                    if not self._is_alarm_active:
                        self._status[r_name] = 'idle'

            elif task_type == 'evac':
                self.get_logger().warn(f'[{r_name}] Safe segment reached.')
        else:
            self.get_logger().warn(f'[{r_name}] Task ({task_type}) failed/canceled with status: {status}')
            with self._lock:
                if not self._is_alarm_active:
                    self._status[r_name] = 'idle'

    def _feedback_cb(self, feedback_msg, r_name: str):
        pass

    def terminal_loop(self):
        time.sleep(2.0)
        while rclpy.ok():
            try:
                print('\n' + '=' * 52)
                print('      MULTI-ROBOT NAVIGATION & STATION CONTROL')
                print('=' * 52)

                with self._lock:
                    alarm = self._is_alarm_active

                print(f'  Alarm Status: {"ACTIVE" if alarm else "OFF"}')
                for r in ROBOTS:
                    with self._lock:
                        st = self._status[r]
                        rx, ry = self._robot_x[r], self._robot_y[r]
                    zone = get_red_zone(rx, ry)
                    z_str = f"RED ZONE {zone['id']}" if zone else "Safe/Free Space"
                    print(f'  [{r}] Status: {st.upper():10s} Pos: ({rx:.2f}, {ry:.2f}) Zone: {z_str}')

                print('-' * 52)
                print('  AVAILABLE STATIONS:')
                for st in station_zone:
                    print(f"   [{st['id']}] Station P{st['id']}: ({st['x']:.2f}, {st['y']:.2f})")
                print('-' * 52)

                if alarm:
                    time.sleep(3.0)
                    continue

                st_str = input('  Select Station Zone (1-8): ').strip()
                if not st_str.isdigit():
                    continue

                st_id = int(st_str)
                target_station = next((s for s in station_zone if s['id'] == st_id), None)

                if target_station is None:
                    print('  [ERROR] Invalid station selection.')
                    continue

                self.dispatch_station_goal(target_station['x'], target_station['y'], station_id=st_id)
                time.sleep(1.0)

            except (ValueError, KeyboardInterrupt, EOFError):
                break


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    t = threading.Thread(target=node.terminal_loop, daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()