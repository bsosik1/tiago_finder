from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped,  Quaternion
import rclpy
import math
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from rclpy.duration import Duration


def quaternion_from_euler(roll, pitch, yaw):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def read_yaml_waypoints(path):
    with open(f'{path}.yaml', 'r') as f:
        output = yaml.safe_load(f)
    return output


def convert_waypoints_to_msg(waypoints, navigator: BasicNavigator):
    msg_list = []
    for key in waypoints:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = navigator.get_clock().now().to_msg()
        waypoint = waypoints[key]
        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]
        msg.pose.position.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, waypoint[2])
        msg.pose.orientation.w = quat.w
        msg.pose.orientation.x = quat.x
        msg.pose.orientation.y = quat.y
        msg.pose.orientation.z = quat.z
        msg_list.append(msg)
    return msg_list


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        self._basic_navigator = BasicNavigator()
        self.head_movement_finished = False
        # SUBSCRIBERS
        self._found_subscriber = self.create_subscription(
            Bool, 'object_found', self.found_callback, 10)
        self._head_movement_finished_subscriber = self.create_subscription(
            Empty, 'head_movement_finished',
            self.head_movement_finished_callback, 10)
        # PUBLISHER
        self._waypoint_reached_publisher = self.create_publisher(
            Empty, 'waypoint_reached', 10)

    def found_callback(self, msg):
        self.get_logger().info('Object found! Stopping motion...')
        self._basic_navigator.cancelTask()

    def head_movement_finished_callback(self, msg):
        self.get_logger().info('Head movement finished!')
        self.head_movement_finished = True

    def proceed_through_waypoints(self):
        # TODO - find more elegant way...
        self._basic_navigator.waitUntilNav2Active()
        waypoints = read_yaml_waypoints(
            '/home/bsosik/tiago_ros2_ws/src/tiago_finder/config/waypoints/W_1')
        goal_poses = convert_waypoints_to_msg(waypoints, self._basic_navigator)
        prev_i = 0
        i = 0
        for pose in goal_poses:
            self._basic_navigator.goToPose(pose)
            i += 1
            while not self._basic_navigator.isTaskComplete():
                if i != prev_i:
                    print(
                        'Executing waypoint: '
                        + str(i)
                        + '/'
                        + str(len(goal_poses))
                        )
                prev_i = i
            result = self._basic_navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Waypoint {i} was reached!')
                print('Waiting for head movement before next waypoint...')
                msg = Empty()
                self._waypoint_reached_publisher.publish(msg)
                now_start = self._basic_navigator.get_clock().now()
                while not self.head_movement_finished:
                    now = self._basic_navigator.get_clock().now()
                    if now - now_start > Duration(seconds=45.0):
                        print('Waiting time exceeded! Moving to next waypoint')
                        break
                self.head_movement_finished = False
            elif result == TaskResult.CANCELED:
                print('Goal was cancelled. Check if object was found.')
            elif result == TaskResult.FAILED:
                print('Could not visit a waypoint!')
                continue
            else:
                print('Invalid return status!')
        print('Visited each waypoint successfully')


def main(args=None):
    rclpy.init(args=None)
    waypoint_manager = WaypointManager()
    waypoint_manager.proceed_through_waypoints()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
