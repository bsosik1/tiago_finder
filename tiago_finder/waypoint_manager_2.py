from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped,  Quaternion
import rclpy
import math
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.action import ActionClient
from play_motion2_msgs.action import PlayMotion2
from rclpy.task import Future


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
        self.movement_completed = False
        self.object_found = False
        self.goal_handle = None
        self.rate = self.create_rate(2)
        self.received_goal = False
        # SUBSCRIBER
        self._found_subscriber = self.create_subscription(
            Bool, 'object_found', self.found_callback, 10)
        self.goal_subscriber = self.create_subscription(
            String, '/intention', self.goal_sub_callback, 10)
        # ACTION
        self._action_client = ActionClient(self, PlayMotion2, 'play_motion2')

    def goal_sub_callback(self, msg):
        self.received_goal = True

    # HEAD MOVEMENT HANDLING
    def send_goal(self):
        msg = PlayMotion2.Goal()
        msg.motion_name = "turn_head"
        msg.skip_planning = False
        self.movement_completed = False

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by the action server!')
            return
        self.get_logger().info('Goal accepted! Head movement is starting...')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        self.movement_completed = True
        p_str = 'Movement finished with success: {0}'
        self.get_logger().info(p_str.format(result.success))
        if result.success is False and self.object_found is True:
            p_str = "Head movement stopped. Shutting down..."
            self.get_logger().info(p_str)
        elif result.success is True:
            self.get_logger().info('Head movement completed!')
        else:
            p_str = 'Error. Shutting down...'
            self.get_logger().error(p_str)
            rclpy.shutdown()

    def found_callback(self, msg):
        self.get_logger().info('Object found! Stopping motion...')
        self.object_found = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._basic_navigator.cancelTask()

    def proceed_through_waypoints(self):
        self._basic_navigator.waitUntilNav2Active()
        waypoints = read_yaml_waypoints(
            '/home/bsosik/tiago_ros2_ws/src/tiago_finder/config/waypoints/W_1')
        goal_poses = convert_waypoints_to_msg(waypoints, self._basic_navigator)
        prev_i = 0
        i = 0
        for pose in goal_poses:
            if self.object_found:
                break
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
                rclpy.spin_once(self)
            result = self._basic_navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Waypoint {i} was reached!')
                print('Waiting for head movement before next waypoint...')
                self.send_goal()
                while not self.movement_completed:
                    rclpy.spin_once(self)
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
    while waypoint_manager.received_goal is False:
        rclpy.spin_once(waypoint_manager)
    waypoint_manager.proceed_through_waypoints()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
