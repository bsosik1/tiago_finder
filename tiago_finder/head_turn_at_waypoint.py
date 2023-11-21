import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from play_motion2_msgs.action import PlayMotion2
from rclpy.task import Future
from std_msgs.msg import Bool, Empty


class HeadTurnAtWaypoint(Node):
    def __init__(self):
        super().__init__('head_turn_at_waypoint')
        self._action_client = ActionClient(self, PlayMotion2, 'play_motion2')
        self.movement_completed = False
        self.object_found = False
        self.goal_handle = None
        # SUBSCRIBERS
        self._found_subscriber = self.create_subscription(
            Bool, 'object_found', self.found_callback, 10)
        self._waypoint_reached_subscriber = self.create_subscription(
            Empty, 'waypoint_reached', self.waypoint_reached_callback, 10)
        # PUBLISHERS
        self._movement_completed_publisher = self.create_publisher(
            Empty, 'head_movement_finished', 10)

    def found_callback(self, msg):
        self.get_logger().info('Object found! Stopping motion...')
        self.object_found = True
        self._action_client._cancel_goal_async(self.goal_handle)

    def waypoint_reached_callback(self, msg):
        p_str = "Starting head movement at waypoint..."
        self.get_logger().info(p_str)
        self.send_goal()

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
        self.movement_completed = result.success
        p_str = 'Movement finished with success: {0}'
        self.get_logger().info(p_str.format(self.movement_completed))
        if self.movement_completed is False and self.object_found is True:
            p_str = "Head movement stopped. Shutting down..."
            self.get_logger().info(p_str)
            rclpy.shutdown()
        elif self.movement_completed is True:
            p_str = 'Movement completed. Proceed to the next waypoint...'
            self.get_logger().info(p_str)
            msg = Empty()
            self._movement_completed_publisher.publish(msg)
        else:
            p_str = 'Error. Cannot procced further. Shutting down...'
            self.get_logger().error(p_str)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = HeadTurnAtWaypoint()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
