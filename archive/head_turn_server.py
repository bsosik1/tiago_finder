import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from play_motion2_msgs.action import PlayMotion2
from rclpy.task import Future
from std_srvs.srv import Trigger


class HeadTurnAtWaypoint(Node):
    def __init__(self):
        super().__init__('head_turn_at_waypoint')
        self.movement_completed = False
        self.is_response_ready = False
        self.rate = self.create_rate(2)
        self._action_client = ActionClient(self, PlayMotion2, 'play_motion2')
        self.srv = self.create_service(Trigger, 'turn_head',
                                       self.turn_head_callback)

    def turn_head_callback(self, request, response):
        self.send_goal()
        while not self.is_response_ready:
            self.rate.sleep()
        response.success = self.movement_completed
        response.message = "Service completed!"
        return response

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
        self.is_response_ready = True
        result = future.result().result
        self.movement_completed = result.success
        p_str = 'Movement finished with success: {0}'
        self.get_logger().info(p_str.format(self.movement_completed))
        if self.movement_completed is False:
            p_str = 'Error. Cannot procced further.'
            self.get_logger().error(p_str)
        elif self.movement_completed is True:
            p_str = 'Movement completed. Proceed to the next waypoint...'
            self.get_logger().info(p_str)
        else:
            p_str = 'Error. Cannot procced further. Shutting down...'
            self.get_logger().error(p_str)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = HeadTurnAtWaypoint()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
