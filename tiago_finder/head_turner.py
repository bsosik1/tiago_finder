import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from play_motion2_msgs.action import PlayMotion2


class HeadTurner(Node):
    def __init__(self):
        super().__init__('head_turner')
        self._action_client = ActionClient(self, PlayMotion2, 'play_motion2')

    def send_goal(self):
        msg = PlayMotion2.Goal()
        msg.motion_name = "turn_head"
        msg.skip_planning = False

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = HeadTurner()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
