import rclpy
from playsound import playsound
import gtts
import os
from rclpy.node import Node
from std_msgs.msg import String
from yolov8_msgs.msg import Yolov8Inference
from std_msgs.msg import Bool


class Goal_Manager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        self.goal_subscriber = self.create_subscription(
            String, '/intention', self.goal_sub_callback, 10)

        self.yolo_subscriber = self.create_subscription(
            Yolov8Inference, '/YOLO_result', self.yolo_sub_callback,
            10)

        self.object_found_publisher = self.create_publisher(
            Bool, '/object_found', 10)

        self.goal = None
        self.goal_reached = False

    def goal_sub_callback(self, msg: String):
        self.get_logger().info("Received goal: {}".format(msg.data))
        self.goal = msg.data
        self.goal_reached = False

    def yolo_sub_callback(self, msg: Yolov8Inference):
        if not self.goal_reached:
            for element in msg.yolov8_inference:
                if element.class_name == self.goal:
                    if element.confidence > 0.7:
                        self.goal_reached = True
                        temp_str = "Found the {}".format(self.goal)
                        pub = Bool()
                        pub.data = True
                        self.object_found_publisher.publish(pub)
                        self.get_logger().info(temp_str)
                        tts = gtts.gTTS(temp_str, lang='en')
                        tts.save('read_this_2.mp3')
                        playsound('read_this_2.mp3')
                        os.remove('read_this_2.mp3')


def main(args=None):
    rclpy.init(args=None)
    node = Goal_Manager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
