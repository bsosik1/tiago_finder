import rclpy
from playsound import playsound
import gtts
import math
import os
from rclpy.node import Node
from std_msgs.msg import String
from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import InferenceResult
from std_msgs.msg import Bool


def convert_to_xyxy(x_center, y_center, width, height):
    x1, y1 = x_center-width/2, y_center-height/2
    x2, y2 = x_center+width/2, y_center+height/2
    return x1, y1, x2, y2


def calculate_area_of_overlap(o: InferenceResult, f: InferenceResult):
    # OBJECT
    x1_o, y1_o, x2_o, y2_o = convert_to_xyxy(
        o.x_center, o.y_center, o.width, o.height)
    # FURNITURE
    x1_f, y1_f, x2_f, y2_f = convert_to_xyxy(
        f.x_center, f.y_center, f.width, f.height)
    # CHECK IF BOUNDING BOXES ARE INTERSECTING
    if_overlap = True
    if x1_o > x2_f or x1_f > x2_o:
        if_overlap = False
    if y2_o < y1_f or y2_f < y1_o:
        if_overlap = False

    # OBJECT AREA
    o_area = o.width*o.height
    # AREA OF OVERLAP
    x1 = max(x1_o, x1_f)
    x2 = min(x2_o, x2_f)
    y1 = max(y1_o, y1_f)
    y2 = min(y2_o, y2_f)
    overlap = ((x2-x1)*(y2-y1))/o_area*100
    if if_overlap is False:
        overlap = 0
    return overlap


def relative_positon(o: InferenceResult, f: InferenceResult):
    overlap = calculate_area_of_overlap(o, f)
    # rel_height = o.height/f.height
    # OBJECT
    x1_o, y1_o, x2_o, y2_o = convert_to_xyxy(
        o.x_center, o.y_center, o.width, o.height)
    # FURNITURE
    x1_f, y1_f, x2_f, y2_f = convert_to_xyxy(
        f.x_center, f.y_center, f.width, f.height)
    match f.class_name:
        case "coffee-table":
            if overlap == 0:
                if o.x_center > x2_f:
                    return "on the right of"
                if o.x_center < x1_f:
                    return "on the left of"
            if y2_o < f.y_center:
                return "on"
            if y2_o > y2_f:
                return "in front of"
            if o.x_center > x2_f:
                return "on the right of"
            if o.x_center < x1_f:
                return "on the left of"
            else:
                return "near"
        case w if w in ["bookshelf", "rack", "wardrobe"]:
            if overlap > 80 and o.y_center < y2_f:
                return "on"
            if y2_o > y2_f:
                return "in front of"
            if o.x_center > x2_f:
                return "on the right of"
            if o.x_center < x1_f:
                return "on the left of"
            else:
                return "near"
        case "table":
            if y2_o < f.y_center:
                return "on"
            if overlap > 95:
                return "under"
            if y2_o > y2_f:
                return "in front of"
            if o.x_center > x2_f:
                return "on the right of"
            if o.x_center < x1_f:
                return "on the left of"
            else:
                return "near"
        case "chair":
            if y2_o < f.y_center+f.height*0.16:
                return "on"
            if overlap > 95:
                return "under"
            if y2_o > y2_f:
                return "in front of"
            if o.x_center > x2_f:
                return "on the right of"
            if o.x_center < x1_f:
                return "on the left of"
            else:
                return "near"
        case "sofa":
            if overlap == 100:
                return "on"
            if y2_o > y2_f:
                return "in front of"
            if o.x_center > f.x_center:
                return "on the right of"
            if o.x_center < f.x_center:
                return "on the left of"
            else:
                return "near"


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
        furniture = ["bookshelf", "chair", "coffee-table", "rack", "sofa",
                     "table", "wardrobe"]
        object_point = None
        object_index = None
        furniture_index = None
        smallest_dist = math.inf
        if not self.goal_reached:
            # CHECK IF OBJECT IS VISIBLE
            for index, item in enumerate(msg.yolov8_inference):
                if item.class_name == self.goal:
                    if item.confidence > 0.7:
                        self.goal_reached = True
                        object_index = index
                        object_point = [item.x_center, item.y_center]
                        print(object_point)
                        temp_str = "Found the {}".format(self.goal)
                        pub = Bool()
                        pub.data = True
                        self.object_found_publisher.publish(pub)
                        self.get_logger().info(temp_str)
                        self.get_logger().info("-----------------------------")
            # FIND CLOSEST FURNITURE
            for index, item in enumerate(msg.yolov8_inference):
                if not self.goal_reached:
                    break
                if item.class_name in furniture:
                    furniture_point = [item.x_center, item.y_center]
                    distance = math.dist(object_point, furniture_point)
                    self.get_logger().info(f'Furniture: {item.class_name}')
                    self.get_logger().info(f'distance: {distance}')
                    if distance < smallest_dist:
                        smallest_dist = distance
                        furniture_index = index
            if self.goal_reached:
                obj = msg.yolov8_inference[object_index]
                fur = msg.yolov8_inference[furniture_index]
                overlap = calculate_area_of_overlap(obj, fur)
                rel_pos = relative_positon(obj, fur)
                self.get_logger().info("-----------------------------")
                self.get_logger().info(f'Coverage: {overlap}%')
                self.get_logger().info(f'Smallest dist: {smallest_dist}')
                self.get_logger().info(f'Closest furniture: {fur.class_name}')
                self.get_logger().info(f'Rel pos: {rel_pos}')
                temp_str = f'The {self.goal} is {rel_pos} the {fur.class_name}'
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
