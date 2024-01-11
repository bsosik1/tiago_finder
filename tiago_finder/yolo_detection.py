import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()
path_pref = '/home/bsosik/tiago_ros2_ws/src/tiago_finder/config/yolo_models/'
chosen_model = 'best_w_furniture.pt'
path = path_pref + chosen_model


class Camera_To_YOLO(Node):
    def __init__(self):
        super().__init__('camera_to_YOLO')

        # YOLO model
        self.model = YOLO(path)
        # YOLO result
        self.yolo_result = Yolov8Inference()
        # Camera subscriber
        self.cam_sub = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.cam_callback,
            10)
        # YOLO result publisher
        self.yolo_pub = self.create_publisher(
            Yolov8Inference, '/YOLO_result', 1)
        # Image publisher
        self.img_pub = self.create_publisher(
            Image, '/YOLO_result_cv2', 1)

    def cam_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        result = self.model(img)

        # HEADER
        self.yolo_result.header.frame_id = "result"
        self.yolo_result.header.stamp = self.get_clock().now().to_msg()

        # BOXES
        for element in result:
            boxes = element.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xywh[0].to('cpu').detach().numpy().copy()
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.confidence = float(box.conf)
                self.inference_result.x_center = int(b[0])
                self.inference_result.y_center = int(b[1])
                self.inference_result.width = int(b[2])
                self.inference_result.height = int(b[3])
                self.yolo_result.yolov8_inference.append(self.inference_result)

        annotated_frame = result[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)

        self.img_pub.publish(img_msg)
        self.yolo_pub.publish(self.yolo_result)
        self.yolo_result.yolov8_inference.clear()


def main(args=None):
    rclpy.init(args=None)
    node = Camera_To_YOLO()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
