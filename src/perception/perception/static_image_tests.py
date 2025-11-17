import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class perception_example(Node):
    def __init__(self):
        super().__init__('perception_example')
        self.bridge = CvBridge()
        self.model = YOLO('/home/jacob/MTRN4231_sandwich_assembler/src/perception/perception/burger_model.pt')
        self.model.verbose = False
        self.subscription = self.create_subscription(Image, 'camera/camera/color/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)
        annotated_image = results[0].plot()

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for i, box in enumerate(boxes):
                    if box.conf[0] > 0.9:
                        # annotated_image = cv_image
                        centroid_x, centroid_y, w, h = box.xywh.numpy()[0]
                        centroid_x = int(centroid_x)
                        centroid_y = int(centroid_y)
                        w = int(w)
                        h = int(h)
                        cv2.circle(annotated_image, (centroid_x, centroid_y), 5, (255, 0, 0), -1)
                        text = f"({centroid_x}, {centroid_y})"
                        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        text_x = centroid_x - text_size[0] // 2
                        text_y = centroid_y + 20
                        cv2.rectangle(annotated_image, (text_x, text_y - text_size[1] - 2), (text_x + text_size[0], text_y + 2), (0, 0, 0), -1)
                        cv2.putText(annotated_image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow('Segmented Image with Centroid', annotated_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    perception_example_node = perception_example()
    rclpy.spin(perception_example_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()