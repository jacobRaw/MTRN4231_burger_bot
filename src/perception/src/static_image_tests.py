import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class perception_example(Node):
    def __init__(self):
        super().__init__('perception_example')
        self.bridge = CvBridge()
        self.model = YOLO('/home/jacob/MTRN4231/lab4-main/runs/detect/train2/weights/best.pt')
        self.model.verbose = False
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)
        
        annotated_image = results[0].plot()

        for result in results:
            masks = result.masks
            if masks is not None:
                for i, mask in enumerate(masks):
                    if result.names[int(result.boxes[i].cls[0])] == 'cell phone':
                        binary_mask = mask.data.cpu().numpy().squeeze()
                        
                        contours, _ = cv2.findContours(binary_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                        if contours:
                            largest_contour = max(contours, key=cv2.contourArea)
                            M = cv2.moments(largest_contour)
                            if M["m00"] != 0:
                                centroid_x = int(M["m10"] / M["m00"])
                                centroid_y = int(M["m01"] / M["m00"])
                                
                                cv2.circle(annotated_image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

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