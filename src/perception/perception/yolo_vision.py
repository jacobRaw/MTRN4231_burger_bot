"""
@file perception/yolo_vision.py
@brief Node that uses a custom YOLO model to perform object detection on RGB images 
       from Realsense camera and estimates 3D coordinates using depth data.
@author Jacob Rawung
@usage ros2 launch perception perception.launch.py
"""

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import CameraInfo
from custom_interfaces.msg import Ingredients, IngredientPos
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from ament_index_python.packages import get_package_share_directory
import os

class perception_example(Node):
    def __init__(self):
        super().__init__('perception_example')

        #CV setup
        self.bridge = CvBridge()
        
        # Load YOLO model
        share_dir = get_package_share_directory('perception')
        # self.model = YOLO(os.path.join(share_dir, 'burger_model.pt'))
        self.model = YOLO(os.path.join(share_dir, 'black_seed.pt'))
        #self.model = YOLO('/home/jacob/MTRN4231_sandwich_assembler/src/perception/perception/black_seed.pt')

        self.model.verbose = False

        # Subscribers
        self.cam_sub = self.create_subscription(RGBD, '/camera/camera/rgbd', self.image_callback, 10)
        self.cam_sub  # prevent unused variable warning
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.cam_info_callback, 10)
        self.cam_info_sub  # prevent unused variable warning

        # Publishers
        self.ingredient_pub = self.create_publisher(Ingredients, '/ingredients', 10)

        # class attributes
        self.fx = self.fy = self.cx = self.cy = None
        self.cam_setup = False

        # static transformation listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # cropping parameters
        self.x_crop = 100

        # offsets 
        self.x_offset = -0.01700 + 0.015
        self.y_offset = -0.01425

    def image_callback(self, msg):
        if not self.cam_setup:
            return
        image_raw = msg.rgb #colour image
        depth_image = np.frombuffer(msg.depth.data, dtype=np.uint16)
        depth_image = depth_image.reshape(msg.depth.height, msg.depth.width)
        cv_image = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')
        cv_image = cv_image[0: 480, self.x_crop : 640]
        results = self.model(cv_image, verbose=False)
        annotated_image = results[0].plot()
        # annotated_image = cv_image
        ingredients_msg = Ingredients()
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for i, box in enumerate(boxes):
                    # get class name
                    class_id = int(box.cls[0])
                    ingredient = self.model.names[class_id]
                    X, Y, Z = self.annotate_image(annotated_image, depth_image, box)
                    ingredient_pos = IngredientPos()
                    ingredient_pos.ingredient = ingredient
                    ingredient_pos.pos = [X, Y, Z]
                    ingredients_msg.ingredients.append(ingredient_pos)

        self.ingredient_pub.publish(ingredients_msg)
        cv2.imshow('Segmented Image with Centroid', annotated_image)
        cv2.waitKey(1)
    
    def annotate_image(self, rgb_image, depth_image, box):
        centroid_x, centroid_y, w, h = box.xywh.numpy()[0]
        centroid_x = int(centroid_x)
        centroid_y = int(centroid_y)
        w = int(w)
        h = int(h)
        cv2.circle(rgb_image, (centroid_x, centroid_y), 5, (255, 0, 0), -1)
        uncropped_centroid_x = centroid_x + self.x_crop
        uncropped_centroid_y = centroid_y
        depth = depth_image[uncropped_centroid_y, uncropped_centroid_x] / 1000.0  # Convert mm to meters
        X = (uncropped_centroid_x - self.cx) * depth / self.fx
        Y = (uncropped_centroid_y - self.cy) * depth / self.fy
        Z = depth

        # convert from camera_pos to base_pos
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_link'
        point_cam.header.stamp = rclpy.time.Time().to_msg()
        point_cam.point.x = float(X)
        point_cam.point.y = float(Y)
        point_cam.point.z = float(Z)

        try:
            point_base = self.tf_buffer.transform(
                point_cam,
                'base_link',
                timeout=rclpy.duration.Duration(seconds=0.3)
            )

        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")


        point_base.point.x += self.x_offset
        point_base.point.y += self.y_offset
        text = f"(X:{point_base.point.x * 1000:.1f}, Y:{point_base.point.y * 1000:.1f}, Z:{point_base.point.z * 1000:.1f})"
        # text = f"(X:{centroid_x:.3f}, Y:{centroid_y:.3f})"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        text_x = centroid_x - text_size[0] // 2
        text_y = centroid_y + 20
        cv2.rectangle(rgb_image, (text_x, text_y - text_size[1] - 2), (text_x + text_size[0], text_y + 2), (0, 0, 0), -1)
        cv2.putText(rgb_image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return point_base.point.x, point_base.point.y, point_base.point.z

    def cam_info_callback(self, msg):
        self.get_logger().info('Camera info received.')
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.cam_setup = True
        # Unsubscribe after getting camera info once
        self.destroy_subscription(self.cam_info_sub)

def main():
    rclpy.init()
    perception_example_node = perception_example()
    rclpy.spin(perception_example_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()