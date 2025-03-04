#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)  # Publish frames at ~10 FPS
        self.cap = cv2.VideoCapture('/home/miriam/Downloads/video.mp4')
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video file.')
            self.cap.release()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV frame to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing video frame...')
        else:
            self.get_logger().info('End of video reached.')
            self.cap.release()
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()

    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        video_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
