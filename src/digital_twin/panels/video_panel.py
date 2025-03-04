from PyQt5.QtWidgets import QLabel, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rviz2.panel import Panel

class VideoPanel(Panel):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('VideoPanel')

        # Layout and QLabel for displaying video
        self.layout = QVBoxLayout()
        self.label = QLabel('Loading video...')
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        # ROS 2 setup
        self.node = context.node
        self.bridge = CvBridge()
        self.subscription = self.node.create_subscription(
            Image,
            'video_frames',  # Topic name
            self.callback,
            10
        )

    def callback(self, msg):
        # Convert ROS image to QPixmap
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width, channel = frame.shape
            bytes_per_line = channel * width
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(q_image)
            self.label.setPixmap(pixmap)
        except Exception as e:
            self.label.setText(f'Error displaying video: {str(e)}')

