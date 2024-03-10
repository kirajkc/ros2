import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class QRCodePublisher(Node):
    def __init__(self):
        super().__init__('qrcode_publisher')
        self.bridge = CvBridge()
        self.qr_pub = self.create_publisher(String, 'qr_data', 10)

    def publish_qr_data(self, decoded_info):
        # Publish the QR code data on the topic
        qr_data_msg = String()
        qr_data_msg.data = decoded_info
        self.qr_pub.publish(qr_data_msg)

def main(args=None):
    rclpy.init(args=args)
    qrcode_publisher = QRCodePublisher()

    # OpenCV VideoCapture for camera
    vid = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()

    while rclpy.ok():
        # Capture the video frame by frame
        ret, frame = vid.read()
        data, bbox, straight_qrcode = detector.detectAndDecode(frame)

        if len(data) > 0:
            # Print the QR code data
            print(data)

            # Publish the QR code data on the ROS2 topic
            qrcode_publisher.publish_qr_data(data)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # The 'q' button is set as the quitting button
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()

    # Destroy all the windows
    cv2.destroyAllWindows()

    qrcode_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
