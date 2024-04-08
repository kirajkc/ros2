#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import numpy as np

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        self.bridge = CvBridge()
        self.color_publisher = self.create_publisher(String, 'traffic_light_topic', 10)
        self.model = YOLO("yolov8n.pt")
        self.cap = cv2.VideoCapture(0)

    def detect_traffic_light(self, frame):
        results = self.model(frame, classes=[9], imgsz=256, vid_stride=2)
        for result in results:
            if result.boxes:
                xywh = result.boxes.xywh.tolist()[0]
                
                x = int(xywh[0])
                y = int(xywh[1])
                w = int(xywh[2])
                h = int(xywh[3])

                # roi = frame[y: y+h,x: x+w]
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                mask_red = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
                mask_green = cv2.inRange(hsv, np.array([45, 100, 50]), np.array([75, 255, 255]))
                mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))

                red_pixels = cv2.countNonZero(mask_red)
                green_pixels = cv2.countNonZero(mask_green)
                yellow_pixels = cv2.countNonZero(mask_yellow)

                if red_pixels > green_pixels and red_pixels > yellow_pixels:
                    color = "red"
                elif green_pixels > red_pixels and green_pixels > yellow_pixels:
                    color = "green"
                elif yellow_pixels > red_pixels and yellow_pixels > green_pixels:
                    color = "yellow"
                else:
                    color = "unknown"
                msg = String()
                msg.data = color
                self.color_publisher.publish(msg)
        # annotated_frames = results[0].plot()
        # cv2.imshow('frame', annotated_frames)

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                self.detect_traffic_light(frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        self.color_publisher.destroy()

def main(args=None):
    rclpy.init(args=args)
    traffic_light_detector = TrafficLightDetector()
    try:
        traffic_light_detector.run()
    finally:
        traffic_light_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
