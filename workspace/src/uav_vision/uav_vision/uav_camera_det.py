 #!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
from ultralytics import YOLO 

# Load the YOLOv8 model
model = YOLO('yolo_model_person.pt')
print("Załadowane klasy modelu:", model.names)

class ImageSubscriber(Node):

  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      'camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription   
    self.br = CvBridge()
   
  def listener_callback(self, data):
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    image = current_frame

    # Object Detection
    results = model.predict(image, classes=[0, 2])
    img = results[0].plot()

    # Show Results
    cv2.imshow('Detected Frame', img)    
    cv2.waitKey(1)
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
