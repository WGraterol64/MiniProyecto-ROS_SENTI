#!/usr/bin/env python3
from email.mime import audio
from unittest import result
import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import argparse
from time import time
import audio 
import images 
import multimodal 
from multimodal_dependencies import FusionTransformer, EmbraceNet, Wrapper
import ontologies
from random import randint

soundFile = '/usr/src/pepper_sim_ws/src/pepper_robot/pepper_sensors_py/nodes/soundfile.txt'
names = ['Jaime', 'Andy', 'Jade']
topic = ['La Gioconda', 'A Starry Night', 'Guernica']

def parse_pred(pred):
  if pred[0] < 0.5 and pred[1] < 0.5:
    return "neutral", 1.0
  elif pred[0] >= 0.5:
    return "happy", pred[0]
  else:
    return "sad", pred[1]

class image_converter:
  def __init__(self, emotion):
    # Create the ROS subscriber node that gets the image from the pepper bot's camera
    self.image_sub = rospy.Subscriber("/pepper/camera/ir/image_raw", Image, self.callback)
    self.emotion = emotion
    self.average_performance_sum = 0
    self.average_time_sum = 0
    self.average_samples = 0
    self.start = 0

  def callback(self, image_data):
    # Convert imagen from sensor_msgs/Image format to cv2 format
    cv_image = np.frombuffer(
      image_data.data, 
      dtype=np.uint8
    ).reshape(image_data.height, image_data.width, -1)

    # Get sound path
    with open(soundFile, 'r') as f:
      path = f.read()

    # Show imagen
    cv2.imshow("Image window", cv_image)
    if cv2.waitKey(3) & 0xFF == ord('m'):
      self.start = (self.start+1)%2
    if self.start == 1:
      if self.emotion == '':
        pred = multimodal.predict_multimodal(cv_image, path)
        print(pred)
      else:
        start_pred_time = time()
        pred = multimodal.predict_multimodal(cv_image, path)
        end_pred_time = time()
        self.average_time_sum += end_pred_time-start_pred_time
        parsed_emotion, intensity = parse_pred(pred)
        if parsed_emotion == self.emotion:
          self.average_performance_sum += 1
        self.average_samples += 1
        print('Tiempo promedio de preficciones (Segundos): {0}\nPrecision: {1}\nNumero de muestras: {2}'.format(self.average_time_sum/self.average_samples, self.average_performance_sum/self.average_samples, self.average_samples))      
        ontologies.add_instance(names[randint(0,len(names)-1)], parsed_emotion, 'event'+str(self.average_samples), intensity, topic[randint(0,len(names)-1)])
        

def main(emotion):
  # Create and initialize ROS node
  ic = image_converter(emotion)
  rospy.init_node('pepper_camera', anonymous=True)

  # The node keeps listening to the camera topic until the process is canceled
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    
  cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--emotion", type=str, help="emotion to test", default='')
    arg = parser.parse_args()
    main(arg.emotion)
