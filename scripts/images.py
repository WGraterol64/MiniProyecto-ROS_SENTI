import numpy as np
import pandas as pd
import tensorflow as tf
import cv2

model_images = tf.keras.models.load_model('models/Facial/model.h5')
mapper = {
    0: "happy",
    1: "sad",
    2: "neutral",
}

mapper_vector = {
    0: [1,0],
    1: [0,1],
    2: [0,0],
}

def predict_emotion_image(img_array, vector_multimodal = False):
  resized_image = cv2.resize(cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY), (48, 48)).reshape(48, 48, 1).astype('float32')
  pred_var = (model_images.predict((resized_image/255).reshape(1,48,48,1))[0]).tolist()
  pred_index = pred_var.index(max(pred_var))
  if not vector_multimodal:
    return mapper[pred_index]
  else:
    return mapper_vector[pred_index]