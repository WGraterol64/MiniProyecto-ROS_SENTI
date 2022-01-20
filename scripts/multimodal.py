import numpy as np
import pandas as pd
import torch
import cv2
import pickle
import audio 
import images 
from multimodal_dependencies import FusionTransformer, EmbraceNet, Wrapper

def predict_multimodal(img_array, filepath, verbose = True):
    image_prediction_vector = images.predict_emotion_image(img_array, vector_multimodal = True)
    text_prediction_vector = audio.predict_emotions_audio(filepath, vector_multimodal = True)
    with open('models/Fusion/fusion_model.pt', 'rb') as dic:
        multimodal_model = pickle.load(dic)
    output = multimodal_model(torch.from_numpy(np.array([image_prediction_vector])).float(), 
                     torch.from_numpy(np.array([[0., 0.]])).float(), 
                     torch.from_numpy(np.array([text_prediction_vector])).float(), 
                     torch.from_numpy(np.array([[1., 0., 1.]])).float()).tolist()[0]
    if verbose:
        print('Probabilidades:\nFeliz: {0}\nTriste: {1}'.format(output[0], output[1]))
    return output

