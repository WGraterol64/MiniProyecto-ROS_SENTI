from sentence_transformers import SentenceTransformer
import pandas as pd
import numpy as np
import pickle
import speech_recognition as sr

models_json = {'roberta-large-nli-stsb-mean-tokens':SentenceTransformer('roberta-large-nli-stsb-mean-tokens'),
'distilbert-base-nli-mean-tokens':SentenceTransformer('distilbert-base-nli-mean-tokens'),
'bert-large-nli-stsb-mean-tokens':SentenceTransformer('bert-large-nli-stsb-mean-tokens')
}

class_json = {'roberta-large-nli-stsb-mean-tokens':pickle.load(open('models/Audio/roberta-large-nli-stsb-mean-tokens.clf','rb')),
'distilbert-base-nli-mean-tokens':pickle.load(open('models/Audio/distilbert-base-nli-mean-tokens.clf','rb')),
'bert-large-nli-stsb-mean-tokens':pickle.load(open('models/Audio/bert-large-nli-stsb-mean-tokens.clf','rb')),
'Meta_learner':pickle.load(open('models/Audio/Meta_learner.clf','rb'))
}
def detect_emotion(sentences):
    # We do the predictions this way to avoid problems with the memory restrictions
    models = ['roberta-large-nli-stsb-mean-tokens', 'distilbert-base-nli-mean-tokens', 'bert-large-nli-stsb-mean-tokens']
    embeddings = []
    classifiers = []
    for model_name in models :
        model = SentenceTransformer(model_name)
        sentence_embeddings = model.encode(sentences)
        # We use the same models trained in the experimental phase
        clf = pickle.load(open('models/Audio/{0}.clf'.format(model_name),'rb'))
        embeddings.append(clf.predict(sentence_embeddings))

    weak_predictions = [np.concatenate((embeddings[0][i], np.concatenate((embeddings[1][i], embeddings[2][i]), axis=0)), axis=0) for i in range(len(embeddings[0]))]
    clf = pickle.load(open('models/Audio/{0}.clf'.format('Meta_learner'),'rb'))
    return clf.predict(weak_predictions)

def extract_audio(filepath):
    r = sr.Recognizer()
    with sr.AudioFile(filepath) as source:
        audio = r.record(source)
    try:
        s = r.recognize_google(audio)
        return s
    except Exception as e:
        print("Exception: " + str(e))
        return 'ERROR'

def predict_emotions_audio(filepath, vector_multimodal = False):
    mapper = {
        0: "anger",
        1: "anticipation",
        2: "disgust",
        3: "fear",
        4: "joy",
        5: "love",
        6: "optimism",
        7: "pessimism",
        8: "sadness",
        9: "surprise",
        10: "trust"
    }
    s = ''
    phrase = extract_audio(filepath)
    if phrase == 'ERROR':
        return 'ERROR'

    prediction = detect_emotion([phrase])[0]

    if not vector_multimodal:
        for i in range(len(prediction)):
            if prediction[i] == 1:
                s += mapper[i] + '; '
                
        return s
    else:
        return [prediction[4], prediction[8]]
