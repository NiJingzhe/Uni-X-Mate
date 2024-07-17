import speech_recognition as sr
import spacy
from spacy.matcher import PhraseMatcher
# 个人日常用品,家具，动物词汇表
categories = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

class V2T:

    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.nlp = spacy.load("en_core_web_sm")
        self.matcher = PhraseMatcher(self.nlp.vocab)
        temp_patterns = [self.nlp(text) for text in categories]
        self.matcher.add("PHRASES", temp_patterns)

    def recognize_speech_from_mic(self):
        """Transcribe speech from recorded from `microphone`."""
        if not isinstance(self.recognizer, sr.Recognizer):
            raise TypeError("`recognizer` must be `speech_recognition.Recognizer` instance")
        if not isinstance(self.microphone, sr.Microphone):
            raise TypeError("`microphone` must be `speech_recognition.Microphone` instance")

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            print("Listening...")
            audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
            print("OK")

        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        try:
            response["transcription"] = self.recognizer.recognize_google(audio)
        except sr.RequestError:
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            response["error"] = "Unable to recognize speech"

        return response

    def extract_objects(self, text):
        """Extract objects from text."""
        doc = self.nlp(text)
        matches = self.matcher(doc)
        objects = [doc[start:end].text for match_id, start, end in matches]
        return objects

    def run(self):
        print("Say something:")
        result = self.recognize_speech_from_mic()
        if result["success"] and result["transcription"]:
            print("You said: " + result["transcription"])
            objects = self.extract_objects(result["transcription"])
            objects = list(set(objects))
            print("Objects in text:", objects)
            return {"objects" : objects, "content" : result["transcription"]}
        else:
            print("I didn't catch that. Error: " + result["error"])
            return None