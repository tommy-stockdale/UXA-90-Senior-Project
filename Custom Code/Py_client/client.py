import pyttsx3
import speech_recognition as sr
from socket import *
import sys

commands = {"turn left": "turn_left", 
"turn right": "turn_right", 
"walk right":"walk_right", 
"walk forward":"walk_forward_short", 
"walk forward 4 steps":"walk_foward_4step", 
"basic motion": "basic_motion", 
"introduce yourself": "demo_introduction", 
"dance":"dance_gangnamstyle", "stop":"stop", 
"sit down":"sit_down", 
"kick":"kick right"}
#print(commands["turn left"])
HOST, PORT = "10.80.32.207", 5000
print("connecting")
s = socket(AF_INET, SOCK_STREAM)

s.connect((HOST, PORT))
print("connected")
s.send(b'Test')

def speak(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()

def get_audio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        said = ""
        try:
            said = r.recognize_google(audio)
        except Exception as e:
            print("Exception: " + str(e))
    return said.lower()

if __name__ == "__main__":
    wake = "hey ratchet"
    while True:
        print("Listening")
        speak("I'm listening")
        text = get_audio()
        print(text)
        if text == "goodbye":
                speak("see you later")
                break
        if text == wake:
            print("listening 2:")
            speak("What can I help you, sir?")
            text3 = get_audio()
            print(text3)
            if text3 in commands:
                s.send(commands[text3].encode())
            else:
                speak("command is not valid")
            print('reloop')
    s.close()