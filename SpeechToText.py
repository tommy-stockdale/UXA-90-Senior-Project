
import speech_recognition as sr

r = sr.Recognizer()

while True:
    i = input("Proceed?")
    with sr.Microphone() as source:
        print("Talk")
        audio_data = r.record(source, duration=5)
        print("Recognizing")
        text = r.recognize_google(audio_data)
        print(text)
        print(type(text))

    f = open("C:/Users/tommy/source/repos/ReadCommandFile/ReadCommandFile/commands.txt", "a")
    f.write(text + '\n')
    f.close()

