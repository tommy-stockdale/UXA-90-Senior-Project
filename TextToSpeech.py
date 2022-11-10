from gtts import gTTS

import os

mytext = "My name is Ratchet"
f = open("C:/Users/tommy/source/repos/ReadCommandFile/ReadCommandFile/commands.txt", "r")
commands = f.read()

language = 'en'

myobj = gTTS(text = mytext, lang = language, slow = False)
myobj.save("name.mp3")

os.system("name.mp3")