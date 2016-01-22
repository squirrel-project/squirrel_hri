# squirrel_speech # 
  
Everything related to the speech recongition.  
Based on 
	Zhang, A. (2015). Speech Recognition (Version 3.1) [Software]. Available from <https://github.com/Uberi/speech_recognition#readme>.

Links:

-  `PyPI <https://pypi.python.org/pypi/SpeechRecognition/>`__
-  `GitHub <https://github.com/Uberi/speech_recognition>`__

## Usage ##





## Troubleshooting ##
If you should expirience the following Error message 
"jack server is not running or cannot be started"


### "jack server is not running or cannot be started" or "Cannot lock down [...] byte memory area (Cannot allocate memory)" ###

Check if current user is in the ``audio`` group. 
Add your current user with ``sudo adduser $(whoami) audio``.
sudo adduser robotino audio

reboot the system.

``pulseaudio --kill``, followed by ``jack_control start``, to fix the "jack server is not running or cannot be started" error.





## Installation ##
sudo apt-get install python-pyaudio python3-pyaudio 

for 64 bit system
sudo apt-get install flac 

sudo apt-get multimedia-jack - select yes in the dialog window.
LAter COnfiguration: 
`sudo dpkg-reconfigure -p high jackd2` and select "Yes" to do so.




## Record stuff ##
show available Audio devices:
-for recording
aplay -l 
-for playing
arecord -l

Make a test recoding in current directory
arecord -f S16_LE -r 16000 -d 5 -D hw:0,0 test.wav

Play the recording
aplay test.wav

Test your speakers
test speaker 
speaker-test -p 500 -D hw:1,0 -c 2

### Debugging audio devices ###
sudo apt-get install audacity
You can generate various sinus and noise signals
change between devices and test them
test the recording 



### HEADSET ###
Recommendation for use with robotino: Logitech G930 (wireless)
The robotino does not have a sound card, therefore a USB sound card is required to handle audio signals. The mentioned Headset has a built-in USB sound card and is tested to work with the robotino system. 
* Check if it is plugged in. 
* Check if it is muted.
 
### Languages ###
Languages supported by Google (excerpt):

* Dutch nl-NL
* English(UK) en-GB
* English(US) en-US
* Finnish fi
* French fr-FR
* German de-DE
* Italian it-IT
* Japanese ja
* Korean ko
* Mandarin Chinese zh-CN
* Norwegian no-NO
* Polish pl
* Portuguese pt-PT
* Russian ru
* Spanish(Spain) es-ES
* Swedish sv-SE
* Turkish tr

