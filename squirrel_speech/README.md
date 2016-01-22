# squirrel_speech 
  
Everything related to the speech recongition.  
Based on 
'Zhang, A. (2015). Speech Recognition (Version 3.1) [Software]'
Available from <https://github.com/Uberi/speech_recognition#readme>

It is possible to use various online APIs to perform a speech recognition. The program records a audiofile and transmits it to the recongition service. the response is then used to gemerate commands for the robot. 

(An offline solution based on CMU Sphinx is under development, but so far, the recognition rate is inferior to the online APIs)

## Usage ##

`rosrun squirrel_speech_rec sq_ros_speech_rec`
Starts a rosnode which listens to the microphone and sends requests to the google speech API. The result is then published to a topic.

`rosrun squirrel_speech_rec sq_ros_speech_parser`
Starts a rosnode which checks the published message for known commands (for example: "Roboter gehe links"/"Robot go left"). If a command is recognized, a new message is published to a different topic.  



## Installation ##
`sudo apt-get install python-pyaudio python3-pyaudio`

For 64 bit system only:
`sudo apt-get install flac` 

For all:
`sudo apt-get multimedia-jack`
select yes in the dialog window.
This can also be changed later with: 
`sudo dpkg-reconfigure -p high jackd2` and select "Yes" to do so.





## Troubleshooting ##

### "jack server is not running or cannot be started" or "Cannot lock down [...] byte memory area (Cannot allocate memory)" ###

Check if current user is in the ``audio`` group. 
Add your current user with ``sudo adduser $(whoami) audio``. 

reboot the system.

``pulseaudio --kill``
``jack_control start``



## Record Audio ##
Show available Audio devices:
* for recording
`aplay -l`
* for playing
`arecord -l`

Recoding a file in current directory named test.wav
`arecord -f S16_LE -r 16000 -d 5 -D hw:0,0 test.wav`
you may need to adjust the length of the recording (`-d` in seconds) and the audio device (`-D`).

Play the file
`aplay test.wav`



## Debugging audio devices ##
Test your speakers
`speaker-test -p 500 -D hw:1,0 -c 2`

Use audacity
`sudo apt-get install audacity`
You can generate various sinus and noise signals and change between devices and test them.

Trivial
* Check if headset/sound card is plugged in. 
* Check if headset is muted.
* Check if headset is connected.
 


### Headset ###
Recommendation for use with robotino: Logitech G930 (wireless)
The robotino does not have a sound card, therefore a USB sound card is required to handle audio signals. The mentioned Headset has a built-in USB sound card and is tested to work with the robotino system. 
In general, the used audio device should support at least a rate of 16kHz.


 
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

