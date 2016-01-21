echo "Reset Audio for Usage of Headset"
pulseaudio --kill
jack_control start
