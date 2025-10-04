#!/bin/bash

sudo modprobe apple_bce

echo "Starting PipeWire services..."
systemctl --user start pipewire.service # pipewire.socket
#systemctl start pipewire.service pipewire.socket


echo "Starting PulseAudio services..."
#systemctl --user start pulseaudio.service pulseaudio.socket
#sudo systemctl start pulseaudio.service pulseaudio.socket

echo "Starting WirePlumber service..."
systemctl --user start wireplumber.service
#sudo systemctl start wireplumber.service


sudo service tiny-dfr start
