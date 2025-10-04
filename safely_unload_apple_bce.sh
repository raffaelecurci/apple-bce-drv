#!/bin/bash

# Check if tiny-dfr service is running
if systemctl is-active --quiet tiny-dfr; then
    echo "tiny-dfr service is running. Stopping it now..."
    sudo service tiny-dfr stop
else
    echo "tiny-dfr service is not running."
fi

echo "Stopping PipeWire services..."
systemctl --user stop pipewire.service pipewire.socket
#sudo systemctl stop pipewire.service pipewire.socket


echo "Stopping PulseAudio services..."
systemctl --user stop pulseaudio.service pulseaudio.socket
#sudo systemctl stop pulseaudio.service pulseaudio.socket

echo "Stopping WirePlumber service..."
systemctl --user stop wireplumber.service
#sudo systemctl stop wireplumber.service

echo "All specified services have been stopped."

# Filter modules with 'snd' prefix from lsmod, usage count in 3rd column
while true; do
    # Get modules with usage count 0
    zero_usage_modules=($(lsmod | grep '^snd' | awk '$3 == 0 {print $1}'))

    if [ ${#zero_usage_modules[@]} -eq 0 ]; then
        echo "No more snd modules with zero usage to remove."
        break
    fi

    for mod in "${zero_usage_modules[@]}"; do
        echo "Removing module: $mod"
        sudo modprobe -r "$mod"
        # Optional: check if removal succeeded
        if lsmod | grep -q "^$mod"; then
            echo "Warning: Could not remove $mod (it might be in use)."
        else
            echo "$mod removed successfully."
        fi
    done

    # Small pause to let kernel update module usage counts
    sleep 1
done

#lsmod | grep apple | grep " 0" | awk '{print $1}' | sudo xargs modprobe -r
sudo modprobe -r apple_bce hid_appletb_bl hid_appletb_kbd appletbdrm

