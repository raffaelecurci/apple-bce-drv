#!/bin/bash
sudo modprobe -r apple_bce && clear && make clean && \
#current_version=$(grep -oP '(?<=#define VERSION_PATCH ")[0-9]+' apple_bce.h) && \
#new_version=$((current_version + 1)) && \
#sed -i "s/#define VERSION_PATCH \"$current_version\"/#define VERSION_PATCH \"$new_version\"/" apple_bce.h && \
make && sudo modinfo -F version ./apple-bce.ko && sudo insmod ./apple-bce.ko
