#!/bin/bash

make clean
make all
sudo make install
#6.6.37-2-t2-jammy instead of uname or any other image you want to amend.
sudo vboxconfig
sudo depmod $(uname -r) 
sudo update-initramfs -u -k $(uname -r)

