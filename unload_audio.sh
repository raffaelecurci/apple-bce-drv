#!/bin/bash

# Target base module to unload last
BASE_MODULE="apple_bce"

# Filter modules related to snd or apple_bce
MODULE_FILTER='snd|apple_bce'

while true; do
  # Get modules matching filter with usage count 0 (excluding BASE_MODULE unless last)
  zero_usage_modules=($(lsmod | grep -E "$MODULE_FILTER" | awk -v base="$BASE_MODULE" '$3==0 && $1!=base {print $1}'))

  if [ ${#zero_usage_modules[@]} -eq 0 ]; then
    # No zero usage modules found other than possibly base
    break
  fi

  # Unload zero usage modules one by one
  for mod in "${zero_usage_modules[@]}"; do
    echo "Unloading $mod"
    sudo modprobe -r "$mod"
    if [ $? -ne 0 ]; then
      echo "Failed to unload $mod, stopping."
      exit 1
    fi
  done
done

# Finally try to unload the base module if usage is zero
base_usage=$(lsmod | grep "^$BASE_MODULE " | awk '{print $3}')
if [ "$base_usage" == "0" ]; then
  echo "Unloading base module $BASE_MODULE"
  sudo modprobe -r "$BASE_MODULE"
else
  echo "Base module $BASE_MODULE is still in use (usage=$base_usage). Cannot unload."
fi
