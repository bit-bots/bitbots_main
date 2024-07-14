#!/usr/bin/env bash
set -eEuo pipefail

ROBOT_NAME="${ROBOT_NAME:-}"

# Mapping robot name to voice and speed
case "$ROBOT_NAME" in
  "jack"|"rory")
    voice="en_UK/apope_low"
    speed=1.0
    ;;
  "amy"|"donna"|"melody"|"rose")
    voice="en_US/vctk_low"
    speed=1.7
    ;;
  *)
    echo "Unknown robot: '$ROBOT_NAME', using default female voice"
    voice="en_US/vctk_low"
    speed=1.7
    ;;
esac

text="$1"
if [ -z "$text" ]; then
  echo "No text provided!"
  exit 1
fi

# Generate the speech with mimic and play it with alsa
mimic3 --remote --voice "$voice" --length-scale "$speed" "$text" | aplay -q -
