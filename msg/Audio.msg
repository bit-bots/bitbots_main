# This message is used to activate the audio output of the robot
# This can be used for debug proposed but also for natural language team communication

# The text will only be outputted if "filename" is empty
string text

# The higher the value, the higher the priority
uint8 priority

# If a file should be read, the path has to be specified here. Otherwise this string should be null
string filename
