# A list of colors which describes a color lookup table.

std_msgs/Header header

int32[] blue
int32[] green
int32[] red

# The first implementation used unit8 to encode the colors. Counterintuitively this was much slower
# (like 400ms for encoding a single message), because python does some weird internal processing with unit8 message arrays.
# Finally we decided to accept the size overhead of an 32 bit integer to get a performance boost.