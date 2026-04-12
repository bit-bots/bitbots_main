# Message to provide information concerning the status of the SVO playback

# SVO File name
string file_name

# SVO playback status
uint8 status

# SVO playback status constants
uint8 STATUS_PLAYING=0
uint8 STATUS_PAUSED=1
uint8 STATUS_END=2

# SVO frame timestamp
uint64 frame_ts

# SVO playback position
uint32 frame_id

# SVO total number of frames
uint32 total_frames

# SVO loop status
bool loop_active

# SVO loop count
uint32 loop_count

# SVO real-time mode status
bool real_time_mode
