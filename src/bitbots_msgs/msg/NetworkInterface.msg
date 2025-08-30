string name
string mac_address
string[] ip_addresses

# global statistics
int64 received_bytes
int64 received_packets
int64 received_packets_errors
int64 received_packets_dropped
int64 sent_bytes
int64 sent_packets
int64 sent_packets_errors
int64 sent_packets_dropped
int64 sent_packets_collisions

# transmission statistics
# these rates are aggregations from the last 2 seconds and measured as per second
int64 rate_received_bytes
int64 rate_received_packets
int64 rate_received_packets_errors
int64 rate_received_packets_dropped
int64 rate_sent_bytes
int64 rate_sent_packets
int64 rate_sent_packets_errors
int64 rate_sent_packets_dropped
int64 rate_sent_packets_collisions
