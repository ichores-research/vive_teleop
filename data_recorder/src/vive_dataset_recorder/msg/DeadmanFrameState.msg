# One label per received camera frame. The header is copied from that frame.
std_msgs/Header header
builtin_interfaces/Time recorder_stamp
uint64 frame_sequence
bool operator_deadman
bool gate_stale
