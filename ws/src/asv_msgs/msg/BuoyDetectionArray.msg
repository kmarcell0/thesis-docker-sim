# All objects seen in one scan. Published after every scan, also when the list
# is empty: "nothing seen" is information too (e.g. for a missed detection).

std_msgs/Header header       # [-] stamp: measurement time of the scan (simulation time); frame_id: "base_link"
BuoyDetection[] detections   # [-] may be empty
