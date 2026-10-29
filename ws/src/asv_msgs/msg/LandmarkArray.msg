# A whole buoy map: the true map (latched, once) or the filter's estimate.

std_msgs/Header header   # [-] stamp: time of the map (estimate) or of publishing (true map); frame_id: "map"
Landmark[] landmarks     # [-] all buoys on the map
