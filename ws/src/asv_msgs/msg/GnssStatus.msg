# True state of the simulated GNSS receiver (design.md section 4.2), published by
# the corruptor on /asv/ground_truth/gnss_status for evaluation ONLY. The filter
# never reads it: it learns the fix quality only from the NavSatFix covariance
# (assumption A4).

uint8 MODE_NO_FIX=0        # no position (sky-view outage)
uint8 MODE_STANDALONE=1    # no corrections, metre level
uint8 MODE_RTK_FLOAT=2     # corrections used, ambiguities not fixed, decimetre level
uint8 MODE_RTK_FIXED=3     # corrections used, ambiguities fixed, centimetre level

std_msgs/Header header     # [-] stamp: GNSS epoch time; frame_id: empty (not a spatial quantity)
uint8 position_mode        # [-] one of the MODE_ constants
bool sky_view_outage       # [-] true: antennas see no satellites; no fix, velocity or heading is published
bool corrections_lost      # [-] true: RTK corrections from the shore base are missing (position falls back; heading stays up)
bool heading_available     # [-] true: a GnssHeading is published this epoch
