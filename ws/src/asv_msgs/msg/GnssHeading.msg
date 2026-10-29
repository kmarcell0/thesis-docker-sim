# Dual-antenna GNSS heading of the bow (design.md section 4.3), as the UM982
# reports it. The antenna baseline mounting offset is assumed configured in the
# receiver, so azimuth is already the bow's direction.
# Relation to the yaw psi of base_link in world (ENU): azimuth = pi/2 - psi (wrapped).

std_msgs/Header header   # [-] stamp: GNSS epoch time; frame_id: "base_link"
float64 azimuth          # [rad] direction of the bow, clockwise from north, in [0, 2*pi)
float64 azimuth_std      # [rad] 1-sigma uncertainty the receiver reports
bool baseline_fixed      # [-] true if the receiver solved the antenna baseline (heading valid)
