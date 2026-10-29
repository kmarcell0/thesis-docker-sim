# One object seen in one scan (design.md section 5).
# Measured from the base_link origin in the level frame: base_link rotated back
# by roll and pitch, so x points forward and is horizontal, z points up.
# Carries no noise sigma (decision M2.4 A a): the filter takes the measurement
# noise covariance Q from its own config. Carries no true buoy ID either:
# slam_replay logs that separately for evaluation.

uint8 CLASS_BUOY=0   # a buoy: may become a landmark
uint8 CLASS_OTHER=1  # another vessel: the filter must not map it

float64 range        # [m] horizontal distance from the base_link origin to the object centre, >= 0
float64 bearing      # [rad] angle in the level frame from base_link x (forward), positive to the left (counter-clockwise seen from above), in (-pi, pi]
BuoyColour colour    # [-] UNKNOWN if no camera saw it
uint8 object_class   # [-] CLASS_BUOY or CLASS_OTHER
