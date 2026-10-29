# One buoy on a map: the true map (/asv/ground_truth/buoys, M2.7) or the
# filter's estimate (/asv/slam/map). Planar: the filter ignores height.
# Position in the frame of the LandmarkArray header ("map", the same as world:
# ENU, x east, y north).

uint32 id               # [-] true map: index in buoys_gt.yaml; estimate: the filter's landmark number
string name             # [-] true map: model instance name (e.g. a Task 1 gate buoy); empty for estimates
BuoyColour colour       # [-] UNKNOWN only for an estimate that was never seen in colour
string model            # [-] buoy class from the course generator (A-0, A-2, Sur-Mark); empty for estimates
float64 radius          # [m] buoy radius at the waterline; 0 if not known (estimates)
float64 x               # [m] east, in map
float64 y               # [m] north, in map
float64[4] covariance   # [m^2] 2x2 position covariance of (x, y), row-major: xx, xy, yx, yy; all zero for the true map
uint32 sightings        # [-] number of detections associated with it; 0 for the true map
bool confirmed          # [-] false while tentative (seen fewer than n_conf times); true for the true map
