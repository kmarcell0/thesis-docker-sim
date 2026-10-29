# Colour of a buoy, as seen by a camera or taken from the true buoy map.
# UNKNOWN: no camera saw the buoy (outside every field of view or too far);
# data association then uses the Mahalanobis gate only (design.md section 6).
# The four colours are the ones in the course generator.

uint8 UNKNOWN=0
uint8 RED=1
uint8 GREEN=2
uint8 YELLOW=3
uint8 BLACK=4

uint8 value  # [-] one of the constants above
