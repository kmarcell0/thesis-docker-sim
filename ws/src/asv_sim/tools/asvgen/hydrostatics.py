"""Hydrostatics of a catamaran with two cylindrical hulls.

This uses the same hull model as VRX's `Surface` plugin, which computes the
buoyancy in the simulation: each hull is a horizontal cylinder of radius r and
length L, and the submerged cross-section is a circular segment. The plugin
caps the submerged depth at r (half the cylinder), so a boat that needs more
than half-submerged hulls cannot float in VRX - the generator refuses it.

Quantities (boat frame, z up, z = 0 at the hull axes):

  draft d       depth of the hull bottom below the waterline, from
                rho * 2 * L * A_seg(d) = M                (Archimedes)
  A_seg(d)      = r^2/2 (theta - sin theta),  theta = 2 acos((r - d)/r)
  z_w           waterline height = -r + d
  z_B           centre of buoyancy = centroid of the segments
                = -4 r sin^3(theta/2) / (3 (theta - sin theta))
  b             waterline width of one hull = 2 sqrt(r^2 - (r - d)^2)
  I_T, I_L      second moments of the waterplane area about the roll and pitch
                axes:  I_T = 2 (L b^3/12 + L b s^2),  I_L = 2 b L^3/12
                (s = half the hull spacing; the hull spacing term dominates,
                which is why catamarans are so stable in roll)
  BM = I / V    metacentric radius, V = M / rho (displaced volume)
  GM = z_B + BM - z_G
                metacentric height: > 0 means a small heel creates a righting
                moment (stable). Small-angle (initial) stability only.
  heel  ~ atan(y_G / GM_T),  trim ~ atan(-(x_G - x_B) / GM_L), x_B = 0
  natural period T = 2 pi sqrt(I_G / (M g GM))   (no added mass, so it is a
                lower bound; real periods are longer)
"""
import math
from dataclasses import dataclass

RHO = 1000.0     # fresh water, kg/m^3
G = 9.81


def segment_area(r, d):
    """Area of the part of a circle (radius r) below a chord at height d above its bottom."""
    d = max(0.0, min(2 * r, d))
    theta = 2 * math.acos((r - d) / r)
    return r * r / 2 * (theta - math.sin(theta))


def segment_centroid_below_axis(r, d):
    """Distance from the circle centre down to the centroid of that segment (d <= r)."""
    theta = 2 * math.acos((r - d) / r)
    return 4 * r * math.sin(theta / 2) ** 3 / (3 * (theta - math.sin(theta)))


def waterline_width(r, d):
    return 2 * math.sqrt(max(0.0, r * r - (r - d) ** 2))


@dataclass
class Hydrostatics:
    draft: float
    z_water: float
    z_b: float
    volume: float
    bm_t: float
    bm_l: float
    gm_t: float
    gm_l: float
    heel_deg: float
    trim_deg: float
    reserve: float        # fraction of the max (half-submerged) buoyancy still unused
    roll_period: float
    pitch_period: float


class HydrostaticsError(Exception):
    pass


def solve(hull, mass, cog, inertia_tensor):
    """Hydrostatic equilibrium and initial stability of the catamaran."""
    r, length, s = hull['radius'], hull['length'], hull['spacing'] / 2
    a_need = mass / RHO / (2 * length)            # submerged area per hull
    a_max = math.pi * r * r / 2                   # VRX Surface plugin limit
    if a_need >= a_max:
        raise HydrostaticsError(
            f'boat too heavy ({mass:.1f} kg) for the hulls: they would have to be more than '
            'half submerged, which VRX\'s Surface plugin cannot model. Use bigger hulls or less mass.')
    lo, hi = 0.0, r                               # bisection: A_seg is monotonic in d
    for _ in range(60):
        mid = (lo + hi) / 2
        if segment_area(r, mid) < a_need:
            lo = mid
        else:
            hi = mid
    d = (lo + hi) / 2
    z_w = -r + d
    z_b = -segment_centroid_below_axis(r, d)
    b = waterline_width(r, d)
    volume = mass / RHO
    i_t = 2 * (length * b ** 3 / 12 + length * b * s * s)
    i_l = 2 * (b * length ** 3 / 12)
    bm_t, bm_l = i_t / volume, i_l / volume
    gm_t, gm_l = z_b + bm_t - cog[2], z_b + bm_l - cog[2]

    def period(i_g, gm):
        return 2 * math.pi * math.sqrt(i_g / (mass * G * gm)) if gm > 0 else float('inf')

    return Hydrostatics(
        draft=d, z_water=z_w, z_b=z_b, volume=volume, bm_t=bm_t, bm_l=bm_l, gm_t=gm_t, gm_l=gm_l,
        heel_deg=math.degrees(math.atan2(cog[1], gm_t)),
        trim_deg=math.degrees(math.atan2(-cog[0], gm_l)),
        reserve=1 - a_need / a_max,
        roll_period=period(inertia_tensor[0][0], gm_t),
        pitch_period=period(inertia_tensor[1][1], gm_l))
