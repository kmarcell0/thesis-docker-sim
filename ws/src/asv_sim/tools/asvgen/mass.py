"""Mass properties of the boat.

Every physical part is a MassItem: a mass m_i at a position r_i (its own
centre of gravity) with its own inertia tensor I_i about that point, aligned
with the boat axes. The parts are combined into one rigid body:

    total mass        M   = sum m_i
    centre of gravity c   = sum(m_i r_i) / M
    inertia about c   I   = sum( I_i + m_i (|d_i|^2 E - d_i d_i^T) ),  d_i = r_i - c

The second term is the parallel axis (Huygens-Steiner) theorem in tensor
form; it also produces the products of inertia (off-diagonal terms) when parts
sit off-centre. E is the 3x3 identity.
"""
from dataclasses import dataclass, field


@dataclass
class MassItem:
    name: str
    mass: float                    # kg
    xyz: list                      # m, boat frame, the part's own centre of gravity
    inertia: tuple = (0.0, 0.0, 0.0)   # Ixx, Iyy, Izz about xyz (kg m^2); 0 = point mass
    group: str = 'other'           # for the mass budget table


@dataclass
class MassProperties:
    mass: float
    cog: list
    tensor: list = field(default_factory=list)   # 3x3, about cog

    @property
    def sdf_inertia(self):
        """(ixx, iyy, izz, ixy, ixz, iyz) as SDF expects them."""
        t = self.tensor
        return (t[0][0], t[1][1], t[2][2], t[0][1], t[0][2], t[1][2])


# --- inertia of uniform solid primitives about their own centre --------------
def box(m, sx, sy, sz):
    return (m * (sy * sy + sz * sz) / 12, m * (sx * sx + sz * sz) / 12, m * (sx * sx + sy * sy) / 12)


def cylinder_x(m, r, length):
    """Solid cylinder whose axis is the x axis (e.g. a hull)."""
    side = m * (3 * r * r + length * length) / 12
    return (m * r * r / 2, side, side)


def cylinder_z(m, r, length):
    """Solid cylinder whose axis is the z axis (e.g. a mast)."""
    side = m * (3 * r * r + length * length) / 12
    return (side, side, m * r * r / 2)


def combine(items):
    """Combine MassItems into one rigid body (see module docstring)."""
    m = sum(i.mass for i in items)
    if m <= 0:
        raise ValueError('total mass must be positive')
    c = [sum(i.mass * i.xyz[k] for i in items) / m for k in range(3)]
    t = [[0.0] * 3 for _ in range(3)]
    for i in items:
        d = [i.xyz[k] - c[k] for k in range(3)]
        d2 = sum(x * x for x in d)
        for a in range(3):
            t[a][a] += i.inertia[a]
            for b in range(3):
                t[a][b] += i.mass * ((d2 if a == b else 0.0) - d[a] * d[b])
    return MassProperties(m, c, t)
