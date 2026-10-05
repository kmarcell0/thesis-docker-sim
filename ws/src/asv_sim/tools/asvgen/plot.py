"""Layout figure: top view and side view of the boat with what each sensor sees.

Needs matplotlib (inside the container: sudo apt install python3-matplotlib).
Everything is drawn from the generated Boat, so the figure always matches the
model.

  * Cameras: the exact pinhole footprint on the water. Rays through the image
    border are intersected with the water plane; rays that reach the horizon
    (or the water beyond VIEW_RANGE) are cut at VIEW_RANGE. The side view shows
    the vertical section through the optical axis, computed the same way.
    Occlusion by the boat is not considered for cameras.
  * Lidar: the line-of-sight water visibility map from analysis.water_visibility
    (light = visible water, grey = blind water next to the boat).
"""
import math

from . import geometry as geo
from .analysis import water_visibility, boat_outline

VIEW_RANGE = 4.0       # m, how far camera footprints are drawn
PALETTE = ['#d62728', '#1f77b4', '#2ca02c', '#9467bd', '#b8860b', '#8c564b', '#e377c2', '#17becf']
BORDER_SAMPLES = 24    # rays per image edge


def _camera_rays(spec, R, border=True):
    """Unit directions (boat frame) of rays through the image border (or,
    with border=False, through the central image column, top to bottom)."""
    w, h = spec['width'], spec['height']
    f = (w / 2) / math.tan(spec['hfov'] / 2)
    n = BORDER_SAMPLES
    if border:
        pix = ([(w * i / n, 0) for i in range(n)] + [(w, h * i / n) for i in range(n)]
               + [(w - w * i / n, h) for i in range(n)] + [(0, h - h * i / n) for i in range(n)])
    else:
        pix = [(w / 2, h * i / (2 * n)) for i in range(2 * n + 1)]
    rays = []
    for u, v in pix:
        d = geo.mat_vec(R, [f, -(u - w / 2), -(v - h / 2)])   # camera body: x fwd, y left, z up
        nrm = geo.norm(d)
        rays.append([c / nrm for c in d])
    return rays


def _ray_end(o, d, z_water, max_horizontal):
    """Where a ray meets the water, or the point at max_horizontal distance."""
    horiz = math.hypot(d[0], d[1])
    if d[2] < -1e-9:
        t = (z_water - o[2]) / d[2]
        if t * horiz <= max_horizontal:
            return [o[i] + t * d[i] for i in range(3)]
    t = max_horizontal / horiz if horiz > 1e-9 else max_horizontal
    return [o[i] + t * d[i] for i in range(3)]


def plot_layout(boat, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch, Polygon, Rectangle
    except ImportError:
        print('  (no matplotlib: skipping the layout figure - sudo apt install python3-matplotlib)')
        return None

    cfg = boat.cfg
    h, d, t = cfg['hull'], cfg['deck'], cfg['thrusters']
    r, hl, hy = h['radius'], h['length'], h['spacing'] / 2
    zw = boat.hydro.z_water
    ox, oy = boat_outline(boat)
    fig, (top, side) = plt.subplots(2, 1, figsize=(13, 13.5), gridspec_kw={'height_ratios': [3.2, 1.25]})
    legend = []
    colors = {part.name: PALETTE[i % len(PALETTE)] for i, part in enumerate(boat.parts)}

    # ------------------------------------------------- lidar water visibility
    for part in boat.parts:
        if part.type != 'lidar':
            continue
        vis = water_visibility(part, boat)
        data = [[float('nan') if v is None else (1.0 if v else 0.0) for v in row] for row in vis['grid']]
        step = vis['xs'][1] - vis['xs'][0]
        top.imshow(data, origin='lower', cmap=ListedColormap(['#9e9e9e', '#d6eaf8']), vmin=0, vmax=1,
                   extent=[vis['xs'][0] - step / 2, vis['xs'][-1] + step / 2,
                           vis['ys'][0] - step / 2, vis['ys'][-1] + step / 2], interpolation='nearest', zorder=0)
        legend += [Patch(fc='#d6eaf8', ec='#aaaaaa', label=f'water {part.name} can see'),
                   Patch(fc='#9e9e9e', label=f'water {part.name} cannot see (blind)')]

    # ------------------------------------------------------------- boat, top
    for y in (hy, -hy):
        top.add_patch(Rectangle((-hl / 2, y - r), hl, 2 * r, fc='#f4f4f4', ec='#555555', zorder=2))
    top.add_patch(Rectangle((-d['length'] / 2, -d['width'] / 2), d['length'], d['width'],
                            fc='#aec6e8', ec='#1f4e79', alpha=0.85, zorder=3))
    bat = dict(cfg['battery'], name='battery')
    for f, fc in [(f, '#555555') for f in cfg['fixed_masses']] + [(bat, '#f39c12')]:
        top.add_patch(Rectangle((f['xyz'][0] - f['size'][0] / 2, f['xyz'][1] - f['size'][1] / 2),
                                f['size'][0], f['size'][1], fc=fc, zorder=4))
        top.annotate(f['name'], f['xyz'][:2], ha='center', va='center', fontsize=7, zorder=9,
                     color='white' if fc == '#555555' else 'black')
    for y in (hy, -hy):
        top.plot(t['x'], y, 's', color='#222222', ms=6, zorder=5)
    for mnt in boat.mounts:
        bx, by, bz = mnt['base']
        top.plot(bx, by, 'o', mfc='none', mec='#444444', ms=5, zorder=6)
        side.plot([bx, bx], [bz, mnt['top_z']], color='#666666', lw=2, zorder=3)
        if mnt['end']:
            top.plot([bx, mnt['end'][0]], [by, mnt['end'][1]], color='#444444', lw=2, zorder=6)
            side.plot([bx, mnt['end'][0]], [mnt['top_z']] * 2, color='#666666', lw=2, zorder=3)
    top.plot(*boat.total.cog[:2], 'k+', ms=14, mew=2, zorder=8)
    legend.append(Line2D([], [], color='k', marker='+', ls='', ms=10, mew=2, label='centre of gravity'))

    # ------------------------------------------------------------ boat, side
    side.axhspan(-1, zw, color='#d6eaf8', zorder=0)
    side.axhline(zw, color='#2e86c1', lw=1.2, zorder=1)
    side.add_patch(Rectangle((-hl / 2, -r), hl, 2 * r, fc='#f4f4f4', ec='#555555', zorder=2))
    side.add_patch(Rectangle((-d['length'] / 2, d['height'] - d['thickness'] / 2), d['length'], d['thickness'],
                             fc='#aec6e8', ec='#1f4e79', zorder=2))
    for f, fc in [(f, '#555555') for f in cfg['fixed_masses']] + [(bat, '#f39c12')]:
        side.add_patch(Rectangle((f['xyz'][0] - f['size'][0] / 2, f['xyz'][2] - f['size'][2] / 2),
                                 f['size'][0], f['size'][2], fc=fc, zorder=3))
    side.plot([t['x'], t['x'] - 0.06], [-r, boat.z_prop], color='#222222', lw=3, zorder=3)
    side.plot(boat.total.cog[0], boat.total.cog[2], 'k+', ms=14, mew=2, zorder=8)

    # --------------------------------------------------------------- sensors
    for part in boat.parts:
        s, spec, c = part.cfg, part.spec, colors[part.name]
        R = geo.rot(s['rpy'])
        if part.type == 'gnss_dual':
            p1, p2 = s['xyz'], s['xyz_secondary']
            top.plot([p1[0], p2[0]], [p1[1], p2[1]], '--', color=c, lw=1.5, zorder=7)
            side.plot([p1[0], p2[0]], [p1[2], p2[2]], '--', color=c, lw=1.5, zorder=7)
            for p in (p1, p2):
                top.plot(p[0], p[1], 'o', color=c, ms=8, mec='k', mew=0.5, zorder=8)
                side.plot(p[0], p[2], 'o', color=c, ms=8, mec='k', mew=0.5, zorder=8)
            legend.append(Line2D([], [], color=c, marker='o', ls='--', label=f'{part.name} antennas + baseline'))
            continue
        x, y, z = s['xyz']
        top.plot(x, y, 'o', color=c, ms=8, mec='k', mew=0.5, zorder=8)
        side.plot(x, z, 'o', color=c, ms=8, mec='k', mew=0.5, zorder=8)
        if part.type in ('camera', 'stereo_camera'):
            foot = [_ray_end(s['xyz'], ray, zw, VIEW_RANGE) for ray in _camera_rays(spec, R)]
            top.add_patch(Polygon([(p[0], p[1]) for p in foot], closed=True, fc=c, alpha=0.18, ec=c, lw=1.5,
                                  zorder=1))
            section = [_ray_end(s['xyz'], ray, zw, VIEW_RANGE) for ray in _camera_rays(spec, R, border=False)]
            side.add_patch(Polygon([(x, z)] + [(p[0], p[2]) for p in section], closed=True, fc=c, alpha=0.18,
                                   ec=c, lw=1.2, zorder=1))
            legend.append(Patch(fc=c, alpha=0.4, ec=c, label=f'{part.name}: water / view in the image '
                                                              f'(cut at {VIEW_RANGE:.0f} m)'))
        elif part.type == 'lidar':
            lo, hi = spec['vfov']
            for az in (0.0, math.pi):                 # outermost beams, forward and backward
                for b in (lo, hi):
                    e = _ray_end(s['xyz'], geo.mat_vec(R, geo.ray(az, b)), zw, VIEW_RANGE)
                    side.plot([x, e[0]], [z, e[2]], color=c, lw=1, zorder=4)
            legend.append(Line2D([], [], color=c, marker='o', label=f'{part.name} (outermost beams in side view)'))
        else:
            legend.append(Line2D([], [], color=c, marker='o', ls='', label=part.name))

    top.set_xlim(-ox - 3.1, ox + 3.1)
    top.set_ylim(-oy - 3.1, oy + 3.1)
    side.set_xlim(-ox - 3.1, ox + 3.1)
    side.set_ylim(-0.45, 1.2)
    for ax, title, ylabel in ((top, 'Top view', 'y (left) [m]'), (side, 'Side view (centre line)', 'z (up) [m]')):
        ax.set_aspect('equal')
        ax.grid(alpha=0.25)
        ax.set_xlabel('x (forward) [m]')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
    top.legend(handles=legend, loc='upper left', fontsize=8, framealpha=0.9)
    fig.suptitle(f'{cfg["name"]} sensor layout (fingerprint {boat.fingerprint})')
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path
