#!/usr/bin/env python3

import json
import math
import os

import numpy as np
from PIL import Image, ImageFilter

SEED = 7
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(PKG, 'models')
WORLDS = os.path.join(PKG, 'worlds')

# ---------------------------------------------------------------- layout
TERRAIN_SIZE = 64.0          # m, square, centred on the world origin
HM_RES = 257                 # heightmap pixels per side (must be 2^n + 1)
TEX_RES = 1024               # ground texture pixels per side (covers whole terrain)

ROW_SPACING = 6.0            # m between tree rows (rows run along x)
TREE_SPACING = 5.0           # m between trees within a row
N_ROWS = 5
N_TREES = 8
ROW_Y = [(i - (N_ROWS - 1) / 2) * ROW_SPACING for i in range(N_ROWS)]        # -12 .. 12
TREE_X = [(j - (N_TREES - 1) / 2) * TREE_SPACING for j in range(N_TREES)]    # -17.5 .. 17.5
JITTER = 0.45                # m, planting position noise
ROW_WAVE = 0.40              # m, amplitude of the slow lateral bend of each row
MISSING_TREES = 0.10         # fraction of the planting positions left empty
YOUNG_TREES = 0.20           # fraction of the planted trees that are young and small
YOUNG_SCALE = 0.65           # size of a young tree relative to a mature one
N_ROCKS = 14
ROCK_RADII = {'field_rock_s': 0.15, 'field_rock_m': 0.22, 'field_rock_l': 0.30}  # m

UNDULATION = 0.22            # m, amplitude of the field-scale undulation
BUMPS = 0.03                 # m, amplitude of the short-wavelength roughness
N_HOLES = 30                 # shallow potholes over the terrain
HOLE_DEPTH = (0.05, 0.10)    # m
HOLE_SIGMA = (0.30, 0.50)    # m
BERM_HEIGHT = 0.08           # m, raised strip under each tree row
BERM_SIGMA = 1.0             # m

SPAWN = dict(x=-23.0, y=-3.0, yaw=0.0)   # headland, entrance of the path between rows 2 and 3
SPAWN_PAD_RADIUS = 2.5       # m, terrain flattened around the spawn point
RTK_BASE = dict(x=-29.5, y=-14.0)  # RTK base station on a known point, corner of the headland

rng = np.random.default_rng(SEED)
ROW_PHASE = rng.uniform(0, 2 * math.pi, N_ROWS)
ROW_WAVELENGTH = rng.uniform(25.0, 40.0, N_ROWS)


def row_centre(i, x):
    return ROW_Y[i] + ROW_WAVE * np.sin(2 * math.pi * x / ROW_WAVELENGTH[i] + ROW_PHASE[i])


# ---------------------------------------------------------------- terrain
def pixel_xy(n, size):
    
    c = np.linspace(-size / 2, size / 2, n)
    return c, c[::-1]


def make_heights():
    xs, ys = pixel_xy(HM_RES, TERRAIN_SIZE)
    X, Y = np.meshgrid(xs, ys)
    h = np.zeros_like(X)
    # gentle field undulation: a few long-wavelength waves with random direction/phase
    for _ in range(6):
        wl = rng.uniform(12.0, 30.0)
        ang = rng.uniform(0, math.pi)
        ph = rng.uniform(0, 2 * math.pi)
        h += np.sin(2 * math.pi * (X * math.cos(ang) + Y * math.sin(ang)) / wl + ph)
    h *= UNDULATION / 6 * 2.0
    # short-wavelength roughness
    for _ in range(8):
        wl = rng.uniform(2.5, 6.0)
        ang = rng.uniform(0, math.pi)
        ph = rng.uniform(0, 2 * math.pi)
        h += BUMPS / 4 * np.sin(2 * math.pi * (X * math.cos(ang) + Y * math.sin(ang)) / wl + ph)
    # potholes
    half = TERRAIN_SIZE / 2 - 2.0
    for _ in range(N_HOLES):
        hx, hy = rng.uniform(-half, half, 2)
        sigma = rng.uniform(*HOLE_SIGMA)
        h -= rng.uniform(*HOLE_DEPTH) * np.exp(-0.5 * ((X - hx) ** 2 + (Y - hy) ** 2) / sigma ** 2)
    # berms under the tree rows (only inside the planted area, faded at the ends)
    x_fade = np.clip((TREE_X[-1] + 3.0 - np.abs(X)) / 3.0, 0.0, 1.0)
    for i in range(N_ROWS):
        h += BERM_HEIGHT * np.exp(-0.5 * ((Y - row_centre(i, X)) / BERM_SIGMA) ** 2) * x_fade
    # flat spawn pad, blended smoothly into the surroundings
    d = np.hypot(X - SPAWN['x'], Y - SPAWN['y'])
    pad_h = bilinear(h, SPAWN['x'], SPAWN['y'])
    w = np.clip((d - SPAWN_PAD_RADIUS) / 2.0, 0.0, 1.0)
    w = w * w * (3 - 2 * w)
    h = pad_h * (1 - w) + h * w
    h -= h.min()
    return h


def bilinear(h, x, y):
    n = h.shape[0]
    step = TERRAIN_SIZE / (n - 1)
    fc = (x + TERRAIN_SIZE / 2) / step
    fr = (TERRAIN_SIZE / 2 - y) / step
    c0, r0 = int(math.floor(fc)), int(math.floor(fr))
    c0, r0 = min(max(c0, 0), n - 2), min(max(r0, 0), n - 2)
    tc, tr = fc - c0, fr - r0
    return float((h[r0, c0] * (1 - tc) + h[r0, c0 + 1] * tc) * (1 - tr)
                 + (h[r0 + 1, c0] * (1 - tc) + h[r0 + 1, c0 + 1] * tc) * tr)


def smooth_noise(res, blur):
    img = Image.fromarray((rng.random((res, res)) * 255).astype(np.uint8))
    return np.asarray(img.filter(ImageFilter.GaussianBlur(blur)), dtype=float) / 255.0


def make_texture():
    xs, ys = pixel_xy(TEX_RES, TERRAIN_SIZE)
    X, Y = np.meshgrid(xs, ys)
    # vegetation cover 0..1: weedy strips under rows, mostly grass outside the orchard
    veg = np.zeros_like(X)
    in_orchard = (np.abs(X) < TREE_X[-1] + 4.0) & (np.abs(Y) < ROW_Y[-1] + 3.0)
    for i in range(N_ROWS):
        veg = np.maximum(veg, np.exp(-0.5 * ((Y - row_centre(i, X)) / 0.9) ** 2))
    veg = np.where(in_orchard, veg, 0.75)
    veg = np.clip(veg + 0.35 * (smooth_noise(TEX_RES, 6) - 0.5) * 2, 0, 1)
    fine = smooth_noise(TEX_RES, 1.2)
    dirt = np.stack([0.55, 0.43, 0.31])[None, None, :] * (0.85 + 0.3 * fine[..., None])
    grass = np.stack([0.42, 0.45, 0.25])[None, None, :] * (0.8 + 0.4 * fine[..., None])
    rgb = dirt * (1 - veg[..., None]) + grass * veg[..., None]
    # faint wheel ruts along the centre of each path
    for k in range(N_ROWS - 1):
        py = (row_centre(k, X) + row_centre(k + 1, X)) / 2
        for off in (-0.29, 0.29):
            rut = np.exp(-0.5 * ((Y - py - off) / 0.06) ** 2) * in_orchard
            rgb *= (1 - 0.12 * rut)[..., None]
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


# ---------------------------------------------------------------- olive tree models
def tree_sdf(name, r, scale=1.0):
    trunk_h = r.uniform(0.9, 1.2) * scale
    trunk_r = r.uniform(0.12, 0.16) * scale
    lean = r.uniform(-0.08, 0.08)
    trunk_col = '0.33 0.27 0.20 1'
    leaf_cols = ['0.30 0.40 0.24 1', '0.36 0.44 0.30 1', '0.40 0.47 0.34 1']
    vis = []
    vis.append(f"""      <visual name="trunk">
        <pose>0 0 {trunk_h/2:.3f} {lean:.3f} 0 0</pose>
        <geometry><cylinder><radius>{trunk_r:.3f}</radius><length>{trunk_h:.3f}</length></cylinder></geometry>
        <material><ambient>{trunk_col}</ambient><diffuse>{trunk_col}</diffuse></material>
      </visual>""")
    # 3 scaffold branches
    for i in range(3):
        a = i * 2 * math.pi / 3 + r.uniform(-0.4, 0.4)
        tilt = r.uniform(0.45, 0.65)
        L = r.uniform(0.8, 1.0) * scale
        cx = math.cos(a) * math.sin(tilt) * L / 2
        cy = math.sin(a) * math.sin(tilt) * L / 2
        cz = trunk_h + math.cos(tilt) * L / 2
        vis.append(f"""      <visual name="branch_{i}">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} {-math.sin(a)*tilt:.3f} {math.cos(a)*tilt:.3f} 0</pose>
        <geometry><cylinder><radius>{trunk_r*0.55:.3f}</radius><length>{L:.3f}</length></cylinder></geometry>
        <material><ambient>{trunk_col}</ambient><diffuse>{trunk_col}</diffuse></material>
      </visual>""")
    # canopy: a central mass plus lobes; overall ~3.2-3.6 m wide, ~3.5 m tall
    lobes = [(0.0, 0.0, trunk_h + 1.45 * scale, 1.15 * scale, 1.15 * scale, 0.85 * scale)]
    for i in range(5):
        a = i * 2 * math.pi / 5 + r.uniform(-0.3, 0.3)
        d = r.uniform(0.6, 0.8) * scale
        lobes.append((math.cos(a) * d, math.sin(a) * d, trunk_h + r.uniform(1.1, 1.6) * scale,
                      r.uniform(0.75, 0.95) * scale, r.uniform(0.75, 0.95) * scale,
                      r.uniform(0.6, 0.75) * scale))
    for i, (x, y, z, rx, ry, rz) in enumerate(lobes):
        col = leaf_cols[i % 3]
        vis.append(f"""      <visual name="canopy_{i}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><ellipsoid><radii>{rx:.3f} {ry:.3f} {rz:.3f}</radii></ellipsoid></geometry>
        <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material>
      </visual>""")
    vis_xml = '\n'.join(vis)
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_orchard.py. Olive tree, primitives only.
     Collision: trunk only (canopy is far above the robot). -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="trunk_collision">
        <pose>0 0 {trunk_h/2 + 0.1:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{trunk_r:.3f}</radius><length>{trunk_h + 0.2:.3f}</length></cylinder></geometry>
      </collision>
{vis_xml}
    </link>
  </model>
</sdf>
""", trunk_r


def rock_sdf(name, radius):
    col = '0.45 0.43 0.40 1'
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_orchard.py. Half-buried field rock. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><ellipsoid><radii>{radius * 1.1:.3f} {radius:.3f} {radius * 0.9:.3f}</radii></ellipsoid></geometry>
        <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def model_config(name, desc):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
"""


def write(path, text, mode='w'):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, mode) as f:
        f.write(text)


# ---------------------------------------------------------------- main
def main():
    # terrain
    h = make_heights()
    z_range = float(h.max())
    tdir = os.path.join(MODELS, 'orchard_terrain')
    texdir = os.path.join(tdir, 'materials', 'textures')
    os.makedirs(texdir, exist_ok=True)
    Image.fromarray(np.round(h / z_range * 255).astype(np.uint8), 'L').save(
        os.path.join(texdir, 'heightmap.png'))
    Image.fromarray(make_texture(), 'RGB').save(os.path.join(texdir, 'ground_diffuse.png'))
    Image.new('RGB', (64, 64), (128, 128, 255)).save(os.path.join(texdir, 'ground_normal.png'))
    # quantised heights (exactly what Gazebo will load)
    hq = np.round(h / z_range * 255) / 255 * z_range

    hm = f"""<uri>model://orchard_terrain/materials/textures/heightmap.png</uri>
            <size>{TERRAIN_SIZE} {TERRAIN_SIZE} {z_range:.4f}</size>
            <pos>0 0 0</pos>"""
    write(os.path.join(tdir, 'model.sdf'), f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_orchard.py. Heightmap terrain {TERRAIN_SIZE:.0f} x {TERRAIN_SIZE:.0f} m,
     height range {z_range:.3f} m. -->
<sdf version="1.9">
  <model name="orchard_terrain">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <heightmap>
            {hm}
          </heightmap>
        </geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <heightmap>
            <use_terrain_paging>false</use_terrain_paging>
            <texture>
              <diffuse>model://orchard_terrain/materials/textures/ground_diffuse.png</diffuse>
              <normal>model://orchard_terrain/materials/textures/ground_normal.png</normal>
              <size>{TERRAIN_SIZE}</size>
            </texture>
            {hm}
          </heightmap>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
""")
    write(os.path.join(tdir, 'model.config'),
          model_config('orchard_terrain', 'Rough orchard ground with potholes and dirt paths.'))

    # tree models
    tree_r = {}
    for variant, scale, desc in (('a', 1.0, 'Mature'), ('b', 1.0, 'Mature'),
                                 ('young', YOUNG_SCALE, 'Young')):
        name = f'olive_tree_{variant}'
        sdf, tree_r[name] = tree_sdf(name, np.random.default_rng(SEED + len(variant) + ord(variant[0])), scale)
        write(os.path.join(MODELS, name, 'model.sdf'), sdf)
        write(os.path.join(MODELS, name, 'model.config'),
              model_config(name, f'{desc} olive tree (primitive geometry).'))
    for name, radius in ROCK_RADII.items():
        write(os.path.join(MODELS, name, 'model.sdf'), rock_sdf(name, radius))
        write(os.path.join(MODELS, name, 'model.config'), model_config(name, 'Field rock.'))

    # trees in the world
    includes, trees = [], []
    for i in range(N_ROWS):
        for j, tx in enumerate(TREE_X):
            # the first trees of the two rows beside the spawn path mark its entrance
            entrance = j == 0 and ROW_Y[i] in (SPAWN['y'] - 3.0, SPAWN['y'] + 3.0)
            if rng.random() < MISSING_TREES and not entrance:
                continue
            x = tx + rng.uniform(-JITTER, JITTER)
            y = row_centre(i, x) + rng.uniform(-JITTER, JITTER)
            if rng.random() < YOUNG_TREES:
                model = 'olive_tree_young'
            else:
                model = 'olive_tree_a' if rng.random() < 0.5 else 'olive_tree_b'
            z = bilinear(hq, x, y) - 0.05
            yaw = rng.uniform(-math.pi, math.pi)
            includes.append(f"""    <include>
      <uri>model://{model}</uri>
      <name>olive_r{i+1}_t{j+1}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
    </include>""")
            trees.append(dict(name=f'olive_r{i+1}_t{j+1}', x=round(x, 3), y=round(y, 3),
                              trunk_radius=round(tree_r[model], 3)))

    # rocks beside the rows, never on the centre of a path
    rocks = []
    while len(rocks) < N_ROCKS:
        i = rng.integers(N_ROWS)
        x = rng.uniform(TREE_X[0] - 2.0, TREE_X[-1] + 2.0)
        y = float(row_centre(i, x)) + rng.uniform(-1.3, 1.3)
        if min(math.hypot(x - t['x'], y - t['y']) for t in trees) < 1.0:
            continue
        if any(math.hypot(x - q['x'], y - q['y']) < 2.0 for q in rocks):
            continue
        model = str(rng.choice(list(ROCK_RADII)))
        z = bilinear(hq, x, y)
        includes.append(f"""    <include>
      <uri>model://{model}</uri>
      <name>rock_{len(rocks) + 1}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {rng.uniform(-math.pi, math.pi):.3f}</pose>
    </include>""")
        rocks.append(dict(name=f'rock_{len(rocks) + 1}', x=round(x, 3), y=round(y, 3),
                          radius=ROCK_RADII[model]))

    spawn_z = bilinear(hq, SPAWN['x'], SPAWN['y'])
    base_z = bilinear(hq, RTK_BASE['x'], RTK_BASE['y'])
    tree_xml = '\n'.join(includes)
    write(os.path.join(WORLDS, 'olive_orchard.sdf'), f"""<?xml version="1.0"?>
<!-- Generated by scripts/generate_orchard.py (seed {SEED}). Do not edit by hand.
     Rough olive orchard: {N_ROWS} irregular rows of up to {N_TREES} trees, rows along x every {ROW_SPACING} m,
     trees every {TREE_SPACING} m, dirt paths between rows, rough heightmap terrain with potholes.
     Robot spawn (launch default): x={SPAWN['x']} y={SPAWN['y']} z={spawn_z + 0.05:.3f} yaw={SPAWN['yaw']} -->
<sdf version="1.9">
  <world name="olive_orchard">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <!-- sensors: rendering-based (lidar, camera), IMU and GPS -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <!-- GPS origin: olive-growing region near Sfax, Tunisia [A] -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>34.74</latitude_deg>
      <longitude_deg>10.76</longitude_deg>
      <elevation>20.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.65 0.78 0.92 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.88 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>

    <include>
      <uri>model://orchard_terrain</uri>
      <name>orchard_terrain</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- RTK GNSS base station on a known point (corrections for the robot GPS) -->
    <include>
      <uri>model://rtk_base_station</uri>
      <name>rtk_base_station</name>
      <pose>{RTK_BASE['x']} {RTK_BASE['y']} {base_z:.3f} 0 0 0</pose>
    </include>

{tree_xml}
  </world>
</sdf>
""")

    meta = dict(
        seed=SEED, terrain_size=TERRAIN_SIZE, heightmap_resolution=HM_RES, height_range=round(z_range, 4),
        heightmap='models/orchard_terrain/materials/textures/heightmap.png',
        row_y=ROW_Y, path_y=[(ROW_Y[k] + ROW_Y[k + 1]) / 2 for k in range(N_ROWS - 1)],
        tree_x=TREE_X, row_spacing=ROW_SPACING, tree_spacing=TREE_SPACING,
        spawn=dict(x=SPAWN['x'], y=SPAWN['y'], z=round(spawn_z + 0.05, 3), yaw=SPAWN['yaw']),
        rtk_base=dict(x=RTK_BASE['x'], y=RTK_BASE['y'], z=round(base_z, 3)),
        trees=trees, rocks=rocks)
    write(os.path.join(WORLDS, 'olive_orchard.json'), json.dumps(meta, indent=1) + '\n')

    slope = np.degrees(np.arctan(np.max(np.hypot(*np.gradient(hq, TERRAIN_SIZE / (HM_RES - 1))))))
    print(f'height range {z_range:.3f} m, max slope {slope:.1f} deg, '
          f'spawn z {spawn_z + 0.05:.3f} m, {len(trees)} trees, {len(rocks)} rocks')


if __name__ == '__main__':
    main()
