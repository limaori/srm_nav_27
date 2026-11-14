#!/usr/bin/env python3
"""Generate the RMUC 2025 occupancy PGM from the bundled Gazebo collision mesh.

Requires NumPy, Pillow and SciPy. This map uses Gazebo world coordinates.
"""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_closing


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/srm27_gazebo_simulator/resource/models/rmuc_2025'
DEFAULT_MAP = ROOT / 'src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025.pgm'
ORIGIN = (-0.10, -0.10)
RESOLUTION = 0.05
WIDTH, HEIGHT = 586, 324
SUBPIXELS = 4


def clip_at_height(vertices, height, direction):
    clipped = []
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        start_inside = direction * (start[2] - height) >= 0
        end_inside = direction * (end[2] - height) >= 0
        if start_inside:
            clipped.append(start)
        if start_inside != end_inside:
            fraction = (height - start[2]) / (end[2] - start[2])
            clipped.append(start + fraction * (end - start))
    return clipped


def project_height_band(triangles, low, high):
    mask = Image.new('L', (WIDTH * SUBPIXELS, HEIGHT * SUBPIXELS))
    draw = ImageDraw.Draw(mask)
    z = triangles[:, :, 2]
    for triangle in triangles[(z.min(axis=1) <= high) & (z.max(axis=1) >= low)]:
        polygon = clip_at_height(clip_at_height(list(triangle), low, 1), high, -1)
        if len(polygon) < 2:
            continue
        xy = np.asarray(polygon)[:, :2]
        pixel = np.floor((xy - ORIGIN) * (SUBPIXELS / RESOLUTION)).astype(int)
        points = [(x, HEIGHT * SUBPIXELS - 1 - y) for x, y in pixel]
        if len(points) >= 3:
            draw.polygon(points, fill=255)
        else:
            draw.line(points, fill=255)
    return np.asarray(mask).reshape(HEIGHT, SUBPIXELS, WIDTH, SUBPIXELS).any(axis=(1, 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_MAP)
    args = parser.parse_args()

    link = ET.parse(MODEL / 'model.sdf').find('model/link')
    pose = np.fromstring(link.findtext('pose'), sep=' ')
    if np.any(pose[3:]):
        raise ValueError('The RMUC 2025 collision mesh has an unexpected rotation')
    mesh = MODEL / 'meshes/rmuc_2025.stl'
    stl = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.fromfile(mesh, dtype=stl, offset=84)['vertices'].astype(float) + pose[:3]

    # Flat terrain is at world z=0.20 m. Close small gaps between mesh facets.
    floor = binary_closing(project_height_band(triangles, 0.188, 0.24), iterations=2)
    # Above the floor, clip faces to the robot's vertical clearance band. This
    # retains wall faces and low obstacles without painting the entire ground.
    obstacles = project_height_band(triangles, 0.25, 0.56)
    pixels = np.where(floor & ~obstacles, 255, 0).astype(np.uint8)
    header = f'P5\n# RMUC 2025 Gazebo collision map\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii')
    args.output.write_bytes(header + pixels.tobytes())


if __name__ == '__main__':
    main()
