#!/usr/bin/env python3
"""Generate a deterministic 80 m x 80 m mountain tea-garden Gazebo world.

The generated scene targets Gazebo Classic / SDF 1.6.  All important objects
have collision geometry so the world can be used by lidar, depth-camera and
ground-robot path-planning experiments.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw


WORLD_SIZE = 80.0
TERRAIN_BASE_Z = 0.5
HEIGHT_SCALE = 17.0
HEIGHTMAP_SAMPLES = 257
DEFAULT_SEED = 20260730

PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORLD_PATH = Path(__file__).with_name("mountain_tea_garden_80m.world")
TERRAIN_DIR = PACKAGE_DIR / "models" / "mountain_tea_garden_terrain"
TEXTURE_DIR = TERRAIN_DIR / "materials" / "textures"
HEIGHTMAP_PATH = TEXTURE_DIR / "tea_garden_heightmap.png"
DIFFUSE_PATH = TEXTURE_DIR / "tea_soil_diffuse.png"
NORMAL_PATH = TEXTURE_DIR / "flat_normal.png"


def child(parent: ET.Element, tag: str, text: str | None = None, **attrs: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = text
    return node


def values(*items: float) -> str:
    return " ".join(f"{item:.6g}" for item in items)


def pose(parent: ET.Element, xyzrpy: tuple[float, float, float, float, float, float]) -> None:
    child(parent, "pose", values(*xyzrpy))


def indent_xml(node: ET.Element, level: int = 0) -> None:
    whitespace = "\n" + level * "  "
    child_whitespace = "\n" + (level + 1) * "  "
    if len(node):
        if not node.text or not node.text.strip():
            node.text = child_whitespace
        for item in node:
            indent_xml(item, level + 1)
        if not item.tail or not item.tail.strip():
            item.tail = whitespace
    if level and (not node.tail or not node.tail.strip()):
        node.tail = whitespace


def raw_terrain_height(x: float, y: float) -> float:
    """Unnormalised mountain profile used to construct the heightmap."""
    ny = y / 40.0
    uphill = 1.25 + 5.7 * ((ny + 1.0) / 2.0)
    broad_hill = 2.15 * math.exp(-(((x - 14.0) / 21.0) ** 2 + ((y - 7.0) / 18.0) ** 2))
    west_ridge = 1.15 * math.exp(-(((x + 25.0) / 13.0) ** 2 + ((y - 17.0) / 25.0) ** 2))
    hollow = -1.10 * math.exp(-(((x + 11.0) / 12.0) ** 2 + ((y + 19.0) / 10.0) ** 2))
    undulation = 0.36 * math.sin(0.105 * x + 0.035 * y) + 0.22 * math.sin(0.16 * y - 0.045 * x)
    return uphill + broad_hill + west_ridge + hollow + undulation


def raw_height_bounds() -> tuple[float, float]:
    """Get the exact extrema of the grid written to the PNG."""
    minimum = math.inf
    maximum = -math.inf
    for row in range(HEIGHTMAP_SAMPLES):
        y = 40.0 - row * WORLD_SIZE / (HEIGHTMAP_SAMPLES - 1)
        for col in range(HEIGHTMAP_SAMPLES):
            x = -40.0 + col * WORLD_SIZE / (HEIGHTMAP_SAMPLES - 1)
            height = raw_terrain_height(x, y)
            minimum = min(minimum, height)
            maximum = max(maximum, height)
    return minimum, maximum


RAW_HEIGHT_MIN, RAW_HEIGHT_MAX = raw_height_bounds()


def terrain_height(x: float, y: float) -> float:
    """Gazebo world Z in metres, including Gazebo's 0..255 normalisation."""
    normalised = (raw_terrain_height(x, y) - RAW_HEIGHT_MIN) / (RAW_HEIGHT_MAX - RAW_HEIGHT_MIN)
    return TERRAIN_BASE_Z + HEIGHT_SCALE * max(0.0, min(1.0, normalised))


def terrain_rpy(yaw: float, x: float, y: float) -> tuple[float, float, float]:
    """Return a local tangent-plane orientation for an object at x/y."""
    epsilon = 0.12
    dz_dx = (terrain_height(x + epsilon, y) - terrain_height(x - epsilon, y)) / (2.0 * epsilon)
    dz_dy = (terrain_height(x, y + epsilon) - terrain_height(x, y - epsilon)) / (2.0 * epsilon)
    along_slope = dz_dx * math.cos(yaw) + dz_dy * math.sin(yaw)
    across_slope = -dz_dx * math.sin(yaw) + dz_dy * math.cos(yaw)
    pitch = -math.atan(along_slope)
    roll = math.atan(across_slope)
    return roll, pitch, yaw


def generate_terrain_textures() -> None:
    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)

    # Image rows are inverted so +Y in Gazebo corresponds to the top of the map.
    height_data = np.zeros((HEIGHTMAP_SAMPLES, HEIGHTMAP_SAMPLES), dtype=np.uint8)
    for row in range(HEIGHTMAP_SAMPLES):
        y = 40.0 - row * WORLD_SIZE / (HEIGHTMAP_SAMPLES - 1)
        for col in range(HEIGHTMAP_SAMPLES):
            x = -40.0 + col * WORLD_SIZE / (HEIGHTMAP_SAMPLES - 1)
            normalised = (raw_terrain_height(x, y) - RAW_HEIGHT_MIN) / (RAW_HEIGHT_MAX - RAW_HEIGHT_MIN)
            height_data[row, col] = round(255.0 * max(0.0, min(1.0, normalised)))
    Image.fromarray(height_data, mode="L").save(HEIGHTMAP_PATH)

    rng = random.Random(DEFAULT_SEED)
    diffuse = Image.new("RGB", (512, 512), (71, 83, 42))
    draw = ImageDraw.Draw(diffuse)
    for _ in range(8500):
        x = rng.randrange(512)
        y = rng.randrange(512)
        base = rng.randrange(-18, 19)
        color = (max(30, 71 + base), max(34, 83 + base), max(22, 42 + base // 2))
        draw.point((x, y), fill=color)
    for _ in range(350):
        x = rng.randrange(512)
        y = rng.randrange(512)
        radius = rng.choice((1, 1, 2, 3))
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(52, 68, 32))
    diffuse.save(DIFFUSE_PATH)

    Image.new("RGB", (16, 16), (128, 128, 255)).save(NORMAL_PATH)


def set_material(
    visual: ET.Element,
    color: tuple[float, float, float, float],
    specular: tuple[float, float, float, float] = (0.05, 0.05, 0.05, 1.0),
) -> None:
    material = child(visual, "material")
    rgba = values(*color)
    child(material, "ambient", rgba)
    child(material, "diffuse", rgba)
    child(material, "specular", values(*specular))


def add_geometry(parent: ET.Element, shape: str, size: tuple[float, ...]) -> None:
    geometry = child(parent, "geometry")
    primitive = child(geometry, shape)
    if shape == "box":
        child(primitive, "size", values(*size))
    elif shape == "cylinder":
        child(primitive, "radius", values(size[0]))
        child(primitive, "length", values(size[1]))
    elif shape == "sphere":
        child(primitive, "radius", values(size[0]))
    else:
        raise ValueError(f"Unsupported shape: {shape}")


def add_link(
    model_node: ET.Element,
    name: str,
    shape: str,
    size: tuple[float, ...],
    xyzrpy: tuple[float, float, float, float, float, float],
    color: tuple[float, float, float, float],
    *,
    collision: bool = True,
    laser_retro: float = 70.0,
    cast_shadows: bool = True,
) -> None:
    link = child(model_node, "link", name=name)
    pose(link, xyzrpy)
    if collision:
        collision_node = child(link, "collision", name="collision")
        add_geometry(collision_node, shape, size)
        child(collision_node, "laser_retro", values(laser_retro))
        surface = child(collision_node, "surface")
        friction = child(surface, "friction")
        ode = child(friction, "ode")
        child(ode, "mu", "1.15")
        child(ode, "mu2", "1.15")
    visual = child(link, "visual", name="visual")
    child(visual, "cast_shadows", "true" if cast_shadows else "false")
    add_geometry(visual, shape, size)
    set_material(visual, color)


def static_model(world: ET.Element, name: str) -> ET.Element:
    node = child(world, "model", name=name)
    child(node, "static", "true")
    child(node, "self_collide", "false")
    return node


def add_primitive(
    world: ET.Element,
    name: str,
    shape: str,
    size: tuple[float, ...],
    xyzrpy: tuple[float, float, float, float, float, float],
    color: tuple[float, float, float, float],
    *,
    collision: bool = True,
) -> ET.Element:
    node = static_model(world, name)
    add_link(node, "body", shape, size, xyzrpy, color, collision=collision)
    return node


def add_cylinder_between(
    model_node: ET.Element,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    color: tuple[float, float, float, float],
    *,
    collision: bool = True,
    laser_retro: float = 45.0,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2, (start[2] + end[2]) / 2)
    add_link(
        model_node,
        name,
        "cylinder",
        (radius, length),
        (*midpoint, 0.0, pitch, yaw),
        color,
        collision=collision,
        laser_retro=laser_retro,
    )


def add_tea_rows(world: ET.Element, rng: random.Random) -> int:
    segment_count = 0
    tea_colors = (
        (0.055, 0.29, 0.055, 1.0),
        (0.075, 0.36, 0.065, 1.0),
        (0.10, 0.42, 0.075, 1.0),
    )
    for row_index, base_y in enumerate(np.arange(-25.5, 28.0, 3.35)):
        row = static_model(world, f"tea_row_{row_index:02d}")
        phase = row_index * 0.43
        for segment_index, x in enumerate(np.arange(-29.0, 30.0, 4.15)):
            # Deterministic missing plants create cross-row escape gaps.
            if (row_index, segment_index) in {
                (2, 5), (2, 6), (5, 10), (5, 11), (8, 3), (8, 4),
                (11, 8), (11, 9), (14, 12), (14, 13),
            }:
                continue
            y = float(base_y + 0.72 * math.sin(x / 11.5 + phase))
            derivative = 0.72 / 11.5 * math.cos(x / 11.5 + phase)
            yaw = math.atan(derivative)
            roll, pitch, yaw = terrain_rpy(yaw, x, y)
            height = 0.78 + rng.uniform(-0.08, 0.12)
            width = 0.82 + rng.uniform(-0.06, 0.08)
            z = terrain_height(x, y) + height / 2 + 0.16
            add_link(
                row,
                f"bush_{segment_index:02d}",
                "box",
                (3.75, width, height),
                (x, y, z, roll, pitch, yaw),
                tea_colors[(row_index + segment_index) % len(tea_colors)],
                laser_retro=55.0,
            )
            segment_count += 1
    return segment_count


def add_tree(world: ET.Element, name: str, x: float, y: float, height: float, crown_radius: float) -> None:
    base_z = terrain_height(x, y)
    node = static_model(world, name)
    trunk_height = height * 0.48
    add_link(
        node,
        "trunk",
        "cylinder",
        (0.16 + crown_radius * 0.07, trunk_height),
        (x, y, base_z + trunk_height / 2, 0, 0, 0),
        (0.24, 0.105, 0.035, 1),
    )
    add_link(
        node,
        "crown_low",
        "sphere",
        (crown_radius,),
        (x, y, base_z + trunk_height + crown_radius * 0.55, 0, 0, 0),
        (0.07, 0.30, 0.045, 1),
    )
    add_link(
        node,
        "crown_high",
        "sphere",
        (crown_radius * 0.78,),
        (x + 0.22, y - 0.13, base_z + height - crown_radius * 0.46, 0, 0, 0),
        (0.10, 0.39, 0.06, 1),
    )


def add_rocks(world: ET.Element, rng: random.Random) -> None:
    positions = (
        (-31.0, -5.5), (-25.5, 12.0), (-18.5, 31.0), (-8.0, -28.5),
        (5.5, 29.0), (13.5, -15.5), (20.0, 21.0), (29.0, -4.0),
        (32.5, 28.5), (35.0, -27.0), (1.5, 10.0),
    )
    colors = ((0.31, 0.30, 0.27, 1), (0.39, 0.37, 0.32, 1), (0.27, 0.29, 0.25, 1))
    for index, (x, y) in enumerate(positions):
        radius = rng.uniform(0.42, 1.05)
        add_primitive(
            world,
            f"rock_{index:02d}",
            "sphere",
            (radius,),
            (x, y, terrain_height(x, y) + radius * 0.48, 0, 0, 0),
            colors[index % len(colors)],
        )


def add_power_line(world: ET.Element) -> tuple[int, int]:
    pole_xy = ((-34.0, -17.0), (-18.0, -9.5), (-1.0, -1.5), (17.0, 7.0), (34.0, 15.0))
    path_yaw = math.atan2(8.0, 17.0)
    pole_height = 8.4
    pole_color = (0.32, 0.25, 0.16, 1)
    metal_color = (0.22, 0.24, 0.24, 1)

    for index, (x, y) in enumerate(pole_xy):
        base_z = terrain_height(x, y)
        node = static_model(world, f"power_pole_{index:02d}")
        add_link(node, "pole", "cylinder", (0.17, pole_height), (x, y, base_z + pole_height / 2, 0, 0, 0), pole_color)
        add_link(
            node,
            "crossarm",
            "box",
            (0.18, 3.5, 0.18),
            (x, y, base_z + pole_height - 0.35, 0, 0, path_yaw),
            pole_color,
        )
        for insulator_index, offset in enumerate((-1.25, 0.0, 1.25)):
            ox = -math.sin(path_yaw) * offset
            oy = math.cos(path_yaw) * offset
            add_link(
                node,
                f"insulator_{insulator_index}",
                "cylinder",
                (0.065, 0.30),
                (x + ox, y + oy, base_z + pole_height - 0.58, 0, 0, 0),
                (0.80, 0.76, 0.60, 1),
            )

    wire_model = static_model(world, "power_conductors")
    wire_count = 0
    for span_index in range(len(pole_xy) - 1):
        x0, y0 = pole_xy[span_index]
        x1, y1 = pole_xy[span_index + 1]
        ground0 = terrain_height(x0, y0)
        ground1 = terrain_height(x1, y1)
        span_yaw = math.atan2(y1 - y0, x1 - x0)
        for conductor_index, offset in enumerate((-1.25, 0.0, 1.25)):
            normal_x = -math.sin(span_yaw) * offset
            normal_y = math.cos(span_yaw) * offset
            previous = None
            for piece in range(9):
                t = piece / 8.0
                x = x0 + (x1 - x0) * t + normal_x
                y = y0 + (y1 - y0) * t + normal_y
                support_z = ground0 + (ground1 - ground0) * t + pole_height - 0.58
                z = support_z - 0.62 * 4.0 * t * (1.0 - t)
                current = (x, y, z)
                if previous is not None:
                    add_cylinder_between(
                        wire_model,
                        f"wire_s{span_index}_c{conductor_index}_p{piece - 1}",
                        previous,
                        current,
                        0.027,
                        metal_color,
                        collision=True,
                        laser_retro=95.0,
                    )
                    wire_count += 1
                previous = current
    return len(pole_xy), wire_count


def add_perimeter_fence(world: ET.Element) -> int:
    fence = static_model(world, "perimeter_fence")
    post_count = 0
    post_color = (0.30, 0.22, 0.12, 1)
    rail_color = (0.36, 0.27, 0.14, 1)
    for side_name, fixed, is_x in (
        ("south", -39.3, True), ("north", 39.3, True),
        ("west", -39.3, False), ("east", 39.3, False),
    ):
        for index, variable in enumerate(np.arange(-36.0, 36.1, 6.0)):
            x, y = (float(variable), fixed) if is_x else (fixed, float(variable))
            base_z = terrain_height(x, y)
            add_link(
                fence,
                f"{side_name}_post_{index:02d}",
                "cylinder",
                (0.075, 1.45),
                (x, y, base_z + 0.725, 0, 0, 0),
                post_color,
            )
            post_count += 1
        # Short rail sections connect sampled terrain heights exactly.
        for index, variable in enumerate(np.arange(-33.0, 33.1, 6.0)):
            start_variable = float(variable - 3.0)
            end_variable = float(variable + 3.0)
            sx, sy = (start_variable, fixed) if is_x else (fixed, start_variable)
            ex, ey = (end_variable, fixed) if is_x else (fixed, end_variable)
            start = (sx, sy, terrain_height(sx, sy) + 0.85)
            end = (ex, ey, terrain_height(ex, ey) + 0.85)
            add_cylinder_between(
                fence,
                f"{side_name}_rail_{index:02d}",
                start,
                end,
                0.045,
                rail_color,
            )
    return post_count


def add_infrastructure(world: ET.Element) -> None:
    # Water tank and concrete pad.
    x, y = 28.0, -24.5
    support_xy = ((x - 1.35, y - 1.35), (x - 1.35, y + 1.35), (x + 1.35, y - 1.35), (x + 1.35, y + 1.35))
    pad_bottom_z = max(terrain_height(px, py) for px, py in support_xy) + 0.12
    tank_support = static_model(world, "water_tank_support")
    for index, (px, py) in enumerate(support_xy):
        ground_z = terrain_height(px, py)
        post_height = max(0.15, pad_bottom_z - ground_z)
        add_link(
            tank_support,
            f"post_{index}",
            "cylinder",
            (0.12, post_height),
            (px, py, ground_z + post_height / 2, 0, 0, 0),
            (0.34, 0.34, 0.32, 1),
        )
    add_primitive(world, "water_tank_pad", "cylinder", (2.15, 0.22), (x, y, pad_bottom_z + 0.11, 0, 0, 0), (0.40, 0.40, 0.38, 1))
    add_primitive(world, "water_tank", "cylinder", (1.55, 2.75), (x, y, pad_bottom_z + 0.22 + 1.375, 0, 0, 0), (0.16, 0.32, 0.38, 1))

    # Small farm shed on short stilts, as commonly built on steep fields.
    x, y = -28.0, 26.5
    shed_support_xy = tuple(
        (x + dx, y + dy)
        for dx in (-2.2, 0.0, 2.2)
        for dy in (-1.55, 1.55)
    )
    base_z = max(terrain_height(px, py) for px, py in shed_support_xy) + 0.12
    shed = static_model(world, "tool_shed")
    add_link(shed, "body", "box", (5.2, 4.0, 3.2), (x, y, base_z + 1.6, 0, 0, 0.08), (0.43, 0.24, 0.12, 1))
    add_link(shed, "roof", "box", (5.8, 4.6, 0.22), (x, y, base_z + 3.32, 0, 0, 0.08), (0.24, 0.20, 0.16, 1))
    add_link(shed, "door", "box", (1.5, 0.10, 2.25), (x + 0.65, y - 2.06, base_z + 1.125, 0, 0, 0.08), (0.15, 0.10, 0.065, 1))
    for index, (px, py) in enumerate(shed_support_xy):
        ground_z = terrain_height(px, py)
        post_height = max(0.15, base_z - ground_z)
        add_link(
            shed,
            f"foundation_post_{index}",
            "cylinder",
            (0.13, post_height),
            (px, py, ground_z + post_height / 2, 0, 0, 0),
            (0.27, 0.20, 0.13, 1),
        )

    # Crates, a fallen log and an irrigation pipe become low, easy-to-miss obstacles.
    for index, (cx, cy, sx, sy, sz) in enumerate((
        (-23.0, -19.5, 1.1, 0.8, 0.75),
        (-21.8, -18.7, 0.9, 0.9, 0.95),
        (22.0, 16.5, 1.3, 1.0, 0.65),
    )):
        add_primitive(
            world,
            f"farm_crate_{index:02d}",
            "box",
            (sx, sy, sz),
            (cx, cy, terrain_height(cx, cy) + sz / 2 + 0.12, 0, 0, 0.18 * index),
            (0.48, 0.30, 0.13, 1),
        )

    log_model = static_model(world, "fallen_log")
    log_start = (-5.5, 18.4, terrain_height(-5.5, 18.4) + 0.28)
    log_end = (-0.2, 20.0, terrain_height(-0.2, 20.0) + 0.33)
    add_cylinder_between(log_model, "trunk", log_start, log_end, 0.26, (0.28, 0.13, 0.045, 1))

    pipe_model = static_model(world, "exposed_irrigation_pipe")
    pipe_start = (7.0, -29.0, terrain_height(7.0, -29.0) + 0.16)
    pipe_end = (19.0, -27.0, terrain_height(19.0, -27.0) + 0.16)
    add_cylinder_between(pipe_model, "pipe", pipe_start, pipe_end, 0.09, (0.055, 0.08, 0.09, 1))

    # Two stone drainage edges create a narrow crossing near the lower field.
    drainage = static_model(world, "drainage_channel")
    for index, y_offset in enumerate((-0.58, 0.58)):
        for piece, x in enumerate(np.arange(-17.5, 7.6, 5.0)):
            y = -31.0 + y_offset
            roll, pitch, yaw = terrain_rpy(0.02, float(x), y)
            z = terrain_height(float(x), y) + 0.24
            add_link(
                drainage,
                f"edge_{index}_{piece}",
                "box",
                (4.85, 0.24, 0.28),
                (float(x), y, z, roll, pitch, yaw),
                (0.34, 0.34, 0.30, 1),
            )


def add_start_goal(world: ET.Element) -> None:
    for name, x, y, color in (
        ("start_pad", -34.0, -33.0, (0.12, 0.38, 0.90, 1)),
        ("goal_pad", 33.0, 33.0, (0.92, 0.18, 0.10, 1)),
    ):
        z = terrain_height(x, y)
        roll, pitch, yaw = terrain_rpy(0.0, x, y)
        add_primitive(world, name, "cylinder", (1.75, 0.10), (x, y, z + 0.18, roll, pitch, yaw), color, collision=False)
        marker = static_model(world, f"{name}_marker")
        add_link(marker, "post", "cylinder", (0.055, 1.7), (x, y, z + 0.85, 0, 0, 0), color, collision=False)
        add_link(marker, "flag", "box", (0.75, 0.035, 0.42), (x + 0.38, y, z + 1.48, 0, 0, 0), color, collision=False)


def build_world(seed: int) -> tuple[ET.Element, dict[str, int]]:
    rng = random.Random(seed)
    sdf = ET.Element("sdf", version="1.6")
    world = child(sdf, "world", name="mountain_tea_garden_80m")
    child(world, "gravity", "0 0 -9.81")
    child(world, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")

    physics = child(world, "physics", name="ode_physics", type="ode")
    child(physics, "max_step_size", "0.002")
    child(physics, "real_time_factor", "1.0")
    child(physics, "real_time_update_rate", "500")
    ode = child(physics, "ode")
    solver = child(ode, "solver")
    child(solver, "type", "quick")
    child(solver, "iters", "35")

    scene = child(world, "scene")
    child(scene, "ambient", "0.48 0.50 0.45 1")
    child(scene, "background", "0.64 0.76 0.87 1")
    child(scene, "shadows", "true")
    child(scene, "grid", "false")

    light = child(world, "light", name="sun", type="directional")
    pose(light, (0, 0, 80, 0, 0, 0))
    child(light, "cast_shadows", "true")
    child(light, "diffuse", "0.95 0.91 0.80 1")
    child(light, "specular", "0.25 0.25 0.22 1")
    child(light, "direction", "-0.42 0.25 -0.87")

    terrain = child(world, "include")
    child(terrain, "uri", "model://mountain_tea_garden_terrain")
    child(terrain, "name", "mountain_tea_garden_terrain")

    tea_segments = add_tea_rows(world, rng)
    add_rocks(world, rng)
    tree_positions = (
        (-35.0, 5.0, 5.6, 1.65), (-33.0, 18.0, 6.4, 1.85),
        (-19.0, -29.5, 5.3, 1.55), (-10.0, 33.5, 6.8, 2.0),
        (9.0, -34.0, 5.8, 1.65), (22.5, 31.5, 7.0, 2.1),
        (34.0, 3.0, 5.5, 1.6), (35.0, -14.0, 6.2, 1.8),
    )
    for index, (x, y, height, radius) in enumerate(tree_positions):
        add_tree(world, f"tree_{index:02d}", x, y, height, radius)
    pole_count, wire_segments = add_power_line(world)
    fence_posts = add_perimeter_fence(world)
    add_infrastructure(world)
    add_start_goal(world)

    gui = child(world, "gui", fullscreen="0")
    camera = child(gui, "camera", name="overview")
    pose(camera, (-52.0, -58.0, 55.0, 0.0, 0.58, 0.76))
    child(camera, "view_controller", "orbit")

    stats = {
        "tea_segments": tea_segments,
        "trees": len(tree_positions),
        "rocks": 11,
        "power_poles": pole_count,
        "wire_segments": wire_segments,
        "fence_posts": fence_posts,
    }
    return sdf, stats


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="deterministic obstacle/plant variation seed")
    parser.add_argument("--output", type=Path, default=WORLD_PATH, help="output .world path")
    args = parser.parse_args()

    generate_terrain_textures()
    sdf, stats = build_world(args.seed)
    indent_xml(sdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(sdf).write(args.output, encoding="utf-8", xml_declaration=True)

    print(f"Generated: {args.output}")
    print(f"Heightmap: {HEIGHTMAP_PATH} ({HEIGHTMAP_SAMPLES} x {HEIGHTMAP_SAMPLES})")
    print("Scene objects: " + ", ".join(f"{key}={value}" for key, value in stats.items()))
    print(f"Terrain elevation: {TERRAIN_BASE_Z:.2f}..{TERRAIN_BASE_Z + HEIGHT_SCALE:.2f} m")
    print(f"Start: (-34, -33, {terrain_height(-34, -33):.2f} m ground)")
    print(f"Goal:  (33, 33, {terrain_height(33, 33):.2f} m ground)")


if __name__ == "__main__":
    main()
