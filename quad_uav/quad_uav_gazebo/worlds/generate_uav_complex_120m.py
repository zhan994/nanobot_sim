#!/usr/bin/env python3
"""Generate the deterministic 120 m Gazebo Classic UAV radar test world."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


CUSTOM_MATERIALS = {
    "asphalt": "UAVComplex/Asphalt",
    "grass": "UAVComplex/Grass",
    "brick": "UAVComplex/Brick",
    "steel": "UAVComplex/RustedSteel",
}
SCRIPT_URI = "model://uav_complex_scene/materials/scripts"
TEXTURE_URI = "model://uav_complex_scene/materials/textures"


def child(parent: ET.Element, tag: str, text: str | None = None, **attrs: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = text
    return node


def values(*items: float) -> str:
    return " ".join(f"{item:.6g}" for item in items)


def indent_xml(node: ET.Element, level: int = 0) -> None:
    """Pretty-print compatibly with the Python 3.8 shipped by Ubuntu 20.04."""
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


def pose(parent: ET.Element, xyzrpy: tuple[float, float, float, float, float, float]) -> None:
    child(parent, "pose", values(*xyzrpy))


def set_material(
    visual: ET.Element,
    material: str | None,
    color: tuple[float, float, float, float],
    specular: tuple[float, float, float, float],
) -> None:
    node = child(visual, "material")
    if material in CUSTOM_MATERIALS:
        script = child(node, "script")
        child(script, "uri", SCRIPT_URI)
        child(script, "uri", TEXTURE_URI)
        child(script, "name", CUSTOM_MATERIALS[material])
    else:
        rgba = values(*color)
        child(node, "ambient", rgba)
        child(node, "diffuse", rgba)
        child(node, "specular", values(*specular))


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
        raise ValueError(f"Unsupported primitive: {shape}")


def add_link_primitive(
    model: ET.Element,
    name: str,
    shape: str,
    size: tuple[float, ...],
    xyzrpy: tuple[float, float, float, float, float, float],
    *,
    material: str | None = None,
    color: tuple[float, float, float, float] = (0.6, 0.6, 0.6, 1.0),
    collision: bool = True,
    cast_shadows: bool = True,
    laser_retro: float = 100.0,
    specular: tuple[float, float, float, float] = (0.08, 0.08, 0.08, 1.0),
) -> None:
    link = child(model, "link", name=name)
    pose(link, xyzrpy)
    if collision:
        collision_node = child(link, "collision", name="collision")
        add_geometry(collision_node, shape, size)
        child(collision_node, "laser_retro", values(laser_retro))
        surface = child(collision_node, "surface")
        friction = child(surface, "friction")
        ode = child(friction, "ode")
        child(ode, "mu", "1.0")
        child(ode, "mu2", "1.0")
    visual = child(link, "visual", name="visual")
    child(visual, "cast_shadows", "true" if cast_shadows else "false")
    add_geometry(visual, shape, size)
    set_material(visual, material, color, specular)


def model(world: ET.Element, name: str) -> ET.Element:
    node = child(world, "model", name=name)
    child(node, "static", "true")
    child(node, "self_collide", "false")
    return node


def primitive(
    world: ET.Element,
    name: str,
    shape: str,
    size: tuple[float, ...],
    xyzrpy: tuple[float, float, float, float, float, float],
    *,
    material: str | None = None,
    color: tuple[float, float, float, float] = (0.6, 0.6, 0.6, 1.0),
    collision: bool = True,
    cast_shadows: bool = True,
) -> ET.Element:
    node = model(world, name)
    add_link_primitive(
        node,
        "body",
        shape,
        size,
        xyzrpy,
        material=material,
        color=color,
        collision=collision,
        cast_shadows=cast_shadows,
    )
    return node


def add_gate(world: ET.Element, name: str, x: float, y: float, yaw: float, width: float, height: float) -> None:
    node = model(world, name)
    orange = (1.0, 0.22, 0.025, 1.0)
    add_link_primitive(node, "left_post", "box", (0.35, 0.35, height), (0, width / 2, height / 2, 0, 0, 0), color=orange)
    add_link_primitive(node, "right_post", "box", (0.35, 0.35, height), (0, -width / 2, height / 2, 0, 0, 0), color=orange)
    add_link_primitive(node, "top_beam", "box", (0.35, width + 0.35, 0.35), (0, 0, height, 0, 0, 0), color=orange)
    pose(node, (x, y, 0, 0, 0, yaw))


def add_tree(world: ET.Element, name: str, x: float, y: float, height: float, radius: float) -> None:
    node = model(world, name)
    trunk_height = height * 0.48
    add_link_primitive(
        node,
        "trunk",
        "cylinder",
        (radius * 0.22, trunk_height),
        (0, 0, trunk_height / 2, 0, 0, 0),
        color=(0.28, 0.14, 0.055, 1),
    )
    add_link_primitive(
        node,
        "lower_crown",
        "sphere",
        (radius,),
        (0, 0, trunk_height + radius * 0.65, 0, 0, 0),
        color=(0.10, 0.33, 0.08, 1),
    )
    add_link_primitive(
        node,
        "upper_crown",
        "sphere",
        (radius * 0.78,),
        (0.18 * radius, -0.12 * radius, height - radius * 0.5, 0, 0, 0),
        color=(0.16, 0.46, 0.11, 1),
    )
    pose(node, (x, y, 0, 0, 0, 0))


def add_building(
    world: ET.Element,
    name: str,
    x: float,
    y: float,
    sx: float,
    sy: float,
    height: float,
    yaw: float = 0,
) -> None:
    node = model(world, name)
    add_link_primitive(node, "shell", "box", (sx, sy, height), (0, 0, height / 2, 0, 0, 0), material="brick")
    add_link_primitive(
        node,
        "roof",
        "box",
        (sx + 0.35, sy + 0.35, 0.28),
        (0, 0, height + 0.14, 0, 0, 0),
        material="steel",
    )
    add_link_primitive(
        node,
        "door",
        "box",
        (0.12, min(2.8, sy * 0.45), min(3.2, height * 0.55)),
        (sx / 2 + 0.065, 0, min(3.2, height * 0.55) / 2, 0, 0, 0),
        color=(0.09, 0.12, 0.14, 1),
        collision=False,
    )
    if height >= 7:
        add_link_primitive(
            node,
            "roof_unit",
            "box",
            (2.1, 1.5, 0.9),
            (-sx * 0.18, sy * 0.14, height + 0.59, 0, 0, 0),
            material="steel",
        )
    pose(node, (x, y, 0, 0, 0, yaw))


def add_container(world: ET.Element, name: str, x: float, y: float, z: float, yaw: float, color: tuple[float, float, float, float]) -> None:
    node = model(world, name)
    add_link_primitive(node, "container", "box", (6.0, 2.45, 2.55), (0, 0, 1.275, 0, 0, 0), material="steel")
    for offset in (-2.1, -0.7, 0.7, 2.1):
        add_link_primitive(
            node,
            f"rib_{offset:+.1f}",
            "box",
            (0.12, 2.49, 2.58),
            (offset, 0, 1.28, 0, 0, 0),
            color=color,
            collision=False,
        )
    pose(node, (x, y, z, 0, 0, yaw))


def add_vehicle(world: ET.Element, name: str, x: float, y: float, yaw: float, color: tuple[float, float, float, float]) -> None:
    node = model(world, name)
    add_link_primitive(node, "body", "box", (4.2, 1.9, 0.75), (0, 0, 0.85, 0, 0, 0), color=color)
    add_link_primitive(node, "cabin", "box", (1.9, 1.7, 0.8), (-0.45, 0, 1.55, 0, 0, 0), color=(0.16, 0.24, 0.30, 1))
    for index, (wx, wy) in enumerate(((-1.35, -1.0), (-1.35, 1.0), (1.25, -1.0), (1.25, 1.0))):
        add_link_primitive(
            node,
            f"wheel_{index}",
            "cylinder",
            (0.38, 0.24),
            (wx, wy, 0.48, math.pi / 2, 0, 0),
            color=(0.025, 0.025, 0.025, 1),
        )
    pose(node, (x, y, 0, 0, 0, yaw))


def add_power_line(world: ET.Element) -> None:
    node = model(world, "power_line")
    pole_x = (-52, -34, -16, 2, 20, 38, 54)
    for idx, x in enumerate(pole_x):
        add_link_primitive(node, f"pole_{idx}", "cylinder", (0.22, 10.5), (x, 0, 5.25, 0, 0, 0), color=(0.30, 0.22, 0.13, 1))
        add_link_primitive(node, f"crossbar_{idx}", "box", (0.32, 5.2, 0.28), (x, 0, 9.6, 0, 0, 0), color=(0.26, 0.19, 0.12, 1))
    for span in range(len(pole_x) - 1):
        midpoint = (pole_x[span] + pole_x[span + 1]) / 2
        length = pole_x[span + 1] - pole_x[span]
        for wire_idx, (dy, z) in enumerate(((-2.1, 9.35), (0, 9.2), (2.1, 9.35))):
            add_link_primitive(
                node,
                f"wire_{span}_{wire_idx}",
                "cylinder",
                (0.035, length),
                (midpoint, dy, z, 0, math.pi / 2, 0),
                color=(0.055, 0.055, 0.055, 1),
                laser_retro=5.0,
                specular=(0.005, 0.005, 0.005, 1),
            )
    pose(node, (0, -16, 0, 0, 0, 0))


def build_world() -> ET.ElementTree:
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = child(sdf, "world", name="uav_complex_120m")
    child(world, "gravity", "0 0 -9.81")
    child(world, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")
    physics = child(world, "physics", name="default_physics", type="ode")
    child(physics, "max_step_size", "0.001")
    child(physics, "real_time_factor", "1.0")
    child(physics, "real_time_update_rate", "1000")
    ode = child(physics, "ode")
    solver = child(ode, "solver")
    child(solver, "type", "quick")
    child(solver, "iters", "40")

    scene = child(world, "scene")
    child(scene, "ambient", "0.46 0.49 0.52 1")
    child(scene, "background", "0.66 0.78 0.90 1")
    child(scene, "shadows", "true")
    child(scene, "grid", "false")
    fog = child(scene, "fog")
    child(fog, "type", "linear")
    child(fog, "color", "0.72 0.79 0.83 1")
    child(fog, "start", "120")
    child(fog, "end", "240")

    sun = child(world, "light", name="sun", type="directional")
    pose(sun, (0, 0, 80, 0, 0, 0))
    child(sun, "cast_shadows", "true")
    child(sun, "diffuse", "0.94 0.90 0.82 1")
    child(sun, "specular", "0.25 0.25 0.22 1")
    child(sun, "direction", "-0.42 0.18 -0.89")

    world.append(ET.Comment("120 x 120 m textured base and crossing roads"))
    primitive(world, "terrain", "box", (120, 120, 0.2), (0, 0, -0.1, 0, 0, 0), material="grass", cast_shadows=False)
    primitive(world, "main_road", "box", (120, 10, 0.035), (0, 0, 0.018, 0, 0, 0), material="asphalt", collision=False, cast_shadows=False)
    primitive(world, "cross_road", "box", (9, 112, 0.04), (20, 0, 0.021, 0, 0, 0), material="asphalt", collision=False, cast_shadows=False)

    markings = model(world, "road_markings")
    for idx, x in enumerate(range(-54, 58, 8)):
        add_link_primitive(markings, f"dash_x_{idx}", "box", (3.5, 0.16, 0.018), (x, 0, 0.052, 0, 0, 0), color=(0.95, 0.90, 0.66, 1), collision=False, cast_shadows=False)
    for idx, y in enumerate(range(-52, 56, 8)):
        add_link_primitive(markings, f"dash_y_{idx}", "box", (0.16, 3.5, 0.018), (20, y, 0.055, 0, 0, 0), color=(0.95, 0.90, 0.66, 1), collision=False, cast_shadows=False)

    pad = model(world, "takeoff_pad")
    add_link_primitive(pad, "outer", "cylinder", (2.5, 0.045), (0, 0, 0.04, 0, 0, 0), color=(0.055, 0.06, 0.065, 1), collision=False)
    add_link_primitive(pad, "inner", "cylinder", (1.65, 0.055), (0, 0, 0.07, 0, 0, 0), color=(0.92, 0.76, 0.06, 1), collision=False)
    add_link_primitive(pad, "center", "cylinder", (0.95, 0.065), (0, 0, 0.10, 0, 0, 0), color=(0.08, 0.09, 0.10, 1), collision=False)

    world.append(ET.Comment("Perimeter fence: thin rails and posts remain visible to lidar"))
    fence = model(world, "perimeter_fence")
    for idx, coord in enumerate(range(-55, 56, 10)):
        for side, x, y in (("n", coord, 59), ("s", coord, -59), ("e", 59, coord), ("w", -59, coord)):
            add_link_primitive(fence, f"post_{side}_{idx}", "cylinder", (0.12, 3.2), (x, y, 1.6, 0, 0, 0), color=(0.20, 0.22, 0.22, 1))
    for side, x, y, sx, sy in (
        ("north", 0, 59, 118, 0.10),
        ("south", 0, -59, 118, 0.10),
        ("east", 59, 0, 0.10, 118),
        ("west", -59, 0, 0.10, 118),
    ):
        for level in (0.65, 1.55, 2.45):
            add_link_primitive(fence, f"rail_{side}_{level}", "box", (sx, sy, 0.09), (x, y, level, 0, 0, 0), color=(0.27, 0.30, 0.30, 1))

    world.append(ET.Comment("Forward +X radar corridor: targets at 12, 20-42, 48 and 54 m"))
    add_gate(world, "gate_12m", 12, 0, 0, 4.4, 4.2)
    for idx, (x, y, radius, height) in enumerate(
        ((20, -2.6, 0.45, 3.8), (25, 2.2, 0.60, 5.2), (31, -1.8, 0.42, 2.6), (37, 2.8, 0.70, 6.2), (42, -2.4, 0.50, 4.5))
    ):
        primitive(world, f"corridor_pillar_{idx}", "cylinder", (radius, height), (x, y, height / 2, 0, 0, 0), color=(0.06, 0.36 + idx * 0.05, 0.78, 1))
    add_gate(world, "overhead_gate_48m", 48, 0, 0, 5.2, 6.0)
    primitive(world, "gap_wall_north_54m", "box", (0.45, 4.1, 5.5), (54, 4.55, 2.75, 0, 0, 0), material="brick")
    primitive(world, "gap_wall_south_54m", "box", (0.45, 4.1, 5.5), (54, -4.55, 2.75, 0, 0, 0), material="brick")
    for idx, (x, y) in enumerate(((16, 3.8), (18, 3.4), (20, 3.9), (23, -3.7), (27, -3.8), (35, 3.9))):
        primitive(world, f"corridor_barrel_{idx}", "cylinder", (0.34, 0.9), (x, y, 0.45, 0, 0, 0), color=(0.92, 0.36, 0.025, 1))

    world.append(ET.Comment("Northwest industrial district"))
    add_building(world, "warehouse_main", -31, 29, 22, 15, 8.5, 0.04)
    add_building(world, "warehouse_annex", -43, 42, 10, 9, 5.5, -0.08)
    for idx, (x, y, radius, height) in enumerate(((-48, 22, 2.2, 8), (-42, 20, 1.7, 6.5), (-36, 19, 2.0, 7.4))):
        primitive(world, f"industrial_silo_{idx}", "cylinder", (radius, height), (x, y, height / 2, 0, 0, 0), material="steel")
    for idx, (x, y, height) in enumerate(((-38, 32, 11), (-27, 32, 12.5), (-22, 26, 9.5))):
        primitive(world, f"smokestack_{idx}", "cylinder", (0.55, height), (x, y, height / 2, 0, 0, 0), material="brick")
    pipe_rack = model(world, "industrial_pipe_rack")
    for idx, x in enumerate(range(-52, -15, 6)):
        add_link_primitive(pipe_rack, f"support_{idx}", "box", (0.3, 5.0, 4.0), (x, 12.5, 2, 0, 0, 0), color=(0.32, 0.35, 0.34, 1))
    for idx, (y, z) in enumerate(((11.2, 3.1), (12.5, 3.8), (13.8, 3.25))):
        add_link_primitive(pipe_rack, f"pipe_{idx}", "cylinder", (0.24, 38), (-33.5, y, z, 0, math.pi / 2, 0), color=(0.68, 0.26 + 0.12 * idx, 0.05, 1))

    world.append(ET.Comment("Northeast container yard and loading traffic"))
    container_colors = ((0.80, 0.18, 0.08, 1), (0.05, 0.35, 0.58, 1), (0.83, 0.58, 0.06, 1), (0.12, 0.48, 0.30, 1))
    container_positions = (
        (29, 18, 0, 0), (36, 18, 0, 0), (43, 18, 0, 0),
        (29, 23, 0, 0), (36, 23, 0, 0), (43, 23, 0, 0),
        (31, 31, 0, 0.16), (39, 32, 0, -0.10), (48, 31, 0, 0.08),
        (29, 18, 2.55, 0), (36, 23, 2.55, 0), (43, 18, 2.55, 0),
        (50, 42, 0, math.pi / 2), (45, 46, 0, math.pi / 2),
    )
    for idx, (x, y, z, yaw) in enumerate(container_positions):
        add_container(world, f"container_{idx}", x, y, z, yaw, container_colors[idx % len(container_colors)])
    add_vehicle(world, "loading_truck", 27, 45, math.pi / 2, (0.82, 0.80, 0.72, 1))
    add_vehicle(world, "yard_car", 18, 36, 0.25, (0.72, 0.12, 0.08, 1))

    world.append(ET.Comment("Southwest urban block with mixed-height structures"))
    for data in (
        ("urban_a", -48, -44, 10, 9, 9, 0.04),
        ("urban_b", -35, -45, 11, 8, 13, -0.05),
        ("urban_c", -22, -45, 9, 10, 7, 0.03),
        ("urban_d", -48, -30, 9, 11, 6, -0.06),
        ("urban_e", -35, -30, 12, 10, 10.5, 0.06),
        ("urban_f", -21, -29, 8, 12, 8, -0.04),
    ):
        add_building(world, *data)
    add_vehicle(world, "urban_van", -28, -17.5, 0.08, (0.88, 0.88, 0.82, 1))
    add_vehicle(world, "urban_car", -45, -18.5, -0.18, (0.05, 0.20, 0.52, 1))
    for idx, (x, y, z) in enumerate(((-42, -36, 2.2), (-29, -36, 3.2), (-16, -37, 1.7))):
        primitive(world, f"urban_sign_{idx}", "box", (0.18, 3.4, z), (x, y, z / 2, 0, 0, 0), color=(0.10, 0.48, 0.62, 1))

    world.append(ET.Comment("Southeast woodland, boulders and uneven-height canopy"))
    tree_positions = (
        (29, -18), (37, -20), (46, -18), (54, -23),
        (25, -28), (34, -30), (43, -29), (51, -34),
        (27, -39), (36, -40), (46, -42), (55, -45),
        (24, -50), (33, -52), (43, -51), (52, -54),
    )
    for idx, (x, y) in enumerate(tree_positions):
        add_tree(world, f"tree_{idx}", x, y, 6.0 + (idx % 5) * 0.75, 1.6 + (idx % 3) * 0.28)
    for idx, (x, y, r) in enumerate(((22, -23, 1.1), (40, -24, 0.8), (50, -28, 1.35), (30, -35, 0.65), (47, -48, 1.0), (56, -38, 0.75))):
        primitive(world, f"boulder_{idx}", "sphere", (r,), (x, y, r * 0.72, 0, 0, 0), color=(0.31, 0.32, 0.29, 1))

    world.append(ET.Comment("South-central concrete maze and low-altitude clutter"))
    maze = model(world, "concrete_maze")
    wall_color = (0.58, 0.60, 0.58, 1)
    maze_walls = (
        (3, -28, 0.35, 18, 2.7, 0),
        (11, -37, 16, 0.35, 3.8, 0),
        (-5, -40, 0.35, 16, 2.2, 0),
        (5, -48, 20, 0.35, 4.5, 0),
        (14, -50, 0.35, 12, 2.8, 0),
        (-12, -30, 13, 0.35, 3.2, 0.22),
        (-10, -50, 0.35, 12, 5.0, 0),
    )
    for idx, (x, y, sx, sy, h, yaw) in enumerate(maze_walls):
        add_link_primitive(maze, f"wall_{idx}", "box", (sx, sy, h), (x, y, h / 2, 0, 0, yaw), color=wall_color)
    for idx, (x, y, h) in enumerate(((0, -20, 2.0), (6, -22, 3.5), (12, -20, 1.5), (-7, -24, 4.3))):
        primitive(world, f"maze_block_{idx}", "box", (2.2, 2.2, h), (x, y, h / 2, 0, 0, idx * 0.17), color=(0.72, 0.58, 0.10, 1))

    world.append(ET.Comment("Power line crossing the scene at y=-16 m"))
    add_power_line(world)

    world.append(ET.Comment("Central construction crane and suspended high obstacle"))
    crane = model(world, "construction_crane")
    for idx, (x, y) in enumerate(((9, 8), (15, 8), (9, 14), (15, 14))):
        add_link_primitive(crane, f"leg_{idx}", "box", (0.45, 0.45, 12), (x, y, 6, 0, 0, 0), color=(0.95, 0.68, 0.04, 1))
    add_link_primitive(crane, "top_x1", "box", (18, 0.45, 0.45), (16, 8, 12, 0, 0, 0), color=(0.95, 0.68, 0.04, 1))
    add_link_primitive(crane, "top_x2", "box", (18, 0.45, 0.45), (16, 14, 12, 0, 0, 0), color=(0.95, 0.68, 0.04, 1))
    add_link_primitive(crane, "top_y1", "box", (0.45, 6, 0.45), (9, 11, 12, 0, 0, 0), color=(0.95, 0.68, 0.04, 1))
    add_link_primitive(crane, "top_y2", "box", (0.45, 6, 0.45), (15, 11, 12, 0, 0, 0), color=(0.95, 0.68, 0.04, 1))
    add_link_primitive(crane, "suspended_load", "box", (3.2, 3.2, 2.2), (21, 11, 6.2, 0, 0, 0.18), material="steel")
    add_link_primitive(crane, "load_cable", "cylinder", (0.045, 4.8), (21, 11, 9.7, 0, 0, 0), color=(0.04, 0.04, 0.04, 1))

    gui = child(world, "gui", fullscreen="false")
    camera = child(gui, "camera", name="complex_world_overview")
    pose(camera, (91, -91, 70, 0, 0.55, 2.35))
    child(camera, "view_controller", "orbit")
    child(camera, "projection_type", "perspective")

    return ET.ElementTree(sdf)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("uav_complex_120m.world"),
        help="Generated .world file",
    )
    args = parser.parse_args()
    tree = build_world()
    indent_xml(tree.getroot())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Generated {args.output}")


if __name__ == "__main__":
    main()
