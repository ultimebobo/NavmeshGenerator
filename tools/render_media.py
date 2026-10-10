"""Render still images from NavmeshGenerator GLB exports, without AI.

Requires NumPy and Pillow. Geometry is read directly from the exported buffers;
only cameras, presentation colors, lighting, and captions are added. This reader
accepts the tool's untransformed triangle meshes, not arbitrary glTF files.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np
from PIL import Image, ImageDraw, ImageFont


BACKGROUND = (16, 23, 31)
TEXT = (232, 240, 244)
MUTED = (157, 174, 185)
AUTHORED = (238, 185, 91)
GENERATED = (70, 216, 181)


def read_scene(path):
    """Read the application's tightly packed, world-coordinate GLB meshes."""
    data = path.read_bytes()
    magic, version, length = struct.unpack_from("<III", data)
    if magic != 0x46546C67 or version != 2 or length != len(data):
        raise ValueError("Expected a complete binary glTF scene")
    json_length, json_type = struct.unpack_from("<II", data, 12)
    if json_type != 0x4E4F534A:
        raise ValueError("Missing GLB JSON chunk")
    document = json.loads(data[20:20 + json_length])
    binary_offset = 20 + json_length + 8
    binary = memoryview(data)[binary_offset:]
    types = {5126: "<f4", 5125: "<u4", 5123: "<u2"}

    def accessor(index):
        item = document["accessors"][index]
        view = document["bufferViews"][item["bufferView"]]
        if "byteStride" in view or "sparse" in item:
            raise ValueError("Only tightly packed export accessors are supported")
        width = {"SCALAR": 1, "VEC3": 3}[item["type"]]
        return np.frombuffer(
            binary, dtype=types[item["componentType"]],
            count=item["count"] * width,
            offset=view.get("byteOffset", 0) + item.get("byteOffset", 0),
        ).reshape(-1, width)

    layers = {}
    for node in document["nodes"]:
        if any(key in node for key in ("matrix", "translation", "rotation", "scale")):
            raise ValueError("Expected untransformed world-coordinate export nodes")
        if "children" in node:
            for child in node["children"]:
                layers[child] = node["name"]
    meshes = []
    for index, node in enumerate(document["nodes"]):
        if "mesh" not in node:
            continue
        mesh = document["meshes"][node["mesh"]]
        for primitive in mesh["primitives"]:
            if primitive.get("mode", 4) != 4:
                raise ValueError("Only triangle primitives are supported")
            vertices = accessor(primitive["attributes"]["POSITION"])
            faces = accessor(primitive["indices"]).reshape(-1, 3)
            meshes.append((layers[index], mesh["name"], vertices[faces]))
    return meshes


def layer_triangles(scene, layer):
    parts = [triangles for group, _, triangles in scene if group == layer]
    return np.concatenate(parts) if parts else np.empty((0, 3, 3))


def camera_basis(azimuth, elevation):
    azimuth, elevation = math.radians(azimuth), math.radians(elevation)
    right = np.array([math.cos(azimuth), math.sin(azimuth), 0])
    toward = np.array([
        math.sin(azimuth) * math.cos(elevation),
        -math.cos(azimuth) * math.cos(elevation), math.sin(elevation),
    ])
    up = np.cross(toward, right)
    return np.stack((right, up, toward), axis=1)


def frame(triangles, basis, width, height):
    projected = triangles.reshape(-1, 3) @ basis
    minimum = projected[:, :2].min(axis=0)
    maximum = projected[:, :2].max(axis=0)
    center = (minimum + maximum) * 0.5
    scale = min(width / (maximum[0] - minimum[0]),
                height / (maximum[1] - minimum[1])) * 0.91
    return center, scale


def rasterize(triangles, color, basis, framing, pixels, depth, wire=False, depth_tolerance=0):
    """Barycentric orthographic rasterization with depth-tested triangle edges."""
    height, width = depth.shape
    center, scale = framing
    projected = triangles @ basis
    projected[:, :, 0] = (projected[:, :, 0] - center[0]) * scale + width * 0.5
    projected[:, :, 1] = height * 0.5 - (projected[:, :, 1] - center[1]) * scale
    # Flat directional lighting conveys the actual exported surface normals.
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(norms[:, None], 1e-10)
    light = np.array([-0.4, -0.3, 0.85])
    brightness = 0.55 + 0.45 * np.abs(normals @ light)
    if wire:
        brightness = np.ones(len(triangles))
    colors = np.clip(np.array(color)[None, :] * brightness[:, None], 0, 255).astype(np.uint8)
    for triangle, face_color in zip(projected, colors):
        left = max(0, math.floor(triangle[:, 0].min()))
        right = min(width - 1, math.ceil(triangle[:, 0].max()))
        top = max(0, math.floor(triangle[:, 1].min()))
        bottom = min(height - 1, math.ceil(triangle[:, 1].max()))
        if left > right or top > bottom:
            continue
        a, b, c = triangle
        denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(denominator) < 1e-8:
            continue
        xs = np.arange(left, right + 1)[None, :] + 0.5
        ys = np.arange(top, bottom + 1)[:, None] + 0.5
        wa = ((b[1] - c[1]) * (xs - c[0]) + (c[0] - b[0]) * (ys - c[1])) / denominator
        wb = ((c[1] - a[1]) * (xs - c[0]) + (a[0] - c[0]) * (ys - c[1])) / denominator
        wc = 1 - wa - wb
        z = wa * a[2] + wb * b[2] + wc * c[2]
        target_depth = depth[top:bottom + 1, left:right + 1]
        visible = (wa >= 0) & (wb >= 0) & (wc >= 0) & (z >= target_depth - depth_tolerance)
        target_pixels = pixels[top:bottom + 1, left:right + 1]
        target_pixels[visible] = face_color
        if wire:
            # Convert barycentric distance to screen pixels for consistent line width.
            edge_lengths = [np.linalg.norm(b[:2] - c[:2]),
                            np.linalg.norm(c[:2] - a[:2]),
                            np.linalg.norm(a[:2] - b[:2])]
            distances = [weights * abs(denominator) / max(length, 1e-8)
                         for weights, length in zip((wa, wb, wc), edge_lengths)]
            edge = visible & (np.minimum.reduce(distances) < 0.65)
            target_pixels[edge] = (np.array(face_color) * 0.38).astype(np.uint8)
        target_depth[visible] = z[visible]


def render(scene, navigation, bounds, width, height, azimuth, elevation):
    basis = camera_basis(azimuth, elevation)
    selected = layer_triangles(scene, navigation)
    if bounds:
        xmin, ymin, xmax, ymax = bounds
        def crop(triangles):
            centers = triangles.mean(axis=1)
            return triangles[(centers[:, 0] >= xmin) & (centers[:, 0] <= xmax)
                             & (centers[:, 1] >= ymin) & (centers[:, 1] <= ymax)]
        selected = crop(selected)
    else:
        crop = lambda triangles: triangles
    terrain = crop(layer_triangles(scene, "Terrain"))
    collision = crop(layer_triangles(scene, "Collision"))
    # Both comparison panels share the union of navigation extents, including
    # authored and generated triangles, so their camera framing is identical.
    authored = crop(layer_triangles(scene, "Original NAVM (current cell)"))
    generated = crop(layer_triangles(scene, "Candidate NAVM"))
    framing_navigation = np.concatenate((authored, generated))
    framing = frame(np.concatenate((framing_navigation, terrain, collision)), basis, width, height)
    pixels = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    depth = np.full((height, width), -np.inf, dtype=np.float64)
    rasterize(terrain, (72, 86, 80), basis, framing, pixels, depth)
    rasterize(collision, (118, 130, 138), basis, framing, pixels, depth)
    color = AUTHORED if navigation.startswith("Original") else GENERATED
    # Navigation uses the collision/terrain depth buffer. A floating-point-sized
    # tolerance reveals coplanar faces without lifting geometry or exposing a
    # route through a bridge deck, wall, terrain, or an individual stair tread.
    rasterize(selected, color, basis, framing, pixels, depth, wire=True, depth_tolerance=0.001)
    return Image.fromarray(pixels)


def font(size, bold=False):
    filename = "segoeuib.ttf" if bold else "segoeui.ttf"
    path = Path("C:/Windows/Fonts") / filename
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default(size=size)


def text(draw, xy, value, size=24, color=TEXT, bold=False):
    draw.text(xy, value, font=font(size, bold), fill=color)


def comparison(scene, output):
    width, height = 2560, 1520
    image = Image.new("RGB", (width, height), BACKGROUND)
    draw = ImageDraw.Draw(image)
    text(draw, (60, 30), "NAVMESHGENERATOR  /  EXPORTED SCENE", 22, MUTED)
    text(draw, (60, 67), "Riverwood: authored vs generated", 50, bold=True)
    text(draw, (60, 136), "Official game plugins • Same scene, camera and display settings", 27, MUTED)
    # This area encloses precisely the six selected village CELLs.
    bounds = (16384, -49152, 28672, -40960)
    for x, layer, label, color in (
        (40, "Original NAVM (current cell)", "DEFAULT  /  Bethesda-authored NAVM", AUTHORED),
        (1300, "Candidate NAVM", "GENERATED  /  NavmeshGenerator", GENERATED),
    ):
        text(draw, (x + 20, 202), label, 30, color, True)
        panel = render(scene, layer, bounds, 1220, 1080, 0, 78)
        image.paste(panel, (x, 260))
    draw.line((1280, 204, 1280, 1340), fill=(51, 66, 75), width=2)
    text(draw, (60, 1370), "Depth-tested navigation • Triangle edges visible • Terrain and collision shown in gray", 27, MUTED)
    text(draw, (60, 1420), "Rendered from the program's GLB export. Experimental output; review and test NPC navigation before use.", 25, MUTED)
    image.save(output / "riverwood-navmesh-comparison.png")


def scene_image(scene, output, filename, title, subtitle, bounds=None, azimuth=-24, elevation=48,
                navigation="Candidate NAVM"):
    image = Image.new("RGB", (2400, 1500), BACKGROUND)
    draw = ImageDraw.Draw(image)
    text(draw, (60, 30), "NAVMESHGENERATOR  /  EXPORTED SCENE", 22, MUTED)
    text(draw, (60, 72), title, 48, bold=True)
    text(draw, (60, 139), subtitle, 26, MUTED)
    panel = render(scene, navigation, bounds, 2280, 1120, azimuth, elevation)
    image.paste(panel, (60, 210))
    authored = navigation.startswith("Original")
    draw.rounded_rectangle((64, 1373, 88, 1397), radius=3, fill=AUTHORED if authored else GENERATED)
    text(draw, (104, 1364), "Authored navigation" if authored else "Generated navigation", 26)
    draw.rounded_rectangle((485, 1373, 509, 1397), radius=3, fill=(118, 130, 138))
    text(draw, (525, 1364), "Collision geometry", 26)
    text(draw, (60, 1430), "Unmodified exported triangles • Shared depth testing", 24, MUTED)
    image.save(output / filename)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--riverwood", type=Path)
    parser.add_argument("--scene", "--fixture", dest="scene", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="Navigation scene")
    parser.add_argument("--bounds", nargs=4, type=float, metavar=("XMIN", "YMIN", "XMAX", "YMAX"))
    parser.add_argument("--azimuth", type=float, default=-24)
    parser.add_argument("--elevation", type=float, default=48)
    parser.add_argument("--navigation", choices=("Candidate NAVM", "Original NAVM (current cell)"),
                        default="Candidate NAVM")
    args = parser.parse_args()
    if not args.scene and not args.riverwood:
        parser.error("Provide --scene or --riverwood")
    if args.bounds and (args.bounds[0] >= args.bounds[2] or args.bounds[1] >= args.bounds[3]):
        parser.error("Bounds must have XMIN < XMAX and YMIN < YMAX")
    args.output.mkdir(parents=True, exist_ok=True)
    sources = []
    if args.riverwood:
        comparison(read_scene(args.riverwood), args.output)
        sources.append(("riverwood", args.riverwood))
        print("Rendered Riverwood comparison", flush=True)
    if args.scene:
        scene_image(read_scene(args.scene), args.output, "scene.png", args.title,
                    "Navigation and supporting geometry from the same export.",
                    args.bounds, args.azimuth, args.elevation, args.navigation)
        sources.append(("scene", args.scene))
        print("Rendered scene", flush=True)
    manifest = {
        "renderer": "tools/render_media.py",
        "sources": {name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                    for name, path in sources},
        "images": sorted(path.name for path in args.output.glob("*.png")),
        "presentation": "Orthographic software rendering; shared terrain/collision/navigation depth testing; unmodified exported triangles.",
    }
    (args.output / "render-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
