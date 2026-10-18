"""Render still images from NavmeshGenerator GLB exports, without AI.

Requires NumPy and Pillow. Geometry is read directly from the exported buffers;
presentation adds cameras, colors, lighting, captions and interior cutaways. This reader
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
WATER = (13, 76, 255)
# Skyrim world units retained above the highest selected navigation vertex.
INTERIOR_CUT_MARGIN = 64.0


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


def crop_triangles(triangles, bounds):
    """Select faces by world XY centroid, including faces hidden by geometry."""
    if bounds is None:
        return triangles
    xmin, ymin, xmax, ymax = bounds
    centers = triangles.mean(axis=1)
    return triangles[(centers[:, 0] >= xmin) & (centers[:, 0] <= xmax)
                     & (centers[:, 1] >= ymin) & (centers[:, 1] <= ymax)]


def clip_support(triangles, cut_height):
    """Clip supporting surfaces to world Z <= cut_height, preserving their winding.

    Intersections stay on the exported surfaces. Navigation is never clipped;
    this is a presentation cutaway, not a change to the source scene.
    """
    if cut_height is None or not len(triangles):
        return triangles
    below = triangles[:, :, 2] <= cut_height
    retained = triangles[below.all(axis=1)]
    crossing = triangles[below.any(axis=1) & ~below.all(axis=1)]
    fragments = []
    # A horizontal half-space clips each crossing triangle to a triangle or quad.
    # Traverse its ordered edges so triangulation keeps the original normal.
    for triangle in crossing:
        polygon = []
        previous = triangle[-1]
        for current in triangle:
            previous_below = previous[2] <= cut_height
            current_below = current[2] <= cut_height
            if previous_below != current_below:
                fraction = (cut_height - previous[2]) / (current[2] - previous[2])
                intersection = previous + fraction * (current - previous)
                intersection[2] = cut_height
                polygon.append(intersection)
            if current_below:
                polygon.append(current)
            previous = current
        for index in range(1, len(polygon) - 1):
            fragment = np.array([polygon[0], polygon[index], polygon[index + 1]])
            if np.linalg.norm(np.cross(fragment[1] - fragment[0], fragment[2] - fragment[0])) > 0:
                fragments.append(fragment)
    if fragments:
        return np.concatenate((retained, np.array(fragments)))
    return retained


def navigation_triangles(scene, bounds):
    """Use both navigation layers for a shared comparison cut and camera."""
    return np.concatenate([
        crop_triangles(layer_triangles(scene, layer), bounds)
        for layer in ("Original NAVM (current cell)", "Candidate NAVM")
    ])


def selection_metadata(path):
    """Read selected CELL evidence independently of the requested camera mode."""
    metadata_path = Path(str(path) + ".metadata.json")
    if not metadata_path.exists():
        return [], None
    data = metadata_path.read_bytes()
    metadata = json.loads(data)["metadata"]
    cells = metadata.get("selected_cells")
    if cells is None:
        cells = [metadata["selected_cell"]] if metadata.get("selected_cell") else []
    if not isinstance(cells, list) or any(not isinstance(cell, dict) for cell in cells):
        raise ValueError("Scene metadata must contain selected CELL objects")
    # Aliases and repeated entries describe one CELL, not additional scene cells.
    unique = {}
    for index, cell in enumerate(cells):
        identity = str(cell.get("form_id", index)).upper()
        unique[identity] = cell
    source = {"path": str(metadata_path), "sha256": hashlib.sha256(data).hexdigest()}
    return list(unique.values()), source


def cell_caption(cell_count, cropped=False):
    """Keep selection counts distinct from a crop whose CELL ownership is unknown."""
    if cell_count is None:
        return "Cell count unavailable"
    label = "cell" if cell_count == 1 else "cells"
    return f"{cell_count:,} selected {label}" + (" (before crop)" if cropped else "")


def presentation(scene, path, view, cut_height, bounds, azimuth, elevation,
                 exterior_elevation=48, cell_count=None):
    """Resolve interior cutaways from the adjacent export metadata or CLI override.

    Auto detection requires all selected cells to be interiors. Missing metadata
    keeps exterior presentation. Both navigation layers define the automatic cut;
    empty selections require an explicit height rather than an invented floor.
    """
    cells, metadata_source = selection_metadata(path)
    count_source = "unavailable"
    if cell_count is not None:
        count_source = "override"
    elif cells:
        cell_count = len(cells)
        count_source = "metadata"
    if view == "auto" and cut_height is None:
        interior = bool(cells) and all(cell.get("is_interior") is True for cell in cells)
        view = "interior" if interior else "exterior"
    elif view == "auto":
        view = "interior"
    if view == "interior" and cut_height is None:
        navigation = navigation_triangles(scene, bounds)
        if not len(navigation):
            raise ValueError("Interior cutaway needs navigation in the selected bounds or --cut-height")
        cut_height = float(navigation[:, :, 2].max()) + INTERIOR_CUT_MARGIN
    return {"view": view, "cut_height": cut_height, "azimuth": azimuth,
            "elevation": elevation if elevation is not None else (70 if view == "interior" else exterior_elevation),
            "metadata_source": metadata_source, "cell_count": cell_count,
            "cell_count_source": count_source,
            "cell_count_before_crop": cell_count is not None and bounds is not None and count_source != "override"}


def render(scene, navigation, bounds, width, height, azimuth, elevation, cut_height=None):
    basis = camera_basis(azimuth, elevation)
    terrain = clip_support(crop_triangles(layer_triangles(scene, "Terrain"), bounds), cut_height)
    collision = clip_support(crop_triangles(layer_triangles(scene, "Collision"), bounds), cut_height)
    # Both comparison panels share the union of navigation extents, including
    # authored and generated triangles, so their camera framing is identical.
    framing_navigation = navigation_triangles(scene, bounds)
    framing = frame(np.concatenate((framing_navigation, terrain, collision)), basis, width, height)
    pixels = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    depth = np.full((height, width), -np.inf, dtype=np.float64)
    rasterize(terrain, (72, 86, 80), basis, framing, pixels, depth)
    rasterize(collision, (118, 130, 138), basis, framing, pixels, depth)
    color = AUTHORED if navigation.startswith("Original") else GENERATED
    # Navigation uses the collision/terrain depth buffer. A floating-point-sized
    # tolerance reveals coplanar faces without lifting geometry or exposing a
    # route through a bridge deck, wall, terrain, or an individual stair tread.
    for layer, name, triangles in scene:
        if layer != navigation:
            continue
        classification = name.rsplit(": ", 1)[-1]
        face_color = WATER if navigation == "Candidate NAVM" and classification in (
            "water", "water_preferred_path") else color
        rasterize(crop_triangles(triangles, bounds), face_color, basis, framing,
                  pixels, depth, wire=True, depth_tolerance=0.001)
    return Image.fromarray(pixels)


def font(size, bold=False):
    filename = "segoeuib.ttf" if bold else "segoeui.ttf"
    path = Path("C:/Windows/Fonts") / filename
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default(size=size)


def text(draw, xy, value, size=24, color=TEXT, bold=False, anchor=None):
    draw.text(xy, value, font=font(size, bold), fill=color, anchor=anchor)


def comparison(scene, output, title="Authored vs generated", attribution="",
               filename="navmesh-comparison.png", bounds=None, azimuth=0, elevation=78,
               note="Experimental output; review and test NPC navigation before use.", cut_height=None,
               cell_count=None, cell_count_before_crop=False):
    """Compare selected CELL navigation with identical cameras and exact face counts.

    Attribution describes the actual winning input, including third-party edits.
    Counts include occluded faces and use the same centroid crop as rendering.
    Neighboring NAVM and connection-marker triangles are excluded.
    """
    width, height = 2560, 1520
    image = Image.new("RGB", (width, height), BACKGROUND)
    draw = ImageDraw.Draw(image)
    text(draw, (60, 30), "NAVMESHGENERATOR  /  EXPORTED SCENE", 22, MUTED)
    text(draw, (60, 67), title, 50, bold=True)
    text(draw, (60, 136), attribution, 27, MUTED)
    text(draw, (width - 60, 38), cell_caption(cell_count, cell_count_before_crop), 22, MUTED,
         anchor="ra")
    counts = {}
    for x, layer, label, color in (
        (40, "Original NAVM (current cell)", "AUTHORED  /  Winning input NAVM", AUTHORED),
        (1300, "Candidate NAVM", "GENERATED  /  NavmeshGenerator", GENERATED),
    ):
        text(draw, (x + 20, 202), label, 30, color, True)
        count = len(crop_triangles(layer_triangles(scene, layer), bounds))
        counts["authored" if layer.startswith("Original") else "generated"] = count
        text(draw, (x + 20, 247), f"{count:,} triangles", 27, color)
        panel = render(scene, layer, bounds, 1220, 1030, azimuth, elevation, cut_height)
        image.paste(panel, (x, 300))
    draw.line((1280, 204, 1280, 1340), fill=(51, 66, 75), width=2)
    support_view = "Interior cutaway" if cut_height is not None else "Same scene and camera"
    text(draw, (60, 1360), support_view + " • Depth-tested triangles • Counts include occluded faces", 27, MUTED)
    text(draw, (60, 1410), "Direct GLB render • Terrain and collision in gray • " + note, 25, MUTED)
    image.save(output / filename)
    return counts


def scene_image(scene, output, filename, title, subtitle, bounds=None, azimuth=-24, elevation=48,
                navigation="Candidate NAVM", cut_height=None, cell_count=None, cell_count_before_crop=False):
    image = Image.new("RGB", (2400, 1500), BACKGROUND)
    draw = ImageDraw.Draw(image)
    text(draw, (60, 30), "NAVMESHGENERATOR  /  EXPORTED SCENE", 22, MUTED)
    text(draw, (60, 72), title, 48, bold=True)
    text(draw, (60, 139), subtitle, 26, MUTED)
    text(draw, (2340, 38), cell_caption(cell_count, cell_count_before_crop), 22, MUTED, anchor="ra")
    panel = render(scene, navigation, bounds, 2280, 1120, azimuth, elevation, cut_height)
    image.paste(panel, (60, 210))
    authored = navigation.startswith("Original")
    draw.rounded_rectangle((64, 1373, 88, 1397), radius=3, fill=AUTHORED if authored else GENERATED)
    text(draw, (104, 1364), "Authored navigation" if authored else "Generated navigation", 26)
    draw.rounded_rectangle((485, 1373, 509, 1397), radius=3, fill=(118, 130, 138))
    text(draw, (525, 1364), "Collision geometry", 26)
    geometry_note = "Interior cutaway • Unmodified navigation" if cut_height is not None else "Unmodified exported triangles"
    text(draw, (60, 1430), geometry_note + " • Shared depth testing", 24, MUTED)
    image.save(output / filename)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--riverwood", type=Path)
    parser.add_argument("--comparison", type=Path, help="Authored/generated comparison from any exported scene")
    parser.add_argument("--scene", "--fixture", dest="scene", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="Navigation scene")
    parser.add_argument("--attribution", help="Winning game plugins and third-party edits, printed on comparisons")
    parser.add_argument("--filename", default="navmesh-comparison.png")
    parser.add_argument("--note", default="Experimental output; review and test NPC navigation before use.")
    parser.add_argument("--bounds", nargs=4, type=float, metavar=("XMIN", "YMIN", "XMAX", "YMAX"))
    parser.add_argument("--azimuth", type=float, default=-24)
    parser.add_argument("--elevation", type=float, help="Camera elevation in degrees; interiors default to a slight incline")
    parser.add_argument("--view", choices=("auto", "interior", "exterior"), default="auto",
                        help="Auto detects interiors from the adjacent GLB metadata sidecar")
    parser.add_argument("--cut-height", type=float, metavar="Z",
                        help="Interior support cut in Skyrim world Z units; implies interior in auto mode")
    parser.add_argument("--navigation", choices=("Candidate NAVM", "Original NAVM (current cell)"),
                        default="Candidate NAVM")
    parser.add_argument("--cell-count", type=int,
                        help="Verified displayed CELL count; otherwise read the selected cells from metadata")
    args = parser.parse_args()
    if not args.scene and not args.riverwood and not args.comparison:
        parser.error("Provide --scene, --comparison or --riverwood")
    if args.comparison and not args.attribution:
        parser.error("--comparison requires --attribution naming the winning inputs")
    if Path(args.filename).name != args.filename or not args.filename.lower().endswith(".png"):
        parser.error("--filename must be a PNG filename without a directory")
    if args.bounds and (args.bounds[0] >= args.bounds[2] or args.bounds[1] >= args.bounds[3]):
        parser.error("Bounds must have XMIN < XMAX and YMIN < YMAX")
    numeric_settings = [args.azimuth]
    numeric_settings.extend(value for value in (args.elevation, args.cut_height) if value is not None)
    numeric_settings.extend(args.bounds or [])
    if not all(math.isfinite(value) for value in numeric_settings):
        parser.error("Camera angles, bounds and cut height must be finite")
    if args.cut_height is not None and args.view == "exterior":
        parser.error("--cut-height requires --view auto or interior")
    if args.cell_count is not None and args.cell_count < 1:
        parser.error("--cell-count must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    sources = []
    comparisons = {}
    renders = {}

    def load_presentation(path, filename, bounds, exterior_elevation=48, azimuth=None):
        scene = read_scene(path)
        try:
            settings = presentation(scene, path, args.view, args.cut_height, bounds,
                                    args.azimuth if azimuth is None else azimuth,
                                    args.elevation, exterior_elevation, args.cell_count)
        except (ValueError, KeyError) as error:
            parser.error(str(error))
        renders[filename] = settings
        return scene, settings

    if args.riverwood:
        bounds = (16384, -49152, 28672, -40960)
        scene, settings = load_presentation(args.riverwood, "riverwood-navmesh-comparison.png", bounds, 78, 0)
        comparisons["riverwood-navmesh-comparison.png"] = comparison(
            scene, args.output, "Riverwood: authored vs generated",
            args.attribution or "Official game plugins • Same scene, camera and display settings",
            "riverwood-navmesh-comparison.png", bounds, settings["azimuth"], settings["elevation"],
            cut_height=settings["cut_height"], cell_count=settings["cell_count"],
            cell_count_before_crop=settings["cell_count_before_crop"])
        sources.append(("riverwood", args.riverwood))
        print("Rendered Riverwood comparison: " + cell_caption(settings["cell_count"],
                                                               settings["cell_count_before_crop"]), flush=True)
    if args.comparison:
        scene, settings = load_presentation(args.comparison, args.filename, args.bounds)
        comparisons[args.filename] = comparison(
            scene, args.output, args.title, args.attribution,
            args.filename, args.bounds, settings["azimuth"], settings["elevation"], args.note,
            settings["cut_height"], settings["cell_count"], settings["cell_count_before_crop"])
        sources.append(("comparison", args.comparison))
        print("Rendered comparison: " + cell_caption(settings["cell_count"],
                                                      settings["cell_count_before_crop"]), flush=True)
    if args.scene:
        scene, settings = load_presentation(args.scene, "scene.png", args.bounds)
        scene_image(scene, args.output, "scene.png", args.title,
                    "Navigation and supporting geometry from the same export.",
                    args.bounds, settings["azimuth"], settings["elevation"], args.navigation,
                    settings["cut_height"], settings["cell_count"], settings["cell_count_before_crop"])
        sources.append(("scene", args.scene))
        print("Rendered scene: " + cell_caption(settings["cell_count"],
                                                settings["cell_count_before_crop"]), flush=True)
    manifest = {
        "renderer": "tools/render_media.py",
        "sources": {name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                    for name, path in sources},
        "images": sorted(path.name for path in args.output.glob("*.png")),
        "comparisons": comparisons,
        "renders": renders,
        "settings": {"title": args.title, "attribution": args.attribution, "note": args.note,
                     "bounds": args.bounds, "azimuth": args.azimuth, "elevation": args.elevation,
                     "view": args.view, "cut_height": args.cut_height, "cell_count": args.cell_count},
        "presentation": "Orthographic software rendering; shared terrain/collision/navigation depth testing; interior support cutaways; unmodified navigation.",
    }
    (args.output / "render-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
