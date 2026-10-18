"""Create a Skyrim SE terrain and follower-door test plugin without authored navigation."""

# Binary layouts follow the WRLD/CELL/LAND/REFR definitions in xEdit:
# https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.5/Core/wbDefinitionsTES5.pas

import argparse
import json
import math
from pathlib import Path
import re
import struct


PLUGIN_NAME = "NavmeshTestWorld.esp"
CELL_SIZE = 4096
SAMPLE_SPACING = 128
LAND_SIDE = 33
HEIGHT_UNIT = 8
FIRST_OBJECT_ID = 0x800
SELF_INDEX = 0x01000000  # Skyrim.esm is master zero; this full ESP owns slot one.

# Vanilla identities, verified against Skyrim.esm. These remain references into
# the installed game; no Bethesda records, meshes or textures are redistributed.
COC_MARKER = 0x00000032  # STAT COCMarkerHeading
LAND_TEXTURE = 0x00000C14  # LTEX LDirt01
CLIMATE = 0x00000812  # CLMT SkyrimClimate
TELEPORT_DOOR = 0x0004203A  # DOOR ImpDoorSingleLoad01
DOOR_SURROUND = 0x0003E1FD  # STAT ImpFreeDoor01


def subrecord(signature, payload):
    return signature.encode("ascii") + struct.pack("<H", len(payload)) + payload


def text_subrecord(signature, value):
    return subrecord(signature, value.encode("ascii") + b"\0")


def record(signature, form_id, payload, flags=0):
    # The final header word contains form version followed by an unused word.
    return (signature.encode("ascii")
            + struct.pack("<IIIIHH", len(payload), flags, form_id, 0, 44, 0)
            + payload)


def group(label, group_type, payload):
    return b"GRUP" + struct.pack("<IIIII", len(payload) + 24, label, group_type, 0, 0) + payload


def coordinate_label(x, y):
    """Exterior block labels store signed Y in the low word and X in the high word."""
    return int.from_bytes(struct.pack("<hh", y, x), "little")


def terrain_height(sample_x, sample_y):
    """Return Skyrim world Z in height units on a global, cell-independent sample grid."""
    # A level central area and gentle ramps exercise flat/sloped seams and shared
    # corners. Global coordinates make duplicated boundary samples identical.
    return 32 + max(0, abs(sample_x - 16) - 16) + max(0, abs(sample_y - 16) - 16)


def land_payload(cell_x, cell_y):
    heights = []
    normals = bytearray()
    for y in range(LAND_SIDE):
        for x in range(LAND_SIDE):
            global_x = cell_x * (LAND_SIDE - 1) + x
            global_y = cell_y * (LAND_SIDE - 1) + y
            heights.append(terrain_height(global_x, global_y))

            # VNML components are signed bytes. Sample derivatives on the same
            # global field so lighting normals also agree on shared borders.
            slope_x = (terrain_height(global_x + 1, global_y)
                       - terrain_height(global_x - 1, global_y)) * HEIGHT_UNIT / (2 * SAMPLE_SPACING)
            slope_y = (terrain_height(global_x, global_y + 1)
                       - terrain_height(global_x, global_y - 1)) * HEIGHT_UNIT / (2 * SAMPLE_SPACING)
            length = math.sqrt(slope_x * slope_x + slope_y * slope_y + 1)
            normals.extend(struct.pack("<bbb", round(-slope_x / length * 127),
                                       round(-slope_y / length * 127), round(127 / length)))

    # VHGT's first delta is relative to its float offset. Each later row starts
    # relative to the preceding row's first sample, not its last sample.
    deltas = bytearray()
    for index, height in enumerate(heights):
        if index == 0:
            previous = heights[0]
        elif index % LAND_SIDE == 0:
            previous = heights[index - LAND_SIDE]
        else:
            previous = heights[index - 1]
        deltas.extend(struct.pack("<b", height - previous))
    height_map = struct.pack("<f", heights[0]) + deltas + bytes(3)
    payload = subrecord("DATA", struct.pack("<I", 7))
    payload += subrecord("VNML", normals)
    payload += subrecord("VHGT", height_map)
    payload += subrecord("VCLR", b"\xff\xff\xff" * LAND_SIDE * LAND_SIDE)
    # Every landscape quadrant has a base texture and no alpha layers.
    for quadrant in range(4):
        payload += subrecord("BTXT", struct.pack("<IBBh", LAND_TEXTURE, quadrant, 0, -1))
    payload += subrecord("VTEX", struct.pack("<I", LAND_TEXTURE))
    return payload


def cell_editor_id(base_name, x, y):
    if x == 0 and y == 0:
        return base_name
    return f"{base_name}_X{'m' if x < 0 else 'p'}{abs(x)}_Y{'m' if y < 0 else 'p'}{abs(y)}"


def build_plugin(cell_name, size):
    """Return the full ESP and file-local identities for terrain seams and a two-way door route."""
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,79}", cell_name):
        raise ValueError("Cell name must start with an ASCII letter and contain only letters, "
                         "digits or underscores (at most 80 characters)")
    if not 2 <= size <= 16:
        raise ValueError("Grid size must be between 2 and 16 cells per side")

    next_object_id = FIRST_OBJECT_ID
    record_count = 0
    group_count = 0

    def allocate():
        nonlocal next_object_id
        result = SELF_INDEX | next_object_id
        next_object_id += 1
        return result

    def emit_record(signature, form_id, payload, flags=0):
        nonlocal record_count
        record_count += 1
        return record(signature, form_id, payload, flags)

    def emit_group(label, group_type, payload):
        nonlocal group_count
        group_count += 1
        return group(label, group_type, payload)

    lower = -(size // 2)
    upper = lower + size - 1
    world_name = cell_name + "World"
    destination_name = cell_name + "DoorDestination"
    coordinates = [(x, y) for y in range(lower, upper + 1) for x in range(lower, upper + 1)]
    # Separate worldspaces make the teleport pair the only route between A and B.
    # Both endpoints use level LAND, with arrivals clear of the door's bounds and
    # facing away from it. XTEL stores the standing transform at the OTHER end.
    center = CELL_SIZE // 2
    ground = terrain_height(center // SAMPLE_SPACING, center // SAMPLE_SPACING) * HEIGHT_UNIT
    door_route = [
        {"point": "A", "reference_id": f"{allocate():08X}", "cell": cell_name,
         "worldspace": world_name, "position": [center, center + 512, ground],
         "rotation": [0.0, 0.0, 0.0], "arrival": [center, center + 256, ground, 0.0, 0.0, math.pi]},
        {"point": "B", "reference_id": f"{allocate():08X}", "cell": destination_name,
         "worldspace": destination_name + "World", "position": [center, center - 512, ground],
         "rotation": [0.0, 0.0, math.pi], "arrival": [center, center - 256, ground, 0.0, 0.0, 0.0]},
    ]
    world_data = b""
    cells = []
    for endpoint, base_name, world_coordinates in (
            (door_route[0], cell_name, coordinates),
            (door_route[1], destination_name, [(0, 0)])):
        world_id = allocate()
        persistent_cell_id = allocate()
        name = endpoint["worldspace"]
        world = text_subrecord("EDID", name)
        world += text_subrecord("FULL", f"Navmesh test world: point {endpoint['point']}")
        world += subrecord("CNAM", struct.pack("<I", CLIMATE))
        world += subrecord("DNAM", struct.pack("<ff", 256.0, -8192.0))
        world += subrecord("ONAM", struct.pack("<4f", 1.0, 0.0, 0.0, 0.0))
        world += subrecord("DATA", bytes([0x8B]))  # Small world, no fast travel/LOD water/grass.
        minimum_x = min(x for x, y in world_coordinates) * CELL_SIZE
        minimum_y = min(y for x, y in world_coordinates) * CELL_SIZE
        maximum_x = (max(x for x, y in world_coordinates) + 1) * CELL_SIZE
        maximum_y = (max(y for x, y in world_coordinates) + 1) * CELL_SIZE
        world += subrecord("NAM0", struct.pack("<ff", minimum_x, minimum_y))
        world += subrecord("NAM9", struct.pack("<ff", maximum_x, maximum_y))
        world_record = emit_record("WRLD", world_id, world)

        # The world container uses the vanilla exterior CELL layout, including
        # placeholder grid coordinates. It holds persistent doors but no LAND;
        # reference positions associate those doors with navigable terrain cells.
        persistent = text_subrecord("EDID", name + "Persistent")
        persistent += subrecord("DATA", struct.pack("<H", 0))
        persistent += subrecord("XCLC", struct.pack("<iiI", 0, 0, 0))
        world_children = emit_record("CELL", persistent_cell_id, persistent, 0x400)
        partner = door_route[1] if endpoint["point"] == "A" else door_route[0]
        door = text_subrecord("EDID", cell_name + "DoorPoint" + endpoint["point"])
        door += subrecord("NAME", struct.pack("<I", TELEPORT_DOOR))
        door += subrecord("XTEL", struct.pack("<I6fI", int(partner["reference_id"], 16),
                                             *partner["arrival"], 0))
        door += subrecord("DATA", struct.pack("<6f", *endpoint["position"], *endpoint["rotation"]))
        door_record = emit_record("REFR", int(endpoint["reference_id"], 16), door, 0x400)
        world_children += emit_group(persistent_cell_id, 6,
                                     emit_group(persistent_cell_id, 8, door_record))
        blocks = {}
        for x, y in world_coordinates:
            cell_id = allocate()
            land_id = allocate()
            marker_id = allocate()
            editor_id = cell_editor_id(base_name, x, y)
            cell = text_subrecord("EDID", editor_id)
            cell += text_subrecord("FULL", f"Navmesh point {endpoint['point']} ({x}, {y})")
            cell += subrecord("DATA", struct.pack("<H", 0))
            cell += subrecord("XCLC", struct.pack("<iiI", x, y, 0))
            cell += subrecord("XCLW", struct.pack("<f", -8192.0))

            spawn = [x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2,
                     terrain_height(x * 32 + 16, y * 32 + 16) * HEIGHT_UNIT + 128]
            marker = subrecord("NAME", struct.pack("<I", COC_MARKER))
            marker += subrecord("DATA", struct.pack("<6f", *spawn, 0.0, 0.0, 0.0))
            temporary = emit_record("LAND", land_id, land_payload(x, y))
            temporary += emit_record("REFR", marker_id, marker)
            if x == 0 and y == 0:
                # This freestanding stone surround and its matching load door
                # share a model origin. The static frame is ordinary collision
                # geometry; the operable door remains a teleport/navigation anchor.
                surround_id = allocate()
                surround = text_subrecord("EDID", cell_name + "DoorSurround" + endpoint["point"])
                surround += subrecord("NAME", struct.pack("<I", DOOR_SURROUND))
                surround += subrecord("DATA", struct.pack("<6f", *endpoint["position"], *endpoint["rotation"]))
                temporary += emit_record("REFR", surround_id, surround)
                endpoint["surround_reference_id"] = f"{surround_id:08X}"
            children = emit_group(cell_id, 6, emit_group(cell_id, 9, temporary))
            cell_records = emit_record("CELL", cell_id, cell) + children

            # Floor division is required for negative exterior coordinates.
            block = blocks.setdefault((x // 32, y // 32), {})
            block.setdefault((x // 8, y // 8), []).append(cell_records)
            cells.append({"editor_id": editor_id, "form_id": f"{cell_id:08X}",
                          "worldspace": name, "coordinates": [x, y], "spawn": spawn,
                          "coc": f"coc {editor_id}"})

        for (block_x, block_y), subblocks in sorted(blocks.items()):
            block_data = b""
            for (subblock_x, subblock_y), entries in sorted(subblocks.items()):
                block_data += emit_group(coordinate_label(subblock_x, subblock_y), 5, b"".join(entries))
            world_children += emit_group(coordinate_label(block_x, block_y), 4, block_data)
        world_data += world_record + emit_group(world_id, 1, world_children)
    body = emit_group(int.from_bytes(b"WRLD", "little"), 0, world_data)

    # HEDR counts every following record AND group. Next Object ID is local,
    # while new on-disk FormIDs include the plugin's implicit self master slot.
    header = subrecord("HEDR", struct.pack("<fII", 1.7, record_count + group_count, next_object_id))
    header += text_subrecord("CNAM", "NavmeshGenerator synthetic fixture")
    header += text_subrecord("SNAM", "Project-authored terrain; no NAVM or NAVI records.")
    header += text_subrecord("MAST", "Skyrim.esm") + subrecord("DATA", bytes(8))
    plugin = record("TES4", 0, header) + body

    by_coordinates = {(cell["worldspace"], *cell["coordinates"]): cell for cell in cells}
    seams = []
    for cell in cells:
        x, y = cell["coordinates"]
        for dx, dy in ((1, 0), (0, 1)):
            neighbor = by_coordinates.get((cell["worldspace"], x + dx, y + dy))
            if neighbor:
                seams.append([cell["editor_id"], neighbor["editor_id"]])
    manifest = {"plugin": PLUGIN_NAME, "master": "Skyrim.esm", "worldspace": world_name,
                "coordinate_space": "skyrim-world-z-up-v1",
                "form_id_space": "file-local (self master index); runtime prefix depends on load order",
                "grid_size": size, "coc": f"coc {cell_name}", "cells": cells, "seams": seams,
                "door_route": door_route}
    return plugin, manifest


def create_test_world(output, cell_name="myTestCell", size=3):
    """Write a new fixture directory; refuse existing destinations and leave game data untouched."""
    plugin, manifest = build_plugin(cell_name, size)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / PLUGIN_NAME).write_bytes(plugin)
    (output / "fixture.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    # The developer CLI accepts physical paths in a load-order manifest. Only
    # Skyrim.esm needs resolution through its separately supplied Data directory.
    (output / "load-order.txt").write_text(f"Skyrim.esm\n{output / PLUGIN_NAME}\n", encoding="utf-8")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output/test-world"),
                        help="new output directory (default: %(default)s); existing paths are refused")
    parser.add_argument("--cell-name", default="myTestCell",
                        help="origin cell editor ID used by coc (default: %(default)s)")
    parser.add_argument("--size", type=int, default=3,
                        help="main exterior grid side, from 2 to 16; adds an isolated door destination "
                             "(default: %(default)s)")
    options = parser.parse_args(argv)
    try:
        manifest = create_test_world(options.output, options.cell_name, options.size)
    except (OSError, ValueError) as error:
        parser.exit(1, f"error: {error}\n")
    print(f"Created {options.output / PLUGIN_NAME}")
    print(f"{len(manifest['cells'])} exterior cells; {len(manifest['seams'])} shared borders; no NAVM/NAVI")
    print(f"Install and enable the ESP with Skyrim.esm, then use: {manifest['coc']}")
    print("Follower door test: use the large wooden door in the stone arch ahead of the origin spawn "
          "to travel A -> B, then return B -> A.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
