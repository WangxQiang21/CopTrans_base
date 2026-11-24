"""Generate the display meshes from the pinned AgileX COLLADA files."""

from copy import deepcopy
from pathlib import Path
import sys
from xml.etree import ElementTree as ET


NS = "http://www.collada.org/2005/11/COLLADASchema"
XML = {"": NS}

# Geometry IDs in the original meshes listed in SOURCE.md.
FRONT_PANEL = "mesh-a18d26fe-f430-4af9-88eb-e7baf734ec24"
BODY_PANELS = {
    FRONT_PANEL,
    "mesh-df3a69b2-1c9e-4d88-8205-3979f532acd2",
    "mesh-c0b9b926-1ef6-4ddc-b300-9f257599ac60",
}
BASE_MATERIALS = {
    "mesh-aef55ad2-8375-4de3-87a3-4a093230d536": "graphite",  # Top plate
    "mesh-a8c1ed52-b75d-424b-b031-bfbe6877739b": "silver",  # Rails
    "mesh-af0b1f61-f522-4e1d-b8c0-06470e249676": "silver",
    "mesh-a38532bb-fa02-4141-b052-dcba710083e4": "silver",  # Front inlays
    "mesh-d47819b3-b5d5-4b1a-9976-0d021be1711d": "silver",
}
# Wheel geometry, spring geometry, and sign of the outward mesh Z axis.
WHEELS = {
    "fl.dae": ("mesh-a100e2d3-b575-42f0-91fb-93fd9a990e53",
               "mesh-a92d25fc-4376-48c5-b9b4-f294691ffcc2", 1),
    "fr.dae": ("mesh-a389d41f-a61f-4eec-9b95-ce4146ae3326",
               "mesh-bb67cb29-9073-413e-a67e-ba8f4f6613de", -1),
}

SHELL_COLORS = {
    "white": "0.94 0.96 0.99",
    "orange": "1.00 0.27 0.018",
    "blue": "0.055 0.24 0.75",
}
# Diffuse RGB, intensity of specular reflection, and shininess.
MATERIALS = {
    "chassis": ("0.085 0.085 0.090", "0.13", "24"),
    "graphite": ("0.12 0.125 0.135", "0.23", "36"),
    "silver": ("0.65 0.67 0.70", "0.52", "55"),
    "rubber": ("0.028 0.029 0.032", "0.035", "10"),
}


def add_material(root, name, rgb, specular, shininess):
    materials = root.find("library_materials", XML)
    effects = root.find("library_effects", XML)
    material = ET.SubElement(materials, f"{{{NS}}}material", id=name, name=name)
    ET.SubElement(material, f"{{{NS}}}instance_effect", url=f"#effect-{name}")
    effect = ET.SubElement(effects, f"{{{NS}}}effect", id=f"effect-{name}")
    profile = ET.SubElement(effect, f"{{{NS}}}profile_COMMON")
    technique = ET.SubElement(profile, f"{{{NS}}}technique", sid="common")
    phong = ET.SubElement(technique, f"{{{NS}}}phong")
    colors = {"ambient": rgb, "diffuse": rgb, "specular": " ".join([specular] * 3)}
    for key, value in colors.items():
        element = ET.SubElement(phong, f"{{{NS}}}{key}")
        ET.SubElement(element, f"{{{NS}}}color").text = f"{value} 1"
    element = ET.SubElement(phong, f"{{{NS}}}shininess")
    ET.SubElement(element, f"{{{NS}}}float").text = shininess
    return phong


def scene_bindings(root):
    scene = root.find("library_visual_scenes/visual_scene", XML)
    nodes = {node.get("url")[1:]: node for node in scene.findall("node/instance_geometry", XML)}
    return {geom_id: node.find("bind_material/technique_common", XML)
            for geom_id, node in nodes.items()}


def bind_material(binding, symbol, material):
    ET.SubElement(binding, f"{{{NS}}}instance_material", symbol=symbol, target=f"#{material}")


def split_triangles(geometry, binding, surface, accent, on_accent):
    """Assign two materials to existing faces without changing vertex data."""
    mesh = geometry.find("mesh", XML)
    triangles = mesh.find("triangles", XML)
    position = mesh.find("source[@name='position']/float_array", XML)
    coords = list(map(float, position.text.split()))
    vertices = list(zip(coords[::3], coords[1::3], coords[2::3]))
    inputs = triangles.findall("input", XML)
    stride = 1 + max(int(item.get("offset")) for item in inputs)
    vertex_offset = int(triangles.find("input[@semantic='VERTEX']", XML).get("offset"))
    indices = triangles.find("p", XML).text.split()
    face_size = stride * 3
    batches = {surface: [], accent: []}
    for start in range(0, len(indices), face_size):
        face = indices[start:start + face_size]
        corners = [vertices[int(index)] for index in face[vertex_offset::stride]]
        batches[accent if on_accent(corners) else surface].extend(face)

    binding.remove(binding.find("instance_material", XML))
    mesh.remove(triangles)
    for material, batch in batches.items():
        if not batch:
            raise ValueError(f"{geometry.get('id')} has no {material} faces")
        section = ET.SubElement(mesh, f"{{{NS}}}triangles", material=material,
                                count=str(len(batch) // face_size))
        section.extend(deepcopy(inputs))
        ET.SubElement(section, f"{{{NS}}}p").text = " ".join(batch)
        bind_material(binding, material, material)


def is_body_trim(corners):
    # Raw mesh coordinates in millimeters. X points rearward and Y points up.
    front_strip = all(
        x < -359.99 and -57.6 < y < -22.4 and abs(z) < 169.6 for x, y, z in corners
    )
    side_badge = all(
        -292.1 < x < -151.9 and -66.1 < y < -15.9 and abs(z) > 250.1 for x, y, z in corners
    )
    return front_strip or side_badge


def select_parts(root, geometry_ids, *, keep):
    library = root.find("library_geometries", XML)
    scene = root.find("library_visual_scenes/visual_scene", XML)
    for geometry in list(library):
        if (geometry.get("id") in geometry_ids) != keep:
            library.remove(geometry)
    for node in list(scene):
        instance = node.find("instance_geometry", XML)
        if instance is not None and (instance.get("url")[1:] in geometry_ids) != keep:
            scene.remove(node)


def paint_body(output, base):
    root = ET.parse(base).getroot()
    missing = (BODY_PANELS | BASE_MATERIALS.keys()) - scene_bindings(root).keys()
    if missing:
        raise ValueError(f"{base.name} is missing parts: {sorted(missing)}")

    for material in ("chassis", "graphite", "silver"):
        add_material(root, material, *MATERIALS[material])
    shell = deepcopy(root)
    select_parts(root, BODY_PANELS, keep=False)
    select_parts(shell, BODY_PANELS, keep=True)

    for geom_id, binding in scene_bindings(root).items():
        binding.find("instance_material", XML).set(
            "target", f"#{BASE_MATERIALS.get(geom_id, 'chassis')}")
    write_mesh(root, output / base.name)

    bindings = scene_bindings(shell)
    for geom_id in BODY_PANELS - {FRONT_PANEL}:
        bindings[geom_id].find("instance_material", XML).set("target", "#shell")
    geometry = shell.find(f"library_geometries/geometry[@id='{FRONT_PANEL}']", XML)
    split_triangles(geometry, bindings[FRONT_PANEL], "shell", "chassis", is_body_trim)
    paint = add_material(shell, "shell", SHELL_COLORS["white"], "0.30", "40")
    for color, rgb in SHELL_COLORS.items():
        for channel in ("ambient", "diffuse"):
            paint.find(f"{channel}/color", XML).text = f"{rgb} 1"
        write_mesh(shell, output / f"shell_{color}.dae")


def paint_wheel(output, wheel):
    root = ET.parse(wheel).getroot()
    for material in ("chassis", "silver", "rubber"):
        add_material(root, material, *MATERIALS[material])
    wheel_id, spring_id, outward = WHEELS[wheel.name]
    bindings = scene_bindings(root)
    for geom_id, binding in bindings.items():
        material = "silver" if geom_id == spring_id else "chassis"
        binding.find("instance_material", XML).set("target", f"#{material}")

    def is_hub(corners):
        # Both wheel files share the same hub center in their raw XY plane.
        return all(
            (x + 245.0)**2 + (y + 220.8)**2 <= 68.0**2 and z * outward > 205.0
            for x, y, z in corners
        )

    geometry = root.find(f"library_geometries/geometry[@id='{wheel_id}']", XML)
    split_triangles(geometry, bindings[wheel_id], "rubber", "silver", is_hub)
    write_mesh(root, output / wheel.name)


def write_mesh(root, path):
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def main():
    output, base, left, right = map(Path, sys.argv[1:])
    ET.register_namespace("", NS)
    output.mkdir(parents=True, exist_ok=True)
    paint_body(output, base)
    paint_wheel(output, left)
    paint_wheel(output, right)


if __name__ == "__main__":
    main()
