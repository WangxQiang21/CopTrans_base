"""Local triangle meshes for the blade scenes, in metres with Z up."""

from collections import defaultdict
import math

import numpy as np
import shapely as sg


COLORS = {
    "asphalt": (.12, .13, .14), "kerb": (.50, .49, .46), "paving": (.54, .52, .49),
    "white_line": (.90, .89, .83), "yellow_line": (.81, .65, .32), "steel": (.48, .51, .52),
    "rock0": (.37, .35, .32), "rock1": (.39, .37, .34), "rock2": (.42, .39, .35),
    "rock3": (.44, .41, .37), "rock_cut": (.39, .36, .32), "rock_seam": (.46, .42, .37),
    "plaster": (.78, .76, .70), "stone": (.67, .67, .63), "brick": (.63, .48, .40),
    "cream": (.82, .80, .73), "slate": (.34, .37, .40), "zinc": (.46, .48, .48),
    "tile": (.50, .34, .27), "brown_tile": (.44, .37, .33),
    "frame": (.43, .43, .40), "glass": (.30, .39, .42), "door": (.30, .29, .25),
    "grass": (.40, .49, .32), "shrub": (.30, .40, .24), "blade": (.93, .94, .94),
}


def materials(directory):
    blocks = []
    for name, rgb in COLORS.items():
        color = " ".join(str(v) for v in rgb)
        blocks.append(f"newmtl {name}\nKa {color}\nKd {color}\nKs 0.06 0.06 0.06\nNs 16")
    (directory / "site.mtl").write_text("\n\n".join(blocks) + "\n")


def polygons(geometry):
    if geometry.geom_type == "Polygon":
        if geometry.area > 1e-9:
            yield geometry
    elif hasattr(geometry, "geoms"):
        for g in geometry.geoms:
            yield from polygons(g)


class Mesh:
    def __init__(self):
        self.vertices = {}
        self.faces = defaultdict(list)

    def face(self, points, material):
        ids = []
        for point in points:
            key = tuple(round(float(v), 7) for v in point)
            ids.append(self.vertices.setdefault(key, len(self.vertices) + 1))
        for b, c in zip(ids[1:-1], ids[2:]):
            if len({ids[0], b, c}) == 3:
                self.faces[material].append((ids[0], b, c))

    def surface(self, polygon, elevation, material):
        for tri in sg.constrained_delaunay_triangles(polygon).geoms:
            ring = tri.exterior
            xy = list(ring.coords)[:3]
            if not ring.is_ccw:
                xy.reverse()
            self.face([(x, y, elevation(x, y)) for x, y in xy], material)

    def walls(self, polygon, bottom, top, material):
        oriented = sg.orient_polygons(polygon)
        for ring in (oriented.exterior, *oriented.interiors):
            coords = list(ring.coords)
            for a, b in zip(coords, coords[1:]):
                self.face([(*a, bottom(*a)), (*b, bottom(*b)), (*b, top(*b)), (*a, top(*a))], material)

    def prism(self, geometry, base, height, material):
        for p in polygons(geometry):
            self.walls(p, lambda x, y: base, lambda x, y: height, material)
            self.surface(p, lambda x, y: height, material)

    def write(self, directory, name):
        filename = f"{name}.obj"
        with (directory / filename).open("w") as dst:
            dst.write(f"mtllib site.mtl\no {name}\n")
            for x, y, z in self.vertices:
                dst.write(f"v {x:.7f} {y:.7f} {z:.7f}\n")
            coordinates = np.asarray(list(self.vertices))
            normal_index = 0
            for mat, faces in self.faces.items():
                triangles = coordinates[np.asarray(faces) - 1]
                normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
                lengths = np.linalg.norm(normals, axis=1)
                dst.write(f"usemtl {mat}\n")
                # DART and Ogre both require an explicit normal at each face corner.
                for (a, b, c), normal, length in zip(faces, normals, lengths):
                    if length == 0.:
                        continue
                    x, y, z = normal / length
                    normal_index += 1
                    dst.write(f"vn {x:.8f} {y:.8f} {z:.8f}\n"
                              f"f {a}//{normal_index} {b}//{normal_index} {c}//{normal_index}\n")
        return filename


def blade_mesh(outline, thickness):
    """One closed shell. Each span station has the prescribed chord endpoints."""
    upper = outline[:len(outline) // 2]
    lower = outline[len(upper):][::-1]
    rings = []
    for (x, hi), (_, lo) in zip(upper, lower):
        # Elliptical transverse section with the specified XY silhouette.
        rings.append([(x, (hi + lo) / 2 + (hi - lo) / 2 * math.cos(a),
                       thickness / 2 * (1 + math.sin(a))) for a in np.arange(16) * (math.tau / 16)])
    result = Mesh()
    for left, right in zip(rings, rings[1:]):
        for j in range(len(left)):
            k = (j + 1) % len(left)
            result.face([left[j], left[k], right[k], right[j]], "blade")
    result.face(rings[0][::-1], "blade")
    result.face(rings[-1], "blade")
    return result
