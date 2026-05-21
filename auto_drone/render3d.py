from __future__ import annotations

from auto_drone.common import FREE, OBSTACLE, UNKNOWN, blend
from auto_drone.core3d import ActiveMappingSim3D
from auto_drone.png import write_png


def render_sim3d(sim: ActiveMappingSim3D, path: str, scale: int = 10) -> None:
    projector = IsoProjector(sim.world.width, sim.world.height, sim.world.depth, scale)
    pixels = bytearray([26, 28, 32] * (projector.image_width * projector.image_height))

    paint_world_box(pixels, projector)

    path_cells = set(sim.path)
    visible_cells = sim.visible_cells
    occupied_or_uncertain = drawable_voxels(sim)

    # Back-to-front painter's order for an isometric camera looking from +x,+y,+z.
    for x, y, z in sorted(occupied_or_uncertain, key=lambda item: item[0] + item[1] + item[2]):
        color = voxel_color(sim, x, y, z)
        if (x, y, z) in visible_cells:
            color = blend(color, (40, 220, 255), 0.32)
        paint_voxel(pixels, projector, x, y, z, color, size=projector.voxel_size)

    for x, y, z in sorted(visible_cells, key=lambda item: item[0] + item[1] + item[2]):
        if (x, y, z) not in occupied_or_uncertain:
            paint_voxel(pixels, projector, x, y, z, (34, 94, 108), size=max(2, projector.voxel_size - 2))

    for x, y, z in sim.path:
        paint_voxel(pixels, projector, x, y, z, (48, 118, 255), size=projector.voxel_size + 1)

    if sim.target:
        paint_voxel(
            pixels,
            projector,
            sim.target.x,
            sim.target.y,
            sim.target.z,
            (255, 214, 52),
            size=projector.voxel_size + 3,
        )

    paint_cone_edges(pixels, projector, sim)
    if sim.estimated_pose != sim.pose:
        paint_voxel(
            pixels,
            projector,
            sim.estimated_pose.x,
            sim.estimated_pose.y,
            sim.estimated_pose.z,
            (80, 235, 120),
            size=projector.voxel_size + 2,
        )
    paint_voxel(
        pixels,
        projector,
        sim.pose.x,
        sim.pose.y,
        sim.pose.z,
        (20, 70, 245),
        size=projector.voxel_size + 5,
    )

    write_png(path, projector.image_width, projector.image_height, bytes(pixels))


class IsoProjector:
    def __init__(self, world_width: int, world_height: int, world_depth: int, scale: int):
        self.world_width = world_width
        self.world_height = world_height
        self.world_depth = world_depth
        self.scale = scale
        self.tile_w = scale
        self.tile_h = max(3, scale // 2)
        self.z_h = max(4, int(scale * 0.78))
        self.voxel_size = max(3, scale // 2)
        self.margin = scale * 5

        span_x = (world_width + world_height) * self.tile_w // 2
        span_y = (world_width + world_height) * self.tile_h // 2 + world_depth * self.z_h
        self.image_width = span_x + self.margin * 2
        self.image_height = span_y + self.margin * 2
        self.origin_x = self.margin + world_height * self.tile_w // 2
        self.origin_y = self.margin + world_depth * self.z_h

    def project(self, x: int | float, y: int | float, z: int | float) -> tuple[int, int]:
        px = self.origin_x + int((x - y) * self.tile_w / 2)
        py = self.origin_y + int((x + y) * self.tile_h / 2) - int(z * self.z_h)
        return px, py


def drawable_voxels(sim: ActiveMappingSim3D) -> set[tuple[int, int, int]]:
    voxels = set()
    for z in range(sim.world.depth):
        for y in range(sim.world.height):
            for x in range(sim.world.width):
                value = sim.belief.cell(x, y, z)
                uncertainty = sim.belief.uncertainty(x, y, z)
                if value == OBSTACLE or uncertainty >= 0.38:
                    voxels.add((x, y, z))
    return voxels


def voxel_color(sim: ActiveMappingSim3D, x: int, y: int, z: int) -> tuple[int, int, int]:
    value = sim.belief.cell(x, y, z)
    uncertainty = sim.belief.uncertainty(x, y, z)
    if value == OBSTACLE:
        return (10, 12, 16)
    if value == UNKNOWN:
        return uncertainty_color(uncertainty)
    if value == FREE:
        return blend((242, 238, 225), (245, 57, 44), uncertainty)
    return (255, 0, 255)


def uncertainty_color(uncertainty: float) -> tuple[int, int, int]:
    # Higher uncertainty renders as deeper red; lower uncertainty fades toward peach.
    low = (255, 178, 142)
    high = (145, 14, 34)
    return blend(low, high, uncertainty)


def paint_world_box(pixels: bytearray, projector: IsoProjector) -> None:
    corners = [
        (0, 0, 0),
        (projector.world_width - 1, 0, 0),
        (projector.world_width - 1, projector.world_height - 1, 0),
        (0, projector.world_height - 1, 0),
        (0, 0, projector.world_depth - 1),
        (projector.world_width - 1, 0, projector.world_depth - 1),
        (projector.world_width - 1, projector.world_height - 1, projector.world_depth - 1),
        (0, projector.world_height - 1, projector.world_depth - 1),
    ]
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    for start, end in edges:
        paint_line(pixels, projector, corners[start], corners[end], (86, 90, 100))


def paint_cone_edges(pixels: bytearray, projector: IsoProjector, sim: ActiveMappingSim3D) -> None:
    if not sim.visible_cells:
        return

    far = []
    px, py, pz = sim.pose.x, sim.pose.y, sim.pose.z
    for cell in sim.visible_cells:
        if cell == (px, py, pz):
            continue
        distance = abs(cell[0] - px) + abs(cell[1] - py) + abs(cell[2] - pz)
        far.append((distance, cell))
    for _, cell in sorted(far, reverse=True)[:14]:
        paint_line(pixels, projector, (px, py, pz), cell, (30, 180, 210))


def paint_voxel(
    pixels: bytearray,
    projector: IsoProjector,
    x: int,
    y: int,
    z: int,
    color: tuple[int, int, int],
    size: int,
) -> None:
    px, py = projector.project(x, y, z)
    half = max(1, size // 2)
    for oy in range(-half, half + 1):
        width = half - abs(oy) + 1
        for ox in range(-width, width + 1):
            set_pixel(pixels, projector.image_width, projector.image_height, px + ox, py + oy, color)


def paint_line(
    pixels: bytearray,
    projector: IsoProjector,
    start: tuple[int, int, int],
    end: tuple[int, int, int],
    color: tuple[int, int, int],
) -> None:
    x1, y1 = projector.project(*start)
    x2, y2 = projector.project(*end)
    dx = abs(x2 - x1)
    dy = -abs(y2 - y1)
    step_x = 1 if x1 < x2 else -1
    step_y = 1 if y1 < y2 else -1
    error = dx + dy
    x, y = x1, y1
    while True:
        set_pixel(pixels, projector.image_width, projector.image_height, x, y, color)
        if x == x2 and y == y2:
            return
        double_error = 2 * error
        if double_error >= dy:
            error += dy
            x += step_x
        if double_error <= dx:
            error += dx
            y += step_y


def set_pixel(
    pixels: bytearray,
    image_width: int,
    image_height: int,
    x: int,
    y: int,
    color: tuple[int, int, int],
) -> None:
    if 0 <= x < image_width and 0 <= y < image_height:
        idx = (y * image_width + x) * 3
        pixels[idx : idx + 3] = bytes(color)
