import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

from IMU import IMU
from tof_stuff import ToFArray


@dataclass
class Pose:
    x_mm: float
    y_mm: float
    theta_rad: float  # 0..2π, robot heading (0 = field north)


@dataclass
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float


class FieldMap:
    """Holds walls/obstacles as line segments and supports raycasting.

    You can add rectangles (axis-aligned) and arbitrary polygons.
    Units: millimeters.
    """

    def __init__(self):
        self.segments: List[Segment] = []

    def add_rectangle(self, x_min: float, y_min: float, x_max: float, y_max: float):
        # axis-aligned rectangle as 4 segments (clockwise)
        self.segments.extend([
            Segment(x_min, y_min, x_max, y_min),
            Segment(x_max, y_min, x_max, y_max),
            Segment(x_max, y_max, x_min, y_max),
            Segment(x_min, y_max, x_min, y_min),
        ])
    
    def add_rectangles_bulk(self, rects: List[Tuple[float, float, float, float]]):
        for (x_min, y_min, x_max, y_max) in rects:
            self.add_rectangle(x_min, y_min, x_max, y_max)

    def add_polygon(self, points: List[Tuple[float, float]]):
        if len(points) < 2:
            return
        for i in range(len(points)):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % len(points)]
            self.segments.append(Segment(x1, y1, x2, y2))

    def raycast(self, x0: float, y0: float, theta: float, max_range_mm: float = 5000.0) -> Optional[float]:
        """Return distance to nearest intersection along ray from (x0,y0) at angle theta.

        If no hit within max_range_mm, returns None.
        """
        dx = math.cos(theta)
        dy = math.sin(theta)
        best_t = None

        for seg in self.segments:
            hit_t = _ray_segment_intersection(x0, y0, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2)
            if hit_t is None:
                continue
            if hit_t <= 0:
                continue
            dist = hit_t  # because ray direction is unit-length
            if dist > max_range_mm:
                continue
            if best_t is None or dist < best_t:
                best_t = dist

        return best_t


def _ray_segment_intersection(x0: float, y0: float, rdx: float, rdy: float,
                              x1: float, y1: float, x2: float, y2: float) -> Optional[float]:
    """Ray (origin x0,y0; direction rdx,rdy) vs segment ((x1,y1)->(x2,y2)).

    Returns t along the ray (distance in mm because rdx,rdy is unit-length), or None if no hit.
    Based on solving: p = (x0,y0) + t*(rdx,rdy) = (x1,y1) + u*(sx,sy), with 0<=u<=1, t>=0.
    """
    sx = x2 - x1
    sy = y2 - y1
    denom = (-rdx * sy + rdy * sx)
    if abs(denom) < 1e-9:
        return None  # parallel or nearly parallel
    s = (-rdy * (x1 - x0) + rdx * (y1 - y0)) / denom
    t = ( sx * (y1 - y0) - sy * (x1 - x0)) / denom
    if s < 0.0 or s > 1.0 or t < 0.0:
        return None
    # rdx,rdy are unit direction; t is already the distance
    return t


def load_rust_field_geometry(field: FieldMap):
    """Populate FieldMap with the same geometry used in tof_rust_thingy_from_prev.rs.

    Each "Wall" in Rust was drawn as an axis-aligned rectangle with corners (a) and (b).
    We replicate them here as rectangles.
    Units are millimeters.
    """
    rects = [
        # walls
        (-10.0, -10.0, 2440.0, 0.0),
        (-10.0, 0.0, 0.0, 1820.0),
        (-10.0, 1820.0, 2440.0, 1830.0),
        (2430.0, 0.0, 2440.0, 1820.0),
        # goal left
        (0.0, 600.0, 140.0, 610.0),
        (56.0, 610.0, 66.0, 1210.0),
        (0.0, 1210.0, 140.0, 1220.0),
        # goal right
        (2430.0 - 140.0, 600.0, 2430.0, 610.0),
        (2430.0 - 66.0, 610.0, 2430.0 - 56.0, 1210.0),
        (2430.0 - 140.0, 1210.0, 2430.0, 1220.0),
    ]
    field.add_rectangles_bulk(rects)


def build_centered_field_with_margins() -> FieldMap:
    """Build a field with origin at center (0,0), outer walls at ±W/±H, and 250mm no-go margins.

    Field size: 2430 x 1820 mm. Origin (0,0) at center.
    Adds:
      - Outer boundary walls (thin rectangles as segments)
      - 250mm margin bands along each wall as obstacles
      - Goals as in the Rust geometry, shifted to center-origin
    """
    W = 2430.0
    H = 1820.0
    hx = W / 2.0
    hy = H / 2.0
    margin = 250.0
    line_thickness = 50.0
    goal_internal_width = 450.0
    goal_internal_depth = 74.0
    goal_box_width = 300.0
    goal_box_height = 900.0
    distance_from_goal_start_to_middle = 915.0

    f = FieldMap()

    # Outer walls (thin rectangles of 10mm thickness)
    wall_t = 10.0
    f.add_rectangles_bulk([
        (-hx - 10.0, -hy - 10.0,  hx + 10.0, -hy),      # top
        (-hx - 10.0, -hy,         -hx,        hy),       # left
        (-hx - 10.0,  hy,          hx + 10.0, hy + 10.0),# bottom
        ( hx,        -hy,           hx + 10.0, hy),      # right
    ])

    # Margin bands (250mm inside each wall)
    f.add_rectangles_bulk([
        (-hx, -hy,  hx, -hy + margin),             # top margin band
        (-hx,  hy - margin,  hx,  hy),             # bottom margin band
        (-hx, -hy, -hx + margin,  hy),             # left margin band
        ( hx - margin, -hy,  hx,  hy),             # right margin band
    ])

    # Goals: copy from Rust and shift by (-hx, -hy)
    rects_rust = [
        # goal left
        (0.0, 600.0, 140.0, 610.0),
        (56.0, 610.0, 66.0, 1210.0),
        (0.0, 1210.0, 140.0, 1220.0),
        # goal right
        (2430.0 - 140.0, 600.0, 2430.0, 610.0),
        (2430.0 - 66.0, 610.0, 2430.0 - 56.0, 1210.0),
        (2430.0 - 140.0, 1210.0, 2430.0, 1220.0),
    ]
    shifted = [(x1 - hx, y1 - hy, x2 - hx, y2 - hy) for (x1, y1, x2, y2) in rects_rust]
    f.add_rectangles_bulk(shifted)

    return f



class LocalizationManager:
    def __init__(self, i2c, field_map: FieldMap):
        self.imu = IMU(i2c)
        self.tof_array = ToFArray(i2c)
        self.field = field_map
        # Call imu.start() if you prefer a cached heading; otherwise read on-demand
        # self.imu.start()

    def get_sensor_pairs(self, fresh: bool = False) -> List[Tuple[float, int]]:
        """Return [(sensor_angle_rad, distance_from_center_mm), ...]"""
        return self.tof_array.get_localization_pairs(fresh=fresh)

    def compute_expected_distance(self, pose: Pose, sensor_angle_rad: float, max_range_mm: float = 5000.0) -> Optional[float]:
        """Raycast from robot pose along sensor world angle and return expected range."""
        theta_world = (pose.theta_rad + sensor_angle_rad) % (2.0 * math.pi)
        # Ray direction for raycast(x0,y0,theta) uses cos,sin with theta measured from +x axis.
        # Our convention: theta_rad is 0 = field north. If your map uses +x to the right and +y up,
        # and 0 radians is +x, then convert here. Example assumes 0 rad = +x.
        return self.field.raycast(pose.x_mm, pose.y_mm, theta_world, max_range_mm=max_range_mm)

    def error_for_pose(self, pose: Pose, pairs: List[Tuple[float, int]], max_range_mm: float = 5000.0) -> float:
        """Compute error between measured distances and map-expected distances via raycasting.

        Uses a robust loss to reduce impact of outliers.
        """
        error = 0.0
        for angle_rad, measured_mm in pairs:
            expected = self.compute_expected_distance(pose, angle_rad, max_range_mm=max_range_mm)
            if expected is None:
                # no hit: penalize if sensor saw something, otherwise small cost
                if measured_mm > 0:
                    error += 1000.0
                continue
            residual = (expected - float(measured_mm))
            # Huber-like loss
            k = 50.0
            if abs(residual) <= k:
                error += 0.5 * residual * residual
            else:
                error += k * (abs(residual) - 0.5 * k)
        return error

    def estimate_pose(self, initial: Pose, search_radius_mm: float = 150.0, angle_span_rad: float = math.radians(10.0),
                      grid_step_mm: float = 25.0, angle_step_rad: float = math.radians(2.0)) -> Pose:
        """Crude grid search around an initial pose to minimize raycast error.

        Replace/extend with your iterative algorithm (Gauss-Newton, particle filter, etc.).
        """
        pairs = self.get_sensor_pairs(fresh=False)
        best_pose = initial
        best_err = self.error_for_pose(best_pose, pairs)

        # Grid search in a neighborhood
        x_range = _frange(initial.x_mm - search_radius_mm, initial.x_mm + search_radius_mm, grid_step_mm)
        y_range = _frange(initial.y_mm - search_radius_mm, initial.y_mm + search_radius_mm, grid_step_mm)
        a_range = _frange(initial.theta_rad - angle_span_rad, initial.theta_rad + angle_span_rad, angle_step_rad)

        for x in x_range:
            for y in y_range:
                for a in a_range:
                    pose = Pose(x, y, _wrap_angle(a))
                    err = self.error_for_pose(pose, pairs)
                    if err < best_err:
                        best_err = err
                        best_pose = pose

        # TODO: Add local refinement here (e.g., smaller steps around best_pose)
        return best_pose

    def estimate_pose_using_imu(self, x_mm: float, y_mm: float) -> Pose:
        """Convenience: use IMU heading as theta, then refine x,y by searching only position."""
        theta = self.imu.read_heading_rad()
        initial = Pose(x_mm, y_mm, theta)
        return self.estimate_pose(initial, angle_span_rad=0.0)  # lock angle


def _wrap_angle(a: float) -> float:
    return (a + 2.0 * math.pi) % (2.0 * math.pi)


def _frange(start: float, stop: float, step: float) -> List[float]:
    values = []
    v = start
    if step <= 0:
        return [start]
    while v <= stop + 1e-9:
        values.append(v)
        v += step
    return values


# =============================
# Notes / scratch space for your algorithm
#
# 1) Raycasting refresher (axis-aligned and arbitrary polygons):
#    - Represent walls/obstacles as line segments.
#    - A sensor ray is (x0,y0) + t*(cos(theta), sin(theta)), t>=0.
#    - For each segment, solve intersection; keep the smallest positive t.
#    - Axis-aligned rectangles are just 4 segments; arbitrary fields can be polygons.
#
# 2) Coordinate frames:
#    - Pose.theta_rad is your robot heading; sensor angles are relative to robot’s forward.
#    - World ray angle = pose.theta_rad + sensor_angle_rad (wrap to 0..2π).
#    - Ensure your field map uses the same XY orientation/units as your localization (mm).
#
# 3) Error model:
#    - Compare measured range vs. expected range from raycast.
#    - Use robust loss (Huber) to reduce influence of outliers/bad sensors.
#
# 4) Solver strategy (fill in as you like):
#    - Start with IMU heading; grid search x,y nearby; then local refine.
#    - Or particle filter: sample poses, weight by exp(-error/σ), resample.
#    - Or gradient-based if you derive Jacobians for ray distances.
#
# 5) Non-rectangular fields:
#    - Use add_polygon to approximate curved or angled boundaries with segments.
#    - Add interior obstacles as more polygons/segments.
#

from IMU import IMU
from tof_stuff import ToFArray

class LocalizationManager:
    def __init__(self, i2c): # do i need stuff here
        self.IMU = IMU(i2c)
        self.ToFArray = ToFArray(i2c)