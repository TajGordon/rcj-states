import math
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Pose:
    x_mm: float
    y_mm: float
    theta_rad: float  # 0..2π


@dataclass
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float


class FieldMap:
    """Holds walls/obstacles as line segments for raycasting."""
    
    def __init__(self):
        self.segments: List[Segment] = []
    
    def add_rectangle(self, x_min: float, y_min: float, x_max: float, y_max: float):
        """Add axis-aligned rectangle as 4 segments."""
        self.segments.extend([
            Segment(x_min, y_min, x_max, y_min),  # top
            Segment(x_max, y_min, x_max, y_max),  # right
            Segment(x_max, y_max, x_min, y_max),  # bottom
            Segment(x_min, y_max, x_min, y_min),  # left
        ])
    
    def raycast(self, x0: float, y0: float, theta: float, max_range_mm: float = 5000.0) -> Optional[float]:
        """Return distance to nearest intersection along ray from (x0,y0) at angle theta."""
        dx = math.cos(theta)
        dy = math.sin(theta)
        best_t = None
        
        for seg in self.segments:
            hit_t = self._ray_segment_intersection(x0, y0, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2)
            if hit_t is None or hit_t <= 0:
                continue
            dist = hit_t
            if dist > max_range_mm:
                continue
            if best_t is None or dist < best_t:
                best_t = dist
        
        return best_t
    
    def _ray_segment_intersection(self, x0: float, y0: float, rdx: float, rdy: float,
                                  x1: float, y1: float, x2: float, y2: float) -> Optional[float]:
        """Ray vs segment intersection. Returns t along ray (distance) or None."""
        sx = x2 - x1
        sy = y2 - y1
        denom = (-rdx * sy + rdy * sx)
        if abs(denom) < 1e-9:
            return None  # parallel
        s = (-rdy * (x1 - x0) + rdx * (y1 - y0)) / denom
        t = ( sx * (y1 - y0) - sy * (x1 - x0)) / denom
        if s < 0.0 or s > 1.0 or t < 0.0:
            return None
        return t


def build_field() -> FieldMap:
    """Build the field geometry with origin at center (0,0)."""
    # Field dimensions
    W = 2430.0  # field width
    H = 1820.0  # field height
    hx = W / 2.0
    hy = H / 2.0
    
    # Line and margin dimensions
    line_thickness = 50.0  # white line thickness
    margin = 250.0  # no-go margin from walls
    
    # Goal dimensions
    goal_width = 450.0  # internal width
    goal_depth = 74.0   # depth
    
    field = FieldMap()
    
    # Outer walls (10mm thick)
    wall_t = 10.0
    field.add_rectangle(-hx - wall_t, -hy - wall_t,  hx + wall_t, -hy)      # top
    field.add_rectangle(-hx - wall_t, -hy,         -hx,        hy)         # left
    field.add_rectangle(-hx - wall_t,  hy,          hx + wall_t, hy + wall_t) # bottom
    field.add_rectangle( hx,        -hy,           hx + wall_t, hy)         # right
    
    # Goals - front edge at internal white line, centered vertically
    goal_half_width = goal_width / 2.0
    left_goal_x = -hx + line_thickness  # internal edge of left white line
    right_goal_x = hx - line_thickness  # internal edge of right white line
    
    # Left goal
    field.add_rectangle(left_goal_x, -goal_half_width, left_goal_x + goal_depth, -goal_half_width)  # top
    field.add_rectangle(left_goal_x + goal_depth, -goal_half_width, left_goal_x + goal_depth, goal_half_width)  # right
    field.add_rectangle(left_goal_x, goal_half_width, left_goal_x + goal_depth, goal_half_width)  # bottom
    field.add_rectangle(left_goal_x, -goal_half_width, left_goal_x, goal_half_width)  # left
    
    # Right goal
    field.add_rectangle(right_goal_x - goal_depth, -goal_half_width, right_goal_x, -goal_half_width)  # top
    field.add_rectangle(right_goal_x - goal_depth, -goal_half_width, right_goal_x - goal_depth, goal_half_width)  # left
    field.add_rectangle(right_goal_x - goal_depth, goal_half_width, right_goal_x, goal_half_width)  # bottom
    field.add_rectangle(right_goal_x, -goal_half_width, right_goal_x, goal_half_width)  # right
    
    return field


def compute_error_at_pose(pose: Pose, sensor_pairs: List[Tuple[float, int]], field: FieldMap) -> float:
    """Compute error between measured ToF distances and expected raycast distances."""
    error = 0.0
    for angle_rad, measured_mm in sensor_pairs:
        # World angle = robot heading + sensor angle
        world_angle = (pose.theta_rad + angle_rad) % (2.0 * math.pi)
        expected = field.raycast(pose.x_mm, pose.y_mm, world_angle, max_range_mm=5000.0)
        
        if expected is None:
            # No hit: penalize if sensor saw something
            if measured_mm > 0:
                error += 1000.0
            continue
        
        residual = expected - float(measured_mm)
        # Huber-like robust loss
        k = 50.0
        if abs(residual) <= k:
            error += 0.5 * residual * residual
        else:
            error += k * (abs(residual) - 0.5 * k)
    
    return error


def is_pose_valid(x_mm: float, y_mm: float) -> bool:
    """Check if pose is in valid playable area (not in margins or goal boxes)."""
    W = 2430.0
    H = 1820.0
    hx = W / 2.0
    hy = H / 2.0
    margin = 250.0
    line_thickness = 50.0
    goal_width = 450.0
    goal_depth = 74.0
    
    # Check margin constraints
    if abs(x_mm) > hx - margin or abs(y_mm) > hy - margin:
        return False
    
    # Check goal box constraints
    goal_half_width = goal_width / 2.0
    left_goal_x = -hx + line_thickness
    right_goal_x = hx - line_thickness
    
    # Left goal box
    if (left_goal_x <= x_mm <= left_goal_x + goal_depth and 
        -goal_half_width <= y_mm <= goal_half_width):
        return False
    
    # Right goal box
    if (right_goal_x - goal_depth <= x_mm <= right_goal_x and 
        -goal_half_width <= y_mm <= goal_half_width):
        return False
    
    return True


def refine_pose(initial_pose: Pose, sensor_pairs: List[Tuple[float, int]], field: FieldMap) -> Pose:
    """Refine pose using local search around initial guess."""
    best_pose = initial_pose
    best_error = compute_error_at_pose(best_pose, sensor_pairs, field)
    
    # Coarse-to-fine search
    step_sizes = [100.0, 25.0, 5.0]
    
    for step in step_sizes:
        improved = True
        while improved:
            improved = False
            # 8-neighborhood search
            for dx, dy in [(step, 0), (-step, 0), (0, step), (0, -step),
                          (step, step), (step, -step), (-step, step), (-step, -step)]:
                candidate_x = best_pose.x_mm + dx
                candidate_y = best_pose.y_mm + dy
                
                # Skip invalid poses
                if not is_pose_valid(candidate_x, candidate_y):
                    continue
                
                candidate = Pose(
                    candidate_x,
                    candidate_y,
                    best_pose.theta_rad  # keep heading from IMU
                )
                error = compute_error_at_pose(candidate, sensor_pairs, field)
                if error < best_error:
                    best_pose = candidate
                    best_error = error
                    improved = True
    
    return best_pose


class Localizer:
    """Localization system that maintains best pose estimate and refines it with sensor data."""

    def __init__(self, initial_pose_estimate: Optional[Pose] = None):
        self.best_pose: Pose = initial_pose_estimate or Pose(0.0, 0.0, 0.0)
        self.last_sensor_pairs: List[Tuple[float, int]] = []
        self.last_heading_rad: float = self.best_pose.theta_rad
        self.field = build_field()

    def update_inputs(self, tof_pairs: List[Tuple[float, int]], heading_rad: float) -> None:
        """Update latest ToF pairs and IMU heading (radians)."""
        self.last_sensor_pairs = tof_pairs or []
        self.last_heading_rad = heading_rad

    def best_estimate(self) -> Pose:
        """Get current best pose estimate."""
        return self.best_pose

    def estimate_position(self) -> Pose:
        """Update best pose estimate using latest sensor data."""
        if not self.last_sensor_pairs:
            return self.best_pose
        
        # Start from previous best, update heading from IMU
        initial = Pose(self.best_pose.x_mm, self.best_pose.y_mm, self.last_heading_rad)
        
        # Refine position
        self.best_pose = refine_pose(initial, self.last_sensor_pairs, self.field)
        
        return self.best_pose