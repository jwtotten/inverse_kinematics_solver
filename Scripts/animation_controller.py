# animation_controller.py — pure animation logic functions


def advance_frame(frame_counter: float, speed: float, num_samples: int) -> float:
    """Advance frame_counter by speed, wrap at num_samples."""
    return (frame_counter + speed) % num_samples


def compute_body_offset(current_offset: float, direction: int, step_length: float, num_samples: int) -> float:
    """Add direction * (step_length / num_samples) to current_offset."""
    return current_offset + direction * (step_length / num_samples)


def clamp_speed(speed: float) -> float:
    return max(0.2, min(3.0, speed))


def translate_joints(joints: list, body_offset: float) -> list:
    """Add body_offset to x-coordinate (index 0) of each joint."""
    return [[j[0] + body_offset, j[1], j[2]] for j in joints]


def build_ground_grid(body_offset: float, spacing: float = 1.0, half_width: float = 12.0) -> tuple:
    """
    Returns (x_vals, y_vals, z_vals) for ground grid lines at z=0,
    centered on body_offset with given half_width and line spacing.
    Each of x_vals, y_vals, z_vals is a list of segments; each segment
    is a list [a, b, None] to allow a single ax.plot() call.
    """
    xs, ys, zs = [], [], []
    x_min = body_offset - half_width
    x_max = body_offset + half_width
    y_min = -half_width
    y_max = half_width

    # Lines parallel to Y axis (varying X)
    x = x_min
    while x <= x_max + 1e-9:
        xs.append([x, x, None])
        ys.append([y_min, y_max, None])
        zs.append([0.0, 0.0, None])
        x += spacing

    # Lines parallel to X axis (varying Y)
    y = y_min
    while y <= y_max + 1e-9:
        xs.append([x_min, x_max, None])
        ys.append([y, y, None])
        zs.append([0.0, 0.0, None])
        y += spacing

    return xs, ys, zs


def apply_direction_to_controllers(controllers: list, direction: int, step_length: float) -> None:
    raise NotImplementedError


def apply_gait_to_controllers(controllers: list, gait_pattern) -> None:
    raise NotImplementedError
