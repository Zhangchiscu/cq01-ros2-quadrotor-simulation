from typing import Dict, Iterable, Optional, Tuple

Obstacle = Dict[str, float]


def expanded_bounds(
    obstacle: Obstacle,
    margin: float,
) -> Tuple[float, float, float, float]:
    half_x = 0.5 * obstacle["size_x"] + margin
    half_y = 0.5 * obstacle["size_y"] + margin
    return (
        obstacle["center_x"] - half_x,
        obstacle["center_x"] + half_x,
        obstacle["center_y"] - half_y,
        obstacle["center_y"] + half_y,
    )


def segment_intersects_aabb_2d(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    obstacle: Obstacle,
    margin: float = 0.0,
) -> bool:
    xmin, xmax, ymin, ymax = expanded_bounds(
        obstacle,
        margin,
    )
    dx = end_x - start_x
    dy = end_y - start_y
    p = (-dx, dx, -dy, dy)
    q = (
        start_x - xmin,
        xmax - start_x,
        start_y - ymin,
        ymax - start_y,
    )

    lower = 0.0
    upper = 1.0

    for p_value, q_value in zip(p, q):
        if abs(p_value) < 1.0e-12:
            if q_value < 0.0:
                return False
            continue

        ratio = q_value / p_value

        if p_value < 0.0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)

        if lower > upper:
            return False

    return True


def first_segment_collision(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    obstacles: Iterable[Obstacle],
    margin: float = 0.0,
) -> Optional[Obstacle]:
    for obstacle in obstacles:
        if segment_intersects_aabb_2d(
            start_x,
            start_y,
            end_x,
            end_y,
            obstacle,
            margin,
        ):
            return obstacle
    return None
