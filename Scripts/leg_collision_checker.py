import itertools
from dataclasses import dataclass
from typing import List
from Scripts.geometry_utils import build_segment_list, segment_to_segment_distance

_SEGMENT_NAMES = ["femur", "tibia"]


@dataclass
class CollisionResult:
    leg_a_index: int
    leg_b_index: int
    segment_a_name: str
    segment_b_name: str
    distance: float
    is_crossing: bool


def check_all_leg_collisions(
    all_leg_coordinates: List[list],
    threshold: float = 0.05,
) -> List[CollisionResult]:
    """
    Check every pair of segments from different legs for crossing.

    :param all_leg_coordinates: List of coordinate triples, one per leg.
        Each entry is the output of solve_leg_position_from_target_coordinates:
        [[hip_x, hip_y, hip_z], [knee_x, knee_y, knee_z], [foot_x, foot_y, foot_z]]
    :param threshold: Distance below which two segments are considered crossing.
    :return: List of CollisionResult for every crossing pair found.
    """
    results = []
    leg_segments = [build_segment_list(coords) for coords in all_leg_coordinates]

    for leg_a, leg_b in itertools.combinations(range(len(leg_segments)), 2):
        for seg_a_idx, seg_a_name in enumerate(_SEGMENT_NAMES):
            for seg_b_idx, seg_b_name in enumerate(_SEGMENT_NAMES):
                p0, p1 = leg_segments[leg_a][seg_a_idx]
                q0, q1 = leg_segments[leg_b][seg_b_idx]
                dist = segment_to_segment_distance(p0, p1, q0, q1)
                is_crossing = dist < threshold
                if is_crossing:
                    results.append(CollisionResult(
                        leg_a_index=leg_a,
                        leg_b_index=leg_b,
                        segment_a_name=seg_a_name,
                        segment_b_name=seg_b_name,
                        distance=dist,
                        is_crossing=True,
                    ))

    return results


def get_crossing_leg_indices(
    all_leg_coordinates: List[list],
    threshold: float = 0.05,
) -> List[int]:
    """
    Return the flat list of 0-based leg indices involved in any crossing.
    Used by the plotter to colour affected legs red.
    """
    results = check_all_leg_collisions(all_leg_coordinates, threshold)
    crossing = set()
    for r in results:
        crossing.add(r.leg_a_index)
        crossing.add(r.leg_b_index)
    return list(crossing)
