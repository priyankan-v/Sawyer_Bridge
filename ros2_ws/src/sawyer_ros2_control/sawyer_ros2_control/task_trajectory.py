"""Conservative retiming of planned joint polylines; every segment stops smoothly."""
import math

from .safety import JOINTS, Quintic, finite, validate_positions


def waypoints(trajectory):
    message = trajectory['joint_trajectory']
    names = message['joint_names']
    if len(names) != len(set(names)) or set(names) != set(JOINTS):
        raise ValueError('Trajectory must contain exactly the seven arm joints')
    result = []
    for point in message['points']:
        if len(point['positions']) != len(names):
            raise ValueError('Malformed trajectory positions')
        positions = {name: finite(value) for name, value in zip(names, point['positions'])}
        if not result or max(abs(positions[name] - result[-1][name]) for name in JOINTS) > 1e-10:
            result.append(positions)
    if not result:
        raise ValueError('Empty arm trajectory')
    return result


def retime(points, limits):
    for point in points:
        validate_positions(point, limits)
    segments = []
    for start, target in zip(points, points[1:]):
        duration = max(.05, max(max(1.875 * abs(target[j] - start[j]) / limits[j].velocity,
            math.sqrt((10 / math.sqrt(3)) * abs(target[j] - start[j]) / limits[j].acceleration)) for j in JOINTS)) * 1.05
        segment = Quintic(start, target, duration, limits, max_displacement=.2)
        segment.target = dict(target)
        segments.append(segment)
    return segments


def sample(segments, elapsed):
    if not segments:
        raise ValueError('Cannot sample an empty trajectory')
    for segment in segments:
        if elapsed <= segment.duration:
            if elapsed <= 0:
                return dict(segment.start)
            if elapsed >= segment.duration:
                return dict(segment.target)
            position = segment.sample(elapsed)[0]
            return {j: min(max(segment.start[j], segment.target[j]),
                           max(min(segment.start[j], segment.target[j]), position[j])) for j in JOINTS}
        elapsed -= segment.duration
    return dict(segments[-1].target)
