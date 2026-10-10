"""Pure joint-space validation and analytic quintic profiles (radians, seconds)."""
import math
from dataclasses import dataclass

JOINTS = tuple(f'right_j{i}' for i in range(7))
SELECTABLE_JOINTS = JOINTS + ('head_pan',)


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'Expected finite number, received {value!r}')
    return float(value)


@dataclass(frozen=True)
class Limit:
    lower: float
    upper: float
    velocity: float
    acceleration: float

    def __post_init__(self):
        for value in (self.lower, self.upper, self.velocity, self.acceleration):
            finite(value)
        if self.lower >= self.upper or self.velocity <= 0 or self.acceleration <= 0:
            raise ValueError('Invalid joint limit')


def validate_positions(positions, limits):
    if set(positions) != set(JOINTS):
        raise ValueError('Provide exactly right_j0 through right_j6')
    for joint in JOINTS:
        value = finite(positions[joint])
        limit = limits[joint]
        if not limit.lower <= value <= limit.upper:
            raise ValueError(f'{joint}: {value} outside [{limit.lower}, {limit.upper}]')


def live_limits(msg, urdf, max_velocity=0.1, max_acceleration=0.2):
    """Intersect verified controller limits with URDF and conservative application caps."""
    names = list(msg.joint_names)
    if len(names) != len(set(names)) or not set(JOINTS).issubset(names):
        raise ValueError('Missing or duplicate live joint limit names')
    for field in ('position_lower', 'position_upper', 'velocity', 'accel'):
        if len(getattr(msg, field)) != len(names):
            raise ValueError(f'Malformed JointLimits.{field}')
    if finite(max_velocity) <= 0 or finite(max_acceleration) <= 0:
        raise ValueError('Safety caps must be positive')
    output = {}
    for joint in JOINTS:
        i = names.index(joint)
        model = urdf[joint]
        controller = Limit(finite(msg.position_lower[i]), finite(msg.position_upper[i]),
                           finite(msg.velocity[i]), finite(msg.accel[i]))
        output[joint] = Limit(max(controller.lower, model['lower']),
                              min(controller.upper, model['upper']),
                              min(controller.velocity, model['velocity'], max_velocity),
                              min(controller.acceleration, max_acceleration))
    return output


class Quintic:
    def __init__(self, start, target, duration, limits, max_displacement=0.02):
        self.duration = finite(duration)
        if self.duration <= 0 or finite(max_displacement) <= 0:
            raise ValueError('Duration and maximum displacement must be positive')
        validate_positions(start, limits)
        validate_positions(target, limits)
        self.start = dict(start)
        self.delta = {joint: finite(target[joint]) - finite(start[joint]) for joint in JOINTS}
        for joint, delta in self.delta.items():
            if abs(delta) > max_displacement:
                raise ValueError(f'{joint}: displacement exceeds {max_displacement} rad')
            # Exact extrema of the quintic derivative profiles.
            if 1.875 * abs(delta) / self.duration > limits[joint].velocity:
                raise ValueError(f'{joint}: peak velocity exceeds limit; increase duration')
            if (10 / math.sqrt(3)) * abs(delta) / self.duration**2 > limits[joint].acceleration:
                raise ValueError(f'{joint}: peak acceleration exceeds limit; increase duration')

    def sample(self, elapsed):
        u = min(1.0, max(0.0, finite(elapsed) / self.duration))
        s = 10*u**3 - 15*u**4 + 6*u**5
        ds = (30*u**2 - 60*u**3 + 30*u**4) / self.duration
        dds = (60*u - 180*u**2 + 120*u**3) / self.duration**2
        return ({j: self.start[j] + self.delta[j]*s for j in JOINTS},
                {j: self.delta[j]*ds for j in JOINTS},
                {j: self.delta[j]*dds for j in JOINTS})


def ready(state, collision):
    return bool(state and collision and state.homed and state.ready and state.enabled
                and not state.stopped and not state.error and not state.low_voltage
                and state.estop_button == 0 and state.estop_source == 0
                and not collision.collision_state)


def single_joint_limit(msg, joint, model, max_velocity, max_acceleration, speed_ratio):
    if not 0 < finite(speed_ratio) <= 1:
        raise ValueError('Speed ratio must be between zero and one')
    names = list(msg.joint_names)
    if len(names) != len(set(names)) or joint not in names:
        raise ValueError(f'Missing or duplicate live limits for {joint}')
    for field in ('position_lower', 'position_upper', 'velocity', 'accel'):
        if len(getattr(msg, field)) != len(names):
            raise ValueError(f'Malformed JointLimits.{field}')
    index = names.index(joint)
    controller = Limit(finite(msg.position_lower[index]), finite(msg.position_upper[index]),
                       finite(msg.velocity[index]), finite(msg.accel[index]))
    hard_velocity = min(controller.velocity, finite(model['velocity']))
    return Limit(max(controller.lower, finite(model['lower'])),
                 min(controller.upper, finite(model['upper'])),
                 min(hard_velocity * finite(speed_ratio), finite(max_velocity)),
                 min(controller.acceleration, finite(max_acceleration)))


class JointQuintic:
    """One joint, zero endpoint derivatives, analytic limit checking over the whole curve."""
    def __init__(self, start, target, duration, limit, max_displacement=0.02):
        self.start, self.target, self.duration = map(finite, (start, target, duration))
        if self.duration <= 0 or finite(max_displacement) <= 0:
            raise ValueError('Duration and maximum displacement must be positive')
        for value in (self.start, self.target):
            if not limit.lower <= value <= limit.upper:
                raise ValueError(f'Position {value} outside [{limit.lower}, {limit.upper}]')
        self.delta = self.target - self.start
        if abs(self.delta) > max_displacement + 1e-12:
            raise ValueError(f'Displacement exceeds {max_displacement} rad; review --max-displacement')
        self.peak_velocity = 1.875 * abs(self.delta) / self.duration
        self.peak_acceleration = (10 / math.sqrt(3)) * abs(self.delta) / self.duration**2
        self.minimum_duration = max(1.875 * abs(self.delta) / limit.velocity,
                                    math.sqrt((10 / math.sqrt(3)) * abs(self.delta) / limit.acceleration))
        if self.peak_velocity > limit.velocity or self.peak_acceleration > limit.acceleration:
            raise ValueError(f'Trajectory exceeds velocity/acceleration limits; use duration >= {self.minimum_duration:.6f} s')

    def sample(self, elapsed):
        u = min(1.0, max(0.0, finite(elapsed) / self.duration))
        return (self.start + self.delta * (10*u**3 - 15*u**4 + 6*u**5),
                self.delta * (30*u**2 - 60*u**3 + 30*u**4) / self.duration,
                self.delta * (60*u - 180*u**2 + 120*u**3) / self.duration**2)
