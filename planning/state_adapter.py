"""Intera SDK feedback conventions at the MoveIt model boundary."""
import math


def model_positions(sdk_positions):
    result = {}
    for name, value in sdk_positions.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'Invalid measured joint {name}: {value!r}')
        result[name] = float(value)
    # Intera publishes the full opening (0..0.041667 m) under this joint name.
    # The URDF has two opposed fingers, each travelling half that distance.
    if 'right_gripper_l_finger_joint' in result:
        result['right_gripper_l_finger_joint'] /= 2.
    return result
