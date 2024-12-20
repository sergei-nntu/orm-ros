import numpy as np
import array as ar


def joints_state_validation(joint_positions, expected_joint_count):
    # FIXME: remove list
    if not isinstance(joint_positions, (list, np.ndarray, ar.array)):
            raise ValueError("joint_positions must be a list or numpy array or array.")
    elif len(joint_positions) != expected_joint_count:
        raise ValueError(f"Expected {expected_joint_count} joint positions, got {len(joint_positions)}.")

    return joint_positions