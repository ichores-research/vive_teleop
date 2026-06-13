import math

from geometry_msgs.msg import PoseStamped, Quaternion


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_quaternion(quaternion: Quaternion) -> bool:
    norm_squared = (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm_squared < 1e-12:
        return False

    norm = math.sqrt(norm_squared)
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return True


def position_norm(message: PoseStamped) -> float:
    position = message.pose.position
    return math.sqrt(
        (position.x * position.x)
        + (position.y * position.y)
        + (position.z * position.z)
    )


def copy_pose_stamped(message: PoseStamped) -> PoseStamped:
    copied = PoseStamped()
    copied.header.stamp = message.header.stamp
    copied.header.frame_id = message.header.frame_id
    copied.pose.position.x = message.pose.position.x
    copied.pose.position.y = message.pose.position.y
    copied.pose.position.z = message.pose.position.z
    copied.pose.orientation.x = message.pose.orientation.x
    copied.pose.orientation.y = message.pose.orientation.y
    copied.pose.orientation.z = message.pose.orientation.z
    copied.pose.orientation.w = message.pose.orientation.w
    return copied


def multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    result = Quaternion()
    result.x = (
        left.w * right.x
        + left.x * right.w
        + left.y * right.z
        - left.z * right.y
    )
    result.y = (
        left.w * right.y
        - left.x * right.z
        + left.y * right.w
        + left.z * right.x
    )
    result.z = (
        left.w * right.z
        + left.x * right.y
        - left.y * right.x
        + left.z * right.w
    )
    result.w = (
        left.w * right.w
        - left.x * right.x
        - left.y * right.y
        - left.z * right.z
    )
    normalize_quaternion(result)
    return result


def inverse_quaternion(quaternion: Quaternion) -> Quaternion:
    result = Quaternion()
    result.x = -quaternion.x
    result.y = -quaternion.y
    result.z = -quaternion.z
    result.w = quaternion.w
    normalize_quaternion(result)
    return result


def orientation_error_vector(
    current: Quaternion,
    target: Quaternion,
) -> tuple[float, float, float]:
    error = multiply_quaternions(target, inverse_quaternion(current))
    if error.w < 0.0:
        error.x *= -1.0
        error.y *= -1.0
        error.z *= -1.0
        error.w *= -1.0

    vector_norm = math.sqrt(
        error.x * error.x + error.y * error.y + error.z * error.z
    )
    if vector_norm < 1e-9:
        return 0.0, 0.0, 0.0

    angle = 2.0 * math.atan2(vector_norm, clamp(error.w, -1.0, 1.0))
    scale = angle / vector_norm
    return error.x * scale, error.y * scale, error.z * scale


def limit_vector_norm(
    vector: tuple[float, float, float],
    limit: float,
) -> tuple[float, float, float]:
    if limit <= 0.0:
        return vector

    norm = math.sqrt(sum(component * component for component in vector))
    if norm <= limit or norm < 1e-12:
        return vector

    scale = limit / norm
    return tuple(component * scale for component in vector)


def limit_vector_change(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    max_change: float,
) -> tuple[float, float, float]:
    if max_change <= 0.0:
        return target

    delta = tuple(target[index] - current[index] for index in range(3))
    limited_delta = limit_vector_norm(delta, max_change)
    return tuple(current[index] + limited_delta[index] for index in range(3))
