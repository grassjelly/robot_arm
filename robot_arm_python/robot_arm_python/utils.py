import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped


def make_pose(frame_id, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


def transform_pose(
    pose: PoseStamped,
    tf_buffer: tf2_ros.Buffer,
    source_frame: str = "flange_link",
    target_frame: str = "tool_link",
) -> PoseStamped:
    """Re-express a pose goal from source_frame to target_frame.

    Both frames must be collocated (same position, different orientation —
    e.g. flange_link and tool_link share the same xyz offset from gripper_link).
    The position is preserved; the orientation is adjusted by the static
    rotation between the two frames.

    Args:
        pose:         Goal pose (PoseStamped in any reference frame).
        tf_buffer:    TF2 buffer for the static transform lookup.
        source_frame: Link the pose was originally designed for (default: "flange_link").
        target_frame: Link to use as the new planning target (default: "tool_link").

    Returns:
        Equivalent PoseStamped goal for target_frame.
    """
    t = tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
    q_rot = t.transform.rotation

    x, y, z, w = _quat_multiply(pose.pose.orientation, q_rot)

    result = PoseStamped()
    result.header = pose.header
    result.pose.position = pose.pose.position
    result.pose.orientation.x = x
    result.pose.orientation.y = y
    result.pose.orientation.z = z
    result.pose.orientation.w = w
    return result


def _quat_multiply(q1, q2) -> tuple:
    """Hamilton product of two quaternions represented as objects with x/y/z/w."""
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
