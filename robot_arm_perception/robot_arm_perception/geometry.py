import numpy
import robot_arm_perception.transforms as transformations


def xyz_to_mat44(pos):
    return transformations.translation_matrix((pos.x, pos.y, pos.z))


def xyzw_to_mat44(ori):
    return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))


def fromTranslationRotation(translation, rotation):
    """
    :param translation: translation expressed as a tuple (x,y,z)
    :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
    :return: a :class:`numpy.matrix` 4x4 representation of the transform
    :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
    
    Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
    """

    return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))


def asMatrix(trans_rot):
    """
    :param target_frame: the tf target frame, a string
    :param hdr: a message header
    :return: a :class:`numpy.matrix` 4x4 representation of the transform
    :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
    
    Uses :meth:`lookupTransform` to look up the transform for ROS message header hdr to frame
    target_frame, and returns the transform as a :class:`numpy.matrix`
    4x4.
    """
    return fromTranslationRotation(trans_rot[0], trans_rot[1])

    ## Returns a Numpy 4x4 matrix for a transform.
    # @param translation  translation as (x,y,z)
    # @param rotation     rotation as (x,y,z,w)


def transform_pose(ref_trans_rot, target_trans_rot):
    """
    :param target_frame: the tf target frame, a string
    :param ps: the geometry_msgs.msg.PoseStamped message
    :return: new geometry_msgs.msg.PoseStamped message, in frame target_frame
    :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
    Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
    """
    # mat44 is frame-to-frame transform as a 4x4
    mat44 = asMatrix(ref_trans_rot)

    # pose44 is the given pose as a 4x4
    pose44 = numpy.dot(xyz_to_mat44(target_trans_rot[0]), xyzw_to_mat44(target_trans_rot[1]))

    # txpose is the new pose in target_frame as a 4x4
    txpose = numpy.dot(mat44, pose44)

    # xyz and quat are txpose's position and orientation
    xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
    quat = tuple(transformations.quaternion_from_matrix(txpose))

    return xyz, quat


def transform_point(ref_trans_rot, target_trans):
    """
    :param target_frame: the tf target frame, a string
    :param ps: the geometry_msgs.msg.PointStamped message
    :return: new geometry_msgs.msg.PointStamped message, in frame target_frame
    :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
    Transforms a geometry_msgs PointStamped message to frame target_frame, returns a new PointStamped message.
    """

    mat44 = asMatrix(ref_trans_rot)

    return tuple(numpy.dot(mat44, numpy.array([target_trans[0], target_trans[1], target_trans[2], 1.0])))[:3]

## Transforms a geometry_msgs Vector3Stamped message to frame target_frame, returns the resulting Vector3Stamped.
# @param target_frame The target frame
# @param ps           geometry_msgs.msg.Vector3Stamped object