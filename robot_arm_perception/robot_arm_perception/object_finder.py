import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from kornia.feature import LoFTR
import kornia as K
import cv2
from robot_arm_perception.numpify.image import image_to_numpy


class ObjectFinder(Node):
    def __init__(self):
        super().__init__('object_finder')

        image_topic = 'camera/color/image_raw'
        depth_topic = 'camera/aligned_depth_to_color/image_raw'
        depth_info_topic = 'camera/aligned_depth_to_color/camera_info'
        
        img_path = '/home/juan/robot_arm_ws/src/robot_arm/robot_arm_perception/robot_arm_perception/ricola.jpg'
        self._ref_image = K.image_to_tensor(cv2.imread(img_path), False).float() /255.
        self._ref_image = K.color.bgr_to_rgb(self._ref_image)

        self._depth_img_rec = False
        self._depth_info_rec = False

        self._depth_info_sub = self.create_subscription(
            CameraInfo,
            depth_info_topic,
            self._depth_info_cb,
            qos_profile_sensor_data
        )
        self._depth_info_sub

        self._depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self._depth_cb,
            qos_profile_sensor_data
        )
        self._depth_sub

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_cb,
            qos_profile_sensor_data
        )
        self._image_sub

    def _depth_cb(self, msg):
        self._depth_img_rec = True
        self._depth_image = image_to_numpy(msg)

    def _depth_info_cb(self, msg):
        self._depth_info_rec = True
        self._depth_constant = 1.0 / msg.k[4]
        self._depth_frame_id = msg.header.frame_id
        self.destroy_subscription(self._depth_info_sub)

    def _image_cb(self, msg):
        if not self._depth_img_rec or not self._depth_info_rec:
            return

        np_image = image_to_numpy(msg)
        img = K.image_to_tensor(np_image, False).float() /255.
        img = K.color.bgr_to_rgb(img)

        input_dict = {
            "image0": K.color.rgb_to_grayscale(img),
            "image1": K.color.rgb_to_grayscale(self._ref_image)
        }
        matcher = LoFTR(pretrained="indoor")

        correspondences_dict = matcher(input_dict)
        for k,v in correspondences_dict.items():
            print (k)
        mkpts0 = correspondences_dict['keypoints0'].cpu().numpy()
        mkpts1 = correspondences_dict['keypoints1'].cpu().numpy()
       
        print(correspondences_dict['confidence'],mkpts0)
# matcher = LoFTR(pretrained="indoor")
# input = {"image0": img1, "image1": img2}
# correspondences_dict = matcher(input)

# fname1 = 'kn_church-2.jpg'
# fname2 = 'kn_church-8.jpg'

# img1 = load_torch_image(fname1)
# img2 = load_torch_image(fname2)


def main(args=None):
    rclpy.init(args=args)
    o = ObjectFinder()
    rclpy.spin(o)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

