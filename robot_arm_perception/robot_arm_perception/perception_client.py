
from robot_arm_msgs.srv import GripperPose
import rclpy
from rclpy.node import Node


class PerceptionClient(Node):
    def __init__(self):
        super().__init__('perception_client')
        self._perception_client = self.create_client(GripperPose, 'gripper_pose')

        while not self._perception_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for perception service')

    def send_request(self):
        self.future = self._perception_client.call_async(GripperPose.Request())
  
    def get_poses(self):
        if self.future.done():
            try:
                response =  self.future.result()
            except Exception as e:
                self.get_logger().info('Perception Request Failed')
                return []
            else:
                return response.poses.poses
        else:
            return []


def main(args=None):
    rclpy.init(args=args)

    perception_client = PerceptionClient()
    perception_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(perception_client)
        poses = perception_client.get_poses()
        if poses:
            for pose in poses:
                perception_client.get_logger().info(
                    f'{pose}'
                )
            break
        else:
            pass

    
    perception_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



