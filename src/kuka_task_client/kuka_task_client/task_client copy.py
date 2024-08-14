import rclpy
from rclpy.node import Node
from kuka_control_box_srvs.srv import KukaTransformInput
import numpy as np
from scipy.spatial.transform import Rotation as R


class TaskClient(Node):

    def __init__(self):
        super().__init__('task_client')
        self.cli = self.create_client(KukaTransformInput, 'kuka_transform_input')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = KukaTransformInput.Request()

    def send_request(self):
        # 사용자 입력 받기
        x = float(input("Enter x (mm): "))
        y = float(input("Enter y (mm): "))
        z = float(input("Enter z (mm): "))
        roll = float(input("Enter roll (degrees): "))
        pitch = float(input("Enter pitch (degrees): "))
        yaw = float(input("Enter yaw (degrees): "))

        # 각도를 라디안으로 변환
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)

        # 각 축에 대한 회전 행렬 생성
        rotation_x = R.from_euler('x', roll_rad).as_matrix()
        rotation_y = R.from_euler('y', pitch_rad).as_matrix()
        rotation_z = R.from_euler('z', yaw_rad).as_matrix()

        # 회전 행렬을 곱하는 순서에 따라 최종 회전 행렬 생성 (Eigen의 순서: roll -> pitch -> yaw)
        rotation_matrix = rotation_x @ rotation_y @ rotation_z

        # 회전 행렬 출력
        print("Rotation Matrix (R):")
        print(rotation_matrix)

        # 서비스 요청에 translation 벡터와 회전 행렬 설정
        self.req.translation_x = x / 1000.0  # mm를 m로 변환
        self.req.translation_y = y / 1000.0
        self.req.translation_z = z / 1000.0

        # 3x3 회전 행렬을 일차원 배열로 변환하여 설정
        self.req.rotation_matrix = rotation_matrix.flatten().tolist()

        # 비동기 서비스 호출
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    task_client = TaskClient()
    response = task_client.send_request()

    if response.success:
        task_client.get_logger().info('Task executed successfully')
    else:
        task_client.get_logger().warn('Task execution failed')

    task_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
