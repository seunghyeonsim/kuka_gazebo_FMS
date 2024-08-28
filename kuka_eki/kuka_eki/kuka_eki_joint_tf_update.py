import rclpy
from rclpy.node import Node
from kuka_control_box_srvs.srv import KukaJoint
from kuka_eki.eki import EkiMotionClient, EkiStateClient
from kuka_eki.krl import Axis
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np


class KukaEkiJointTfUpdate(Node):

    def __init__(self):
        super().__init__('kuka_eki_joint_tf_update')

        # Publisher 초기화 (항상 작동)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "end_effector_joint"]
        self.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0]

        # 노드 시작 시 기본 joint state 바로 퍼블리시
        self.publish_initial_joint_state()

        # 타이머 설정: 주기적으로 기본 값을 퍼블리시
        timer_period = 0.1  # 0.1초마다 실행
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Service 초기화 (서비스 요청이 있을 때만 작동)
        self.joint_service = self.create_service(KukaJoint, 'kuka_joint', self.handle_joint_service)

        # EKI 클라이언트 초기화
        self.eki_connected = False
        self.initialize_eki_clients()

        # 노드 종료 시 EKI 클라이언트 종료 처리
        self.create_shutdown_callback(self.shutdown_callback)

    def publish_initial_joint_state(self):
        """초기 joint state를 바로 퍼블리시"""
        self.get_logger().info("Publishing initial joint state")
        self.publish_joint_state(self.joint_positions)

    def initialize_eki_clients(self):
        """Separate EKI initialization to handle cases where connection fails."""
        try:
            self.eki_motion_client = EkiMotionClient("172.31.1.147")
            self.eki_state_client = EkiStateClient("172.31.1.147")
            self.eki_motion_client.connect()
            self.eki_state_client.connect()
            self.eki_connected = True
            self.get_logger().info("EKI 연결 성공")
        except Exception as e:
            self.get_logger().error(f"EKI 연결 실패: {e}")
            self.eki_connected = False

    def timer_callback(self):
        if not self.eki_connected:
            self.get_logger().info("EKI 연결이 되어 있지 않아서 재연결 시도 중...")
            self.initialize_eki_clients()

        if self.eki_connected:
            self.get_logger().info("Publishing joint states from timer callback")
            self.publish_joint_state(self.joint_positions)
        else:
            self.get_logger().warn("EKI 연결 실패로 인해 joint states를 퍼블리시할 수 없습니다.")

    def handle_joint_service(self, request, response):
        if not self.eki_connected:
            self.get_logger().error("EKI 연결이 되어 있지 않아서 서비스를 처리할 수 없습니다.")
            response.success = False
            return response

        target_axis = Axis(request.a1, request.a2, request.a3, request.a4, request.a5, request.a6)
        self.eki_motion_client.ptp(target_axis, 0.5)

        while True:
            state = self.eki_state_client.state()
            current_axis = np.array([state.axis.a1, state.axis.a2, state.axis.a3, state.axis.a4, state.axis.a5, state.axis.a6], dtype=float)
            target_axis_array = np.array([request.a1, request.a2, request.a3, request.a4, request.a5, request.a6])

            # 서비스 요청에 따른 현재 JointState 퍼블리시
            self.joint_positions = list(current_axis) + [0.0]
            self.get_logger().info(f"Publishing joint states from service: {self.joint_positions}")
            self.publish_joint_state(self.joint_positions)

            if np.linalg.norm(current_axis - target_axis_array) < 0.1:
                break

        response.success = True
        response.a1 = float(state.axis.a1)
        response.a2 = float(state.axis.a2)
        response.a3 = float(state.axis.a3)
        response.a4 = float(state.axis.a4)
        response.a5 = float(state.axis.a5)
        response.a6 = float(state.axis.a6)
        response.x = float(state.pos.x)
        response.y = float(state.pos.y)
        response.z = float(state.pos.z)
        response.a = float(state.pos.a)
        response.b = float(state.pos.b)
        response.c = float(state.pos.c)
        response.s = float(state.pos.s)
        response.t = float(state.pos.t)
        
        return response

    def publish_joint_state(self, positions):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.get_logger().info(f"Publishing: {positions}")
        self.publisher.publish(msg)

    def shutdown_callback(self):
        """Handles cleanup when the node is shutting down."""
        if self.eki_connected:
            self.eki_motion_client.disconnect()
            self.eki_state_client.disconnect()
            self.get_logger().info("EKI 클라이언트 연결 종료됨")


def main(args=None):
    rclpy.init(args=args)
    node = KukaEkiJointTfUpdate()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
