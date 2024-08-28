import rclpy
from rclpy.node import Node
from kuka_control_box_srvs.srv import KukaJoint
from kuka_eki.eki import EkiMotionClient, EkiStateClient
from kuka_eki.krl import Axis
import numpy as np

class KukaEkiJointServer(Node):

    def __init__(self):
        super().__init__('kuka_eki_joint_server')
        self.joint_service = self.create_service(KukaJoint, 'kuka_joint', self.handle_joint_service)
        self.eki_motion_client = EkiMotionClient("172.31.1.147")
        self.eki_state_client = EkiStateClient("172.31.1.147")
        self.eki_motion_client.connect()
        self.eki_state_client.connect()

    def handle_joint_service(self, request, response):
        target_axis = Axis(request.a1, request.a2, request.a3, request.a4, request.a5, request.a6)
        self.eki_motion_client.ptp(target_axis, 0.5)

        while True:
            state = self.eki_state_client.state()
            # print(f"Current State: Axis({state.axis.a1}, {state.axis.a2}, {state.axis.a3}, {state.axis.a4}, {state.axis.a5}, {state.axis.a6})")
            current_axis = np.array([state.axis.a1, state.axis.a2, state.axis.a3, state.axis.a4, state.axis.a5, state.axis.a6], dtype=float)
            target_axis_array = np.array([request.a1, request.a2, request.a3, request.a4, request.a5, request.a6])
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

def main(args=None):
    rclpy.init(args=args)
    node = KukaEkiJointServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
