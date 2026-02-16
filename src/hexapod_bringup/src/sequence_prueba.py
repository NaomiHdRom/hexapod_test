#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class HexapodGaitController(Node):

    def __init__(self):
        super().__init__('hexapod_gait_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/hexapod_legs_controller/joint_trajectory', 10)

        # Publicar cada 0.5 segundos para mayor fluidez
        self.timer = self.create_timer(0.5, self.publish_gait)

        self.joint_names = []
        for i in range(1, 7):
            self.joint_names += [f'leg{i}_jointA', f'leg{i}_jointB', f'leg{i}_jointC']

        self.phase = 0
        self.get_logger().info("Hexapod gait controller initialized.")

    def publish_gait(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Movimiento más marcado
        swing_amplitude = 20.0  # grados
        lift_angle = -45.0      # jointC debe bajar bastante para "tocar el suelo"
        support_angle = -20.0   # para mantener las patas de soporte bajas

        positions = []
        for i in range(6):
            if (i in [0, 3, 4] and self.phase == 0) or (i in [1, 2, 5] and self.phase == 1):
                # Fase de swing (la pierna se levanta y avanza)
                positions += [
                    math.radians(swing_amplitude),  # jointA: mueve hacia adelante
                    0.0,                            # jointB: opcional
                    math.radians(lift_angle)        # jointC: levanta más la pierna
                ]
            else:
                # Fase de soporte (pierna hacia atrás y abajo)
                positions += [
                    math.radians(-swing_amplitude),  # jointA: empuja hacia atrás
                    0.0,
                    math.radians(support_angle)      # jointC: más abajo, toca el suelo
                ]

        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.5 * 1e9)  # 0.5 segundos
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"Published phase {self.phase}")
        self.phase = (self.phase + 1) % 2

def main(args=None):
    rclpy.init(args=args)
    node = HexapodGaitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
