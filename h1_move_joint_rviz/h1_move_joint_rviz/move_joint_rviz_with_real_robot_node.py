# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS 2-узел для конвертации и публикации состояний суставов робота Unitree H1 в
RViz. Агрегирует данные из трех источников: низкоуровневые состояния
(LowState), пальцы (Inspire) и запястья. Критичные зависимости: ROS 2 Humble,
unitree_go, sensor_msgs, аппаратные лимиты суставов.

ANNOTATION
ROS 2 node for converting and publishing Unitree H1 joint states to RViz. 
Aggregates data from three sources: low-level states (LowState), fingers
(Inspire) and wrists. Key dependencies: ROS 2 Humble, unitree_go, sensor_msgs,
hardware joint limits.
'''

from unitree_go.msg import MotorStates
from unitree_go.msg import LowState
from sensor_msgs.msg import JointState
from rclpy.node import Node
import rclpy
from h1_info_library import LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR
from h1_move_joint_rviz import (
    LIMITS_URDF, START_POSITION, JOINTS_NAMES, map_range
)

# Frequency in Hz for the node
FREQUENCY = 60.0
FINGERS_OFFSET = 20
WRISTS_OFFSET = 32


class MoveJointRvizNode(Node):

    def __init__(self):
        super().__init__('move_joint_rviz')
        self.get_logger().info('Node started')

        # current position
        self.current_jpos_H1 = START_POSITION.copy()
    

        self.control_dt = 1 / FREQUENCY

        self.subscription_lowstate = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback_lowstate,
            10
        )

        self.subscription_state = self.create_subscription(
            MotorStates,
            'inspire/state',
            self.listener_callback_inspire_states,
            10
        )

        self.subscription_wrist_state = self.create_subscription(
            MotorStates,
            'wrist/states',
            self.listener_callback_wrist_states,
            10
        )

        self.publisher_joint_state = self.create_publisher(
            JointState,
            'joint_states', 
            10
        )

        self.timer = self.create_timer(self.control_dt, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pelvis"
        msg.name = JOINTS_NAMES.copy()
        
        h1_positions = []
        
        for i in range(len(self.current_jpos_H1)):
            if i != 9:
                orig_lim = LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR[i]
                rviz_lim = LIMITS_URDF[i]
                converted_value = map_range(
                    self.current_jpos_H1[i],
                    orig_lim[0],
                    orig_lim[1],
                    rviz_lim[0],
                    rviz_lim[1],
                )
                h1_positions.append(converted_value)
        
        msg.position = h1_positions

        self.publisher_joint_state.publish(msg)

    def listener_callback_lowstate(self, msg):
        '''Updating the current position of the robot'''
        for i in range(20):
            self.current_jpos_H1[i] = msg.motor_state[i].q

    def listener_callback_inspire_states(self, msg):
        '''Updating the current position of the fingers'''
        for i in range(12):
            self.current_jpos_H1[i + FINGERS_OFFSET] = msg.states[i].q

    def listener_callback_wrist_states(self, msg):
        '''Updating the current position of the brushes'''
        for i in range(2):
            self.current_jpos_H1[i + WRISTS_OFFSET] = msg.states[i].q


def main(args=None):
    """The main function for launching a node."""
    rclpy.init(args=args)
    node = MoveJointRvizNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(e)

    finally:
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)