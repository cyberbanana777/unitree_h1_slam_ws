# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS 2-узел для преобразования и публикации позиций суставов робота H1 в 
формате JointState. Обрабатывает JSON-команды с ограничениями диапазонов из
vendor-specific данных. Критичные зависимости: ROS 2 Humble, sensor_msgs,
rclpy, аппаратные лимиты суставов.

ANNOTATION
ROS 2 node for converting and publishing H1 robot joint positions as 
JointState. Processes JSON commands with vendor-specific range constraints.
Key dependencies: ROS 2 Humble, sensor_msgs, rclpy, hardware joint limits.
'''

import json
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
            String,
            'positions_to_unitree',
            self.listener_callback_positions_to_unitree,
            10
        )

        self.publisher_joint_state = self.create_publisher(
            JointState,
            'joint_states', 
            10
        )

        self.timer = self.create_timer(self.control_dt, self.timer_callback)

    def listener_callback_positions_to_unitree(self, msg):
        """
        Process incoming position commands from the 'positions_to_unitree' 
        topic. Message format should be: JSON_POSITIONS$IMPACT_VALUE
        """
        raw_data = msg.data

        # Split message into position data and impact value
        parts = raw_data.split("$")
        if len(parts) != 2:
            self.get_logger().error(f"Invalid message format: {raw_data}")
            return

        data_part, impact_part = parts

        # Skip if no position data
        if not data_part or data_part == "{}":
            self.get_logger().debug(
                "Empty pose data received, skipping processing"
            )
            return

        # Process position data
        try:
            pose = json.loads(data_part)
            for i in range(len(pose) + 1):
                if i != 9:
                    self.current_jpos_H1[i] = pose[str(i)]
                else:
                    self.current_jpos_H1[i] = float(impact_part)


        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

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