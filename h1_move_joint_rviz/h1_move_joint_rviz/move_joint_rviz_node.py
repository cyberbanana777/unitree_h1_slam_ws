from unitree_go.msg import MotorState
from unitree_go.msg import MotorStates
from unitree_go.msg import MotorCmd
from unitree_go.msg import MotorCmds
from unitree_go.msg import LowCmd
from unitree_go.msg import LowState
from sensor_msgs.msg import JointState
from rclpy.node import Node
import rclpy

# Частота в гц для ноды
FREQUENCY = 333.33

JOINT_INDEX_H1 = {
    'right_hip_roll_joint': 0,
    'right_hip_pitch_joint': 1,
    'right_knee_joint': 2,
    'left_hip_roll_joint': 3,
    'left_hip_pitch_joint': 4,
    'left_knee_joint': 5,
    'torso_joint': 6,
    'left_hip_yaw_joint': 7,
    'right_hip_yaw_joint': 8,
    'NOT USED': 9,
    'left_ankle_joint': 10,
    'right_ankle_joint': 11,
    'right_shoulder_roll_joint': 12,
    'right_shoulder_pitch_joint': 13,
    'right_shoulder_yaw_joint': 14,
    'right_elbow_joint': 15,
    'left_shoulder_roll_joint': 16,
    'left_shoulder_pitch_joint': 17,
    'left_shoulder_yaw_joint': 18,
    'left_elbow_joint': 19
}

JOINT_INDEX_HANDS = {
    'right_pinky': 0,
    'right_ring': 1,
    'right_middle': 2,
    'right_index': 3,
    'right_thumb_bend': 4,
    'right_thumb_rotation': 5,
    'left_pinky': 6,
    'left_ring': 7,
    'left_middle': 8,
    'left_index': 9,
    'left_thumb_bend': 10,
    'left_thumb_rotation': 11,
}

class MoveJointRvizNode(Node):

    def __init__(self):
        super().__init__('move_joint_rviz')
        self.get_logger().info('Node started')

                # текущая позиция
        self.current_jpos_H1 = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        self.current_jpos_hands = {
            0: 1.0,  # pinky
            1: 1.0,  # ring
            2: 1.0,  # middle
            3: 1.0,  # index
            4: 1.0,  # thumb-bend
            5: 1.0,  # thumb-rotation
            6: 1.0,  # pinky
            7: 1.0,  # ring
            8: 1.0,  # middle
            9: 1.0,  # index
            10: 1.0,  # thumb-bend
            11: 1.0  # thumb-rotation
        }
    
        self.control_dt = 1 / FREQUENCY

        # подписка на топик, в который публикуется информация о углах поворота звеньев
        self.subscription_LowCmd = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback_LowCmd,
            10
        )
        
        self.publisher_joint_state = self.create_publisher(
            JointState,
            '/joint_states', 
            10
        )

        self.subscription_state = self.create_subscription(
            MotorStates,
            'inspire/state',
            self.listener_callback_states,
            10
        )

        self.timer = self.create_timer(self.control_dt, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
                    "right_hip_roll_joint", 
                    "right_hip_pitch_joint", 
                    "right_knee_joint",
                    "left_hip_roll_joint", 
                    "left_hip_pitch_joint", 
                    "left_knee_joint",
                    "torso_joint", 
                    "left_hip_yaw_joint", 
                    "right_hip_yaw_joint",
                    "left_ankle_joint",
                    "right_ankle_joint",
                    "right_shoulder_pitch_joint",  # Исправлено shouldEr -> shoulder
                    "right_shoulder_roll_joint",   # Исправлено
                    "right_shoulder_yaw_joint",    # Исправлено
                    "right_elbow_joint",
                    "left_shoulder_pitch_joint",   # Исправлено
                    "left_shoulder_roll_joint",    # Исправлено 
                    "left_shoulder_yaw_joint", 
                    "left_elbow_joint",
                    "R_pinky_proximal_joint",
                    "R_ring_proximal_joint",
                    "R_middle_proximal_joint",
                    "R_index_proximal_joint",
                    "R_thumb_proximal_pitch_joint",
                    "R_thumb_proximal_yaw_joint",
                    "L_pinky_proximal_joint",
                    "L_ring_proximal_joint",
                    "L_middle_proximal_joint",
                    "L_index_proximal_joint",
                    "L_thumb_proximal_pitch_joint",
                    "L_thumb_proximal_yaw_joint"
                ]
        
        h1_positions = [self.current_jpos_H1[k] for k in sorted(self.current_jpos_H1)]
        hands_positions = [self.current_jpos_hands[k] for k in sorted(self.current_jpos_hands)]
        msg.position = h1_positions + hands_positions

        self.publisher_joint_state.publish(msg)

    def listener_callback_LowCmd(self, msg):
        '''Обновляем текущее положение'''
        for i in list(self.current_jpos_H1.keys()):
            self.current_jpos_H1[i] = msg.motor_state[i].q

    def listener_callback_states(self, msg):
        '''Обновляем текущее положение кистей'''
        for i in list(self.current_jpos_hands.keys()):
            self.current_jpos_hands[i] = msg.states[i].q

def main(args=None):
    """Основная функция для запуска ноды."""
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
