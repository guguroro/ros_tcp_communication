import struct
import time

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
import math
from geometry_msgs.msg import PoseStamped, Quaternion

def bytes_to_twist(data: bytes) -> Twist:
    values = [struct.unpack('<d', data[i:i+8])[0] for i in range(0, 48, 8)]
    return Twist(
        linear=Vector3(x=values[0], y=values[1], z=values[2]),
        angular=Vector3(x=values[3], y=values[4], z=values[5])
    )

# def bytes_to_pose_stamped(data: bytes) -> PoseStamped:
#     values = [struct.unpack('<d', data[i:i+8])[0] for i in range(0, 56, 8)]
#     pose_msg = PoseStamped()
#     pose_msg.header.stamp.sec = int(time.time())
#     pose_msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
#     pose_msg.header.frame_id = "base_link"
#     pose_msg.pose.position = Point(x=values[0], y=values[1], z=values[2])
#     pose_msg.pose.orientation = Quaternion(x=values[3], y=values[4], z=values[5], w=values[6])
#     return pose_msg

#Added by Jimmy
def bytes_to_pose_stamped(data: bytes, pose_offset: int = 16) -> PoseStamped:
    if len(data) < pose_offset + 56:
        raise ValueError(f"需要至少 {pose_offset + 56} 字节数据，实际只有 {len(data)}")

    values = [struct.unpack('<d', data[pose_offset + i : pose_offset + i + 8])[0] for i in range(0, 56, 8)]

    pose_msg = PoseStamped()
    t = time.time()
    pose_msg.header.stamp.sec = int(t)
    pose_msg.header.stamp.nanosec = int((t % 1) * 1e9)
    pose_msg.header.frame_id = "base_link"

    pose_msg.pose.position = Point(x=values[0], y=values[1], z=values[2])
    pose_msg.pose.orientation = Quaternion(x=values[3], y=values[4], z=values[5], w=values[6])
    return pose_msg

# def bytes_to_ovr2ros_inputs(data: bytes) -> OVR2ROSInputs:
#     values = [struct.unpack('<d', data[i:i+8])[0] for i in range(0, 40, 8)]
#     msg = OVR2ROSInputs()
#     msg.thumb_stick_horizontal = values[0]
#     msg.thumb_stick_vertical = values[1]
#     msg.press_index = values[2]
#     msg.press_middle = values[3]
#     msg.button_lower = bool(round(values[4]))
#     return msg

def bytes_to_ovr2ros_inputs(data: bytes) -> OVR2ROSInputs:
    if len(data) < 18:
        raise ValueError(f"Expected 18 bytes, got {len(data)}")

    # 解包：2个bool + 4个float32
    button_upper, button_lower, x, y, index, middle = struct.unpack('<??ffff', data[0:18])

    msg = OVR2ROSInputs()
    msg.button_upper = button_upper
    msg.button_lower = button_lower
    msg.thumb_stick_horizontal = x
    msg.thumb_stick_vertical = y
    msg.press_index = index
    msg.press_middle = middle

    return msg

def bytes_to_ovr2ros_haptic_feedback(data: bytes) -> OVR2ROSHapticFeedback:
    freq, amp = struct.unpack('<dd', data[0:16])
    msg = OVR2ROSHapticFeedback()
    msg.frequency = freq
    msg.amplitude = amp
    return msg

def convert_data(topic: str, data: bytes):
    if topic in ["q2r_right_hand_twist", "q2r_left_hand_twist", "dice_twist", "q2r_twist"]:
        return bytes_to_twist(data)
    elif topic in ["q2r_right_hand_pose", "q2r_left_hand_pose"]:
        return bytes_to_pose_stamped(data)
    elif topic in ["q2r_right_hand_inputs", "q2r_left_hand_inputs"]:
        return bytes_to_ovr2ros_inputs(data)
    elif topic in ["q2r_right_hand_haptic_feedback", "q2r_left_hand_haptic_feedback"]:
        return bytes_to_ovr2ros_haptic_feedback(data)
    else:
        print(f"[WARNING] Unknown topic '{topic}', cannot convert.")
        return None
