# Command for the 6-DOF sample-collection arm (ICD v2.0 section 6.2 / 9.5).
# Consumed by rover_hardware/arm_bridge on /arm/command:
#   use_pose == true  -> ARM_SET_POSE (0x41) with pose_id
#   use_pose == false -> ARM_SET_JOINTS (0x40) with joint_deg (clamped to the
#                        ICD joint limits) and duration_ms

# predefined poses (pose_id)
uint8 POSE_HOME=0
uint8 POSE_STOW=1
uint8 POSE_READY=2
uint8 POSE_REACH_GROUND=3
uint8 POSE_GRIP_OPEN=4
uint8 POSE_GRIP_CLOSE=5
uint8 POSE_LIFT_SAMPLE=6
uint8 POSE_DEPOSIT=7

std_msgs/Header header

bool use_pose
uint8 pose_id
# servo angles [deg], order: base yaw, shoulder, elbow, wrist pitch, wrist roll, gripper
# limits: 0-180, 15-165, 0-180, 0-180, 0-180, 10-80 (10 = open)
float32[6] joint_deg
uint16 duration_ms
