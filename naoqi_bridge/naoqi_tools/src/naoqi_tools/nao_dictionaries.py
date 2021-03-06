import math
import naoqi_tools.urdf as ur
pi_2 = math.pi/2.0

Nao_offsets = {
    #### sensors offsets  ####
    # cameras
    'CameraTopV4OffsetX': 5.871e-2,
    'CameraTopV4OffsetY': 0.0,
    'CameraTopV4OffsetZ': 6.364e-2,
    'CameraTopV4RotX': 0.0,
    'CameraTopV4RotY': 2.09e-2,
    'CameraTopV4RotZ': 0.0,
    }

Nao_links = {
    u'Torso_link': 'torso',
    u'HeadYaw_link': 'Neck',
    u'HeadPitch_link': 'Head',
    u'LShoulderPitch_link': 'LShoulder',
    u'RShoulderPitch_link': 'RShoulder',
    u'LShoulderRoll_link': 'LBicep',
    u'RShoulderRoll_link': 'RBicep',
    u'LElbowYaw_link': 'LElbow',
    u'RElbowYaw_link': 'RElbow',
    u'LElbowRoll_link': 'LForeArm',
    u'RElbowRoll_link': 'RForeArm',
    u'LHipYawPitch_link': 'LPelvis',
    u'RHipYawPitch_link': 'RPelvis',
    u'LHipRoll_link': 'LHip',
    u'RHipRoll_link': 'RHip',
    u'LHipPitch_link': 'LThigh',
    u'RHipPitch_link': 'RThigh',
    u'LKneePitch_link': 'LTibia',
    u'LAnklePitch_link': 'LAnklePitch',
    u'RAnklePitch_link': 'RAnklePitch',
    u'RKneePitch_link': 'RTibia',
    u'LAnkleRoll_link': 'l_ankle',
    u'RAnkleRoll_link': 'r_ankle',
    u'LWristYaw_link': 'l_wrist',
    u'RWristYaw_link': 'r_wrist',
    u'LHand_actuator_frame': 'l_gripper',
    u'RHand_actuator_frame': 'r_gripper',
    u'LLeg_effector': 'l_sole',
    u'RLeg_effector': 'r_sole',

    # SENSORS
    'RFoot/FSR/FrontLeft_sensor': 'RFsrFL_frame',
    'RFoot/FSR/RearLeft_sensor': 'RFsrRL_frame',
    'RFoot/FSR/FrontRight_sensor': 'RFsrFR_frame',
    'RFoot/FSR/RearRight_sensor': 'RFsrRR_frame',
    'LFoot/FSR/FrontLeft_sensor': 'LFsrFL_frame',
    'LFoot/FSR/RearLeft_sensor': 'LFsrRL_frame',
    'LFoot/FSR/FrontRight_sensor': 'LFsrFR_frame',
    'LFoot/FSR/RearRight_sensor': 'LFsrRR_frame',
    'Sonar/Right_sensor': 'RSonar_frame',
    'Sonar/Left_sensor': 'LSonar_frame',
    'InfraredL_sensor': 'LInfraRed_frame',
    'InfraredR_sensor': 'RInfraRed_frame',
    'CameraTop_sensor': 'CameraTop_frame',
    'CameraBottom_sensor': 'CameraBottom_frame',
    'LFoot/Bumper/Left_sensor': 'LFootBumperLeft_frame',
    'LFoot/Bumper/Right_sensor': 'LFootBumperRight_frame',
    'RFoot/Bumper/Left_sensor': 'RFootBumperLeft_frame',
    'RFoot/Bumper/Right_sensor': 'RFootBumperRight_frame',
    'ChestBoard/Button_sensor': 'ChestButton_frame',
    'Head/Touch/Front_sensor': 'HeadTouchFront_frame',
    'Head/Touch/Middle_sensor': 'HeadTouchMiddle_frame',
    'Head/Touch/Rear_sensor': 'HeadTouchRear_frame',
    'LHand/Touch/Left_sensor': 'LHandTouchLeft_frame',
    'LHand/Touch/Back_sensor': 'LHandTouchBack_frame',
    'LHand/Touch/Right_sensor': 'LHandTouchRight_frame',
    'RHand/Touch/Left_sensor': 'RHandTouchLeft_frame',
    'RHand/Touch/Back_sensor': 'RHandTouchBack_frame',
    'RHand/Touch/Right_sensor': 'RHandTouchRight_frame',
    'Accelerometer_sensor': 'ImuTorsoAccelerometer_frame',
    'Gyrometer_sensor': 'ImuTorsoGyrometer_frame',
    'MicroFront_sensor': 'MicroFrontCenter_frame',
    'MicroRear_sensor': 'MicroRearCenter_frame',
    'MicroLeft_sensor': 'MicroSurroundLeft_frame',
    'MicroRight_sensor': 'MicroSurroundRight_frame',
    }

Nao_visu = {
    u'Torso_link': ur.Cylinder(0.015, 0.2115),
    u'HeadPitch_link': ur.Cylinder(0.04, 0.14),
    u'LShoulderRoll_link': ur.Cylinder(0.015, 0.09),
    u'LElbowRoll_link': ur.Cylinder(0.015, 0.05065),
    u'LHipPitch_link': ur.Cylinder(0.015, 0.1),
    u'LKneePitch_link':   ur.Cylinder(0.015, 0.1),
    u'RHipPitch_link': ur.Cylinder(0.015, 0.1),
    u'RKneePitch_link': ur.Cylinder(0.015, 0.1),
    u'RShoulderRoll_link': ur.Cylinder(0.015, 0.09),
    u'RElbowRoll_link':  ur.Cylinder(0.015, 0.05065),
    u'LAnkleRoll_link': ur.Box((0.16, 0.06, 0.015)),
    u'RAnkleRoll_link': ur.Box((0.16, 0.06, 0.015)),
    u'LWristYaw_link':  ur.Cylinder(0.015, 0.058),
    u'RWristYaw_link':  ur.Cylinder(0.015, 0.058),
    }

Nao_orig = {
    u'Torso_link': ur.Pose((0, 0, 0.02075), (0, 0, 0)),
    u'HeadPitch_link':  ur.Pose((0, 0, 0.058), (pi_2, 0, 0)),
    u'LShoulderRoll_link':  ur.Pose((0.045, 0, 0), (pi_2, 0, pi_2)),
    u'LElbowRoll_link': ur.Pose((0.025325, 0, 0), (pi_2, 0, pi_2)),
    u'LHipPitch_link': ur.Pose((0, 0, -0.05), (0, 0, 0)),
    u'LKneePitch_link': ur.Pose((0, 0, -0.05), (0, 0, 0)),
    u'RHipPitch_link': ur.Pose((0, 0, -0.05), (0, 0, 0)),
    u'RKneePitch_link': ur.Pose((0, 0, -0.05), (0, 0, 0)),
    u'RShoulderRoll_link': ur.Pose((0.045, 0, 0), (pi_2, 0, pi_2)),
    u'RElbowRoll_link': ur.Pose((0.025325, 0, 0), (pi_2, 0, pi_2)),
    u'LAnkleRoll_link': ur.Pose((0.02, 0, 0.0075), (0, 0, 0)),
    u'RAnkleRoll_link': ur.Pose((0.02, 0, 0.0075), (0, 0, 0)),
    u'LWristYaw_link': ur.Pose((0.029, 0, 0), (pi_2, 0, pi_2)),
    u'RWristYaw_link': ur.Pose((0.029, 0, 0), (pi_2, 0, pi_2)),
    }
