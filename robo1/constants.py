
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d
from pyfrc.physics.units import units


# from 2022 drive base
kLeftMotor1 = 1
kLeftMotor2 = 2
kRightMotor1 = 3
kRightMotor2 = 4

kMotors = (kLeftMotor1, kLeftMotor2, kRightMotor1, kRightMotor2)

# from FRC-2023 PhotonVision-Testing branch
kLeftEncoder1 = 0
kLeftEncoder2 = 1
kRightEncoder1 = 2
kRightEncoder2 = 3

# XBox
kXbox = 0
kSimStick = 1

# Joystick buttons
# <ButtonType.kTriggerButton: 0>
# <ButtonType.kTopButton: 1>

# Axis types
# <AxisType.kThrottleAxis: 4>
# <AxisType.kTwistAxis: 3>
# <AxisType.kXAxis: 0>  negative left?
# <AxisType.kYAxis: 1>  negative forward?
# <AxisType.kZAxis: 2>  twist, on Logitech Extreme 3D Pro

cam1Name = 'camera'
cam1Pitch = 37 * units.degree
cam1OffsetY = 4 * units.inch
cam1Height = 6 * units.inch
cam1Trans = Translation3d(0, cam1OffsetY.m_as(units.m), cam1Height.m_as(units.m))
cam1Rot = Rotation3d(0, cam1Pitch, 0)
# transform camera pose to robot pose
robotToCam1 = Transform3d(cam1Trans, cam1Rot)

# Apriltags
tagsize = (6.0 * units.inch).to(units.m)

