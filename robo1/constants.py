import math

# from 2022 drive base
kLeftRear = 1     # VictorSPX
kLeftFront = 2     # CANSparkMax (inverted)
kRightRear = 3    # VictorSPX (inverted)
kRightFront = 4    # CANSparkMax

kMotors = (kLeftFront, kLeftRear, kRightFront, kRightRear)

# from FRC-2023 PhotonVision-Testing branch
kLeftEncoder1 = 0
kLeftEncoder2 = 1
kRightEncoder1 = 2
kRightEncoder2 = 3

ENCODER_PULSES_PER_REVOLUTION = 360
WHEEL_DIAMETER = 6
DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * math.pi / ENCODER_PULSES_PER_REVOLUTION

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
