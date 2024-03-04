
import math

# Constants for GamePads (taken from Driverstation)
class Gamepads:
    DRIVER = 0
    OPERATOR = 1

# CAN IDs for motor controllers
class CAN:
    TALON_MOTOR = 1
    SPARK_MOTOR = 2

# Constants for digitals pins on the roboRIO
class Digital:
    LEFT_DRIVE_ENCODER = (2, 3)
    RIGHT_DRIVE_ENCODER = (4, 5)

    LIMITSWITCH = 0
    COLOURMARK = 1
    BEAMBREAK = 2

# Constants for the Drivebase/Robot Driving
class Drive:
    ENCODER_PULSES_PER_REVOLUTION = 360
    WHEEL_DIAMETER = 6
    DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * math.pi / ENCODER_PULSES_PER_REVOLUTION

    LEFT_DRIVE_INVERTED = False
    RIGHT_DRIVE_INVERTED = True

    kMaxSpeedMetersPerSecond = 2
    kMaxAccelerationMetersPerSecondSquared = 1.5

# # Constants for I2C ports
# class I2C:
#     COLOUR_SENSOR = edu.wpi.first.wpilibj.I2C.Port.kOnboard
#     NAVX_I2C = edu.wpi.first.wpilibj.I2C.Port.kMXP

# Constants for LEDs
class LED:
    port = 0
    length = 60

# enum modes:
#     None,
#     Green,
#     Blue,
#     oneSpace,
