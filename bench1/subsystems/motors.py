
# import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
# import com.ctre.phoenix.motorcontrol.can.TalonSRX;
# import com.revrobotics.CANSparkMax;
# import com.revrobotics.CANSparkLowLevel.MotorType;

import commands2
import commands2.cmd
import constants
import wpilib
import wpilib.drive
import typing

import phoenix5 as ctre
import rev
# from ctre import TalonSRXControlMode

import constants


class MotorSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.talon = ctre.TalonSRX(constants.CAN.TALON_MOTOR)
        # self.spark = rev.CANSparkMax(Constants.CAN.SPARK_MOTOR, MotorType.kBrushless)

        self.max_speed = 1.0    # abs value
        self.speed = 0

        # # The robot's drive
        # self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)

        # # The left-side drive encoder
        # self.left_encoder = wpilib.Encoder(
        #     constants.kLeftEncoderPorts[0],
        #     constants.kLeftEncoderPorts[1],
        #     constants.kLeftEncoderReversed,
        # )

        # # The right-side drive encoder
        # self.right_encoder = wpilib.Encoder(
        #     constants.kRightEncoderPorts[0],
        #     constants.kRightEncoderPorts[1],
        #     constants.kRightEncoderReversed,
        # )

        # # Creates a new drive subsystem
        # self.left_encoder.setDistancePerPulse(constants.kEncoderDistancePerPulse)
        # self.right_encoder.setDistancePerPulse(constants.kEncoderDistancePerPulse)

        # # We need to invert one side of the drivetrain so that positive voltages
        # # result in both sides moving forward. Depending on how your robot's
        # # gearbox is constructed, you might have to invert the left side instead.
        # self.right.setInverted(True)

    # def arcadeDriveCommand(
    #     self, fwd: typing.Callable[[], float], rot: typing.Callable[[], float]
    # ) -> commands2.Command:
    #     """
    #     A split-stick arcade command, with forward/backward controlled by the left hand, and turning
    #     controlled by the right.

    #     Args:
    #         fwd: Supplier for the commanded forward movement
    #         rot: Supplier for the commanded rotation
    #     """
    #     return commands2.cmd.run(
    #         lambda: self.drive.arcadeDrive(fwd(), rot(), True), self
    #     )

    # def resetEncoders(self) -> None:
    #     """Resets the drive encoders to currently read a position of 0."""
    #     self.left.reset()
    #     self.right.reset()

    # def getAverageEncoderDistance(self) -> float:
    #     """Gets the average distance of the two encoders.

    #     Returns:
    #         The average of the two encoder readings.
    #     """
    #     return (self.left_encoder.getDistance() + self.right_encoder.getDistance()) / 2

    # def getLeftEncoder(self) -> wpilib.Encoder:
    #     """Gets the left drive encoder.

    #     Returns:
    #         The left drive encoder.
    #     """
    #     return self.left_encoder

    # def getRightEncoder(self) -> wpilib.Encoder:
    #     """Gets the right drive encoder.

    #     Returns:
    #         The right drive encoder.
    #     """
    #     return self.right_encoder

    # def limitOutputCommand(self, max_output: float) -> commands2.Command:
    #     """Sets the max output of the drive. Useful for scaling the drive to drive more slowly.

    #     Args:
    #         maxoutput: The maximum output to which the drive will be constrained.
    #     """
    #     return commands2.cmd.runOnce(lambda: self.drive.setMaxOutput(max_output), self)

    # Set the motor speed using the Talon SRX
    def _setTalonSpeed(self, speed: float):
        self.talon.set(ctre.TalonSRXControlMode.PercentOutput, speed)

    # Set the motor speed using the Spark MAX
    # def setSparkSpeed(speed: float):
    #     self.spark.set(speed)

    def set_speed(self, speed):
        self.speed = speed


    def set_max_speed(self, speed):
        self.max_speed = speed


    def setDriveStates(self, states, *args):
        print(states)
        self.set_speed(states.velocity)


    def periodic(self):
        # motorOn = vMotor.get()
        # xValue = vX.get()
        motorOn = True

        speed = self.speed
        # clip to max magnitude
        if speed > self.max_speed:
            speed = self.max_speed
        elif speed < -self.max_speed:
            speed = -self.max_speed

        if motorOn:
            self._setTalonSpeed(speed)
        else:
            self._setTalonSpeed(0)
