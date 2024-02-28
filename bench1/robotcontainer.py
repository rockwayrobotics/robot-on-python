
import wpilib
import wpilib.drive
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard
import wpimath.trajectory

# import commands.drivedistanceprofiled
import commands2
import commands2.cmd
import commands2.button

import phoenix5 as ctre
import rev

import constants
from subsystems.motors import MotorSubsystem


class RobotContainer:
    def __init__(self):
        # Creates a ping-response Ultrasonic object on DIO
        self.rangeFinder = wpilib.Ultrasonic(3, 4) # ping, echo pins

        # Add the ultrasonic to the "Sensors" tab of the dashboard
        # Data will update automatically
        Shuffleboard.getTab("Sensors").add(self.rangeFinder)

        self.motors = MotorSubsystem()

        self.talon = ctre.TalonSRX(constants.CAN.TALON_MOTOR)

        # Retained command references
        self.driveFullSpeed = commands2.cmd.runOnce(
            lambda: self.motors.set_max_speed(1), self.motors
        )
        self.driveHalfSpeed = commands2.cmd.runOnce(
            lambda: self.motors.set_max_speed(0.5), self.motors
        )

        # The driver's controller
        self.driverController = commands2.button.CommandXboxController(
            constants.Gamepads.DRIVER
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.motors.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.cmd.run(
                lambda: self.motors.set_speed(
                    self.driverController.getRightY(),
                ),
                self.motors,
            )
        )


    # Use this method to define your button->command mappings. Buttons can be created via the button
    # factories on commands2.button.CommandGenericHID or one of its
    # subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
    def configureButtonBindings(self):
        # We can bind commands while retaining references to them in RobotContainer

        # Drive at half speed when the bumper is held
        self.driverController.rightBumper().onTrue(self.driveHalfSpeed).onFalse(
            self.driveFullSpeed
        )

        # # Drive forward by 3 meters when the 'A' button is pressed, with a timeout of 10 seconds
        # self.driverController.a().onTrue(
        #     commands.drivedistanceprofiled.DriveDistanceProfiled(
        #         3, self.motors
        #     ).withTimeout(10)
        # )

        # Do the same thing as above when the 'B' button is pressed, but defined inline
        self.driverController.b().onTrue(
            commands2.TrapezoidProfileCommand(
                wpimath.trajectory.TrapezoidProfile(
                    # Limit the max acceleration and velocity
                    wpimath.trajectory.TrapezoidProfile.Constraints(
                        constants.Drive.kMaxSpeedMetersPerSecond,
                        constants.Drive.kMaxAccelerationMetersPerSecondSquared,
                    ),
                ),
                # Pipe the profile state to the drive
                lambda setpointState: self.motors.setDriveStates(
                    setpointState, setpointState
                ),
                # End at desired position in meters; implicitly starts at 0
                lambda: wpimath.trajectory.TrapezoidProfile.State(3, 0),
                wpimath.trajectory.TrapezoidProfile.State,
                self.motors,
            )
            # .beforeStarting(self.robotDrive.resetEncoders)
            .withTimeout(10)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.none()
