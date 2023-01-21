#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    tank drive.
"""

import wpilib
from wpilib.drive import DifferentialDrive

from constants import *


class MyRobot(wpilib.TimedRobot):
    def buildDriveMotors(self):
        '''Create and return the drive motors for sim or normal mode.'''
        if self.sim:
            return (
                wpilib.PWMSparkMax(kLeftMotor1),
                wpilib.PWMSparkMax(kLeftMotor2),
                wpilib.PWMSparkMax(kRightMotor1),
                wpilib.PWMSparkMax(kRightMotor2),
                )
        else:
            brushed = rev.CANSparkMax.MotorType.kBrushed
            return (
                rev.CANSparkMax(kLeftMotor1, brushed),
                rev.CANSparkMax(kLeftMotor2, brushed),
                rev.CANSparkMax(kRightMotor1, brushed),
                rev.CANSparkMax(kRightMotor2, brushed),
                )


    def buildDriveStick(self):
        '''Create and return the drive joystick for sim or normal mode.'''
        if self.sim:
            return wpilib.Joystick(0)
        else:
            return wpilib.XboxController(kXbox)


    def robotInit(self):
        """Robot initialization function"""
        self.sim = self.isSimulation()

        self.left1, self.left2, self.right1, self.right2 = self.buildDriveMotors()

        self.left = wpilib.MotorControllerGroup(self.left1, self.left2)
        self.right = wpilib.MotorControllerGroup(self.right1, self.right2)

        # # This was in 2022
        # self.left.setInverted(True)
        # self.right.setInverted(True)

        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        self.driveStick = self.buildDriveStick()


    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)


    def teleopPeriodic(self):
        """Runs the motors with X steering (arcade, tank, curvature)"""
        dstick = self.driveStick

        # TODO: make a class to delegate more cleanly to a joystick configured
        # appropriately for sim or normal mode, so we can use common code here
        if self.sim:
            self.myRobot.curvatureDrive(dstick.getX(), dstick.getY())
        else:
            # matches FRC-2023 5616e35
            self.myRobot.curvatureDrive(-dstick.getLeftY(), dstick.getLeftX())


if __name__ == "__main__":
    wpilib.run(MyRobot)
