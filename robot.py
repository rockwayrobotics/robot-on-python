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
    def buildMotors(self):
        if self.isSimulation():
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


    def robotInit(self):
        """Robot initialization function"""
        self.left1, self.left2, self.right1, self.right2 = self.buildMotors()

        self.left = wpilib.MotorControllerGroup(self.left1, self.left2)
        self.right = wpilib.MotorControllerGroup(self.right1, self.right2)

        # # This was in 2022
        # self.left.setInverted(True)
        # self.right.setInverted(True)

        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joystick 1 is index 0?
        # self.stick = wpilib.Joystick(0)
        self.drvStick = wpilib.XboxController(kXbox)


    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)


    def teleopPeriodic(self):
        """Runs the motors with X steering (arcade or tank)"""
        self.myRobot.arcadeDrive(self.drvStick.getX(), -self.drvStick.getY())


if __name__ == "__main__":
    wpilib.run(MyRobot)
