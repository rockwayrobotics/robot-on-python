#!/usr/bin/env python3
"""Experimental RobotPy for FRC 2023."""

import wpilib
from wpilib.drive import DifferentialDrive

# Note: could move this below so we do it only in normal mode, allowing
# tests/sim to run faster if they don't require this.
import rev

from constants import *


class MyRobot(wpilib.TimedRobot):
    def buildDriveMotors(self):
        '''Create and return the drive motors for sim or normal mode.'''
        if self.sim:
            return [wpilib.PWMSparkMax(x) for x in kMotors]
        else:
            brushed = rev.CANSparkMax.MotorType.kBrushed
            return [rev.CANSparkMax(x, brushed) for x in kMotors]


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


    def disabledInit(self):
        self.myRobot.arcadeDrive(0, 0)


    def autonomousInit(self):
        """Executed at the start of autonomous mode"""
        self.myRobot.setSafetyEnabled(True)
        self.phase = self.run_phases()

    # crude substitute for Command stuff
    PHASES = [
        (1.0, 'initial'),
        (4.0, 'curve_out'),
        (1.1, 'pivot_left'),
        (2.5, 'straight'),
        (5.0, 'back_right'),
        ]

    def _phase_initial(self):       self.myRobot.arcadeDrive(0, 0)
    def _phase_curve_out(self):     self.myRobot.arcadeDrive(-0.1, -0.7)
    def _phase_pivot_left(self):    self.myRobot.arcadeDrive(-0.3, -0.5)
    def _phase_straight(self):      self.myRobot.arcadeDrive(0, -0.80)
    def _phase_back_right(self):    self.myRobot.arcadeDrive(-0.2, 0.6)

    def run_phases(self):
        self._phase = 0
        timer = wpilib.Timer()
        timer.start()

        for (duration, name) in self.PHASES:
            self.logger.info('phase: %s (%.1fs)', name, duration)

            handler = getattr(self, f'_phase_{name}')

            timer.reset()
            while timer.get() < duration:
                # self.logger.info('phase: %s @%.1fs', name, timer.get())
                handler()
                yield


    def autonomousPeriodic(self):
        """Autonomous sequence"""
        try:
            next(self.phase)
        except StopIteration:
            self.myRobot.arcadeDrive(0, 0)
            # breakpoint()


    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)
        # self.logger.info('', self.getBatteryVoltage())


    def teleopPeriodic(self):
        """Runs the motors with X steering (arcade, tank, curvature)"""
        dstick = self.driveStick

        # TODO: make a class to delegate more cleanly to a joystick configured
        # appropriately for sim or normal mode, so we can use common code here
        if self.sim:
            self.myRobot.arcadeDrive(dstick.getX() * 0.5, dstick.getY())
        else:
            # matches FRC-2023 5616e35
            self.myRobot.curvatureDrive(-dstick.getLeftY(), dstick.getLeftX())


if __name__ == "__main__":
    wpilib.run(MyRobot)
