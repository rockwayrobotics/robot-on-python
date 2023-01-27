#!/usr/bin/env python3
"""Experimental RobotPy for FRC 2023."""

import ntcore

import wpilib
from wpilib.drive import DifferentialDrive

# Note: could move this below so we do it only in normal mode, allowing
# tests/sim to run faster if they don't require this.
import rev

from constants import *


class MyRobot(wpilib.TimedRobot):
    state = 'init'

    def buildDriveMotors(self):
        '''Create and return the drive motors for sim or normal mode.'''
        if self.sim:
            return [wpilib.PWMSparkMax(x) for x in kMotors]
        else:
            brushed = rev.CANSparkMax.MotorType.kBrushed
            return [rev.CANSparkMax(x, brushed) for x in kMotors]


    def buildStick(self, sim=False):
        '''Routine create a control device.  This allows us to
        substitute a simulated one when appropriate, etc.  The current
        code doesn't really do that.'''
        if sim:
            return wpilib.Joystick(kSimStick)
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
        # self.myRobot.setExpiration(0.1)

        self.simStick = self.buildStick(sim=True)
        self.driveStick = self.buildStick()

        self.accel = wpilib.BuiltInAccelerometer()

        # self.dash = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        self.dash = wpilib.SmartDashboard

        self.ds = wpilib.DSControlWord()
        # print('ds attached', self.ds.isDSAttached())

        # self.updateDashboard()

        print(f'stick: {self.driveStick.isConnected()}')

        # if self.sim:
        #     breakpoint()


    def updateDashboard(self):
        dash = wpilib.SmartDashboard
        dash.putString('State', self.state)
        # dash.putBoolean('Disabled?', self.state == 'disabled')
        # dash.putBoolean('Autonomous?', self.state == 'auto')
        # dash.putBoolean('Teleop?', self.state == 'teleop')

        axes = ','.join(f'{k}={v:.2f}' for k,v in zip('xyz', (self.accel.getX(), self.accel.getY(), self.accel.getX())))
        dash.putString('accel', axes)

        if self.simStick.isConnected():
            text = f'x={self.simStick.getX():.2f} y={self.simStick.getY():.2f}'
            dash.putString('joy', text)
        else:
            dash.putString('joy', 'missing')

        if self.driveStick.isConnected():
            text = f'x={self.driveStick.getLeftX():.2f} y={self.driveStick.getLeftY():.2f}'
            dash.putString('xbox', text)
        else:
            dash.putString('xbox', 'missing')


    def robotPeriodic(self):
        self.updateDashboard()


    def disabledInit(self):
        self.state = 'disabled'
        print('state: disabled')
        self.myRobot.arcadeDrive(0, 0)


    # def disabledPeriodic(self):
    #     self.updateDashboard()


    def autonomousInit(self):
        self.state = 'auto'
        print('state: auto')
        if not self.sim:
            self.myRobot.setSafetyEnabled(True)
        self.phase = self.run_phases()

        print(f'stick: {self.driveStick.isConnected()}')


    def autonomousExit(self):
        self.state = 'between'


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
        self.state = 'teleop'
        print('state: teleop')
        self.myRobot.setSafetyEnabled(not self.sim)
        # self.logger.info('batt: %s', self.getBatteryVoltage())
        data = wpilib.DriverStation.getGameSpecificMessage()
        if data:
            # Set the robot gamedata property and set a network tables value
            self.gameData = data
            self.dash.putString("gameData", self.gameData)

        print(f'stick: {self.driveStick.isConnected()}')

        # if self.sim:
        #     breakpoint()

    def teleopExit(self):
        self.state = 'between'


    def teleopPeriodic(self):
        """Runs the motors with X steering (arcade, tank, curvature)"""
        dstick = self.driveStick

        # TODO: make a class to delegate more cleanly to a joystick configured
        # appropriately for sim or normal mode, so we can use common code here
        if self.sim:
            self.myRobot.curvatureDrive(self.simStick.getX() * 0.4, self.simStick.getY(), True)
            # self.myRobot.arcadeDrive(self.simStick.getX() * 0.5, self.simStick.getY())
            # self.myRobot.curvatureDrive(-dstick.getLeftY(), dstick.getLeftX(), False)
        else:
            self.myRobot.curvatureDrive(dstick.getLeftX(), dstick.getLeftY(), True)


    def testInit(self):
        self.state = 'test'

    def testExit(self):
        self.state = 'between'


if __name__ == "__main__":
    wpilib.run(MyRobot)
