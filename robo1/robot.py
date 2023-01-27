#!/usr/bin/env python3
"""Experimental RobotPy for FRC 2023."""

import ntcore

import wpilib
# import wpilib.deployinfo
from wpilib.drive import DifferentialDrive
# from wpilib import deployinfo

# Note: could move this below so we do it only in normal mode, allowing
# tests/sim to run faster if they don't require this.
import rev

from constants import *

DASH = wpilib.SmartDashboard

ARCADE = True

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

        self.ds = wpilib.DSControlWord()
        # print('ds attached', self.ds.isDSAttached())
        DASH.putString('git', DEPLOY_INFO.get('git-desc', 'missing'))

        # self.updateDashboard()

        print(f'stick: {self.driveStick.isConnected()}')

        # if self.sim:
        #     breakpoint()


    def updateDashboard(self):
        DASH.putString('State', self.state)
        # DASH.putBoolean('Disabled?', self.state == 'disabled')
        # DASH.putBoolean('Autonomous?', self.state == 'auto')
        # DASH.putBoolean('Teleop?', self.state == 'teleop')

        axes = ','.join(f'{k}={v:.2f}' for k,v in zip('xyz', (self.accel.getX(), self.accel.getY(), self.accel.getX())))
        DASH.putString('accel', axes)

        if self.simStick.isConnected():
            text = f'x={self.simStick.getX():.2f} y={self.simStick.getY():.2f}'
            DASH.putString('joy', text)
        else:
            DASH.putString('joy', 'missing')

        if self.driveStick.isConnected():
            text = f'x={self.driveStick.getLeftX():.2f} y={self.driveStick.getLeftY():.2f}'
            DASH.putString('xbox', text)
        else:
            DASH.putString('xbox', 'missing')


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
            DASH.putString("gameData", self.gameData)

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
            dstick = self.simStick
            if ARCADE:
                speed_scale = 0.7 if dstick.getTrigger() else 1.0
                rot_scale = 0.4 if dstick.getTrigger() else 0.6
                # True mean square inputs (higher sensitivity at low values)
                self.myRobot.arcadeDrive(dstick.getX() * rot_scale, dstick.getY() * speed_scale, True)
            else:
                speed_scale = 0.3 if dstick.getTrigger() else 1.0
                rot_scale = 0.4 if dstick.getTrigger() else 0.3
                # True means allow turn in place
                self.myRobot.curvatureDrive(
                    dstick.getX() * rot_scale, dstick.getY() * speed_scale, True)

        else:
            self.myRobot.curvatureDrive(dstick.getLeftX(), dstick.getLeftY(), True)


    def testInit(self):
        self.state = 'test'

    def testExit(self):
        self.state = 'between'


def git_desc():
    '''Get the git description e.g. branchX-12df1a1-dirty'''
    try:
        import subprocess as subp
        desc = subp.getoutput('git describe --always --dirty')
    except Exception:
        desc = '(git?)'
    return desc


if __name__ == "__main__":
    DEPLOY_INFO = wpilib.deployinfo.getDeployData() or {
        'git-desc': git_desc(),
        }

    wpilib.run(MyRobot)
