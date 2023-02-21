#!/usr/bin/env python3
"""Experimental RobotPy for FRC 2023."""

# import ntcore

import wpilib
from wpilib.drive import DifferentialDrive
from wpilib.shuffleboard import Shuffleboard

# Note: could move this below so we do it only in normal mode, allowing
# tests/sim to run faster if they don't require this.
import ctre
import rev

from constants import *

DASH = wpilib.SmartDashboard
DS = wpilib.DriverStation
PREFS = wpilib.Preferences

ARCADE = True
BREAK = False

class MyRobot(wpilib.TimedRobot):
    state = 'init'


    def buildEncoders(self):
        enc1 = wpilib.Encoder(kLeftEncoder1, kLeftEncoder2)
        enc1.setDistancePerPulse(DISTANCE_PER_ENCODER_PULSE)
        enc2 = wpilib.Encoder(kRightEncoder1, kRightEncoder2)
        enc2.setDistancePerPulse(-DISTANCE_PER_ENCODER_PULSE)
        return (enc1, enc2)


    def buildDriveMotors(self):
        '''Create and return the drive motors for sim or normal mode.'''
        if self.sim:
            return [wpilib.PWMSparkMax(x) for x in kMotors]
        else:
            brushed = rev.CANSparkMax.MotorType.kBrushed
            # (kLeftMotor1, kLeftMotor2, kRightMotor1, kRightMotor2)
            left1 = ctre.WPI_VictorSPX(kLeftMotor1)
            right1 = ctre.WPI_VictorSPX(kRightMotor1)
            right1.setInverted(True)
            left2 = rev.CANSparkMax(kLeftMotor2, brushed)
            right2 = rev.CANSparkMax(kRightMotor2, brushed)
            return (left1, left2, right1, right2)


    def buildStick(self, sim=False):
        '''Routine create a control device.  This allows us to
        substitute a simulated one when appropriate, etc.  The current
        code doesn't really do that.'''
        # if sim:
        # return wpilib.Joystick(kSimStick)
        return wpilib.XboxController(kXbox)
        # else:
            # return wpilib.XboxController(kXbox)


    def robotInit(self):
        """Robot initialization function"""
        self.sim = self.isSimulation()

        self.enc1, self.enc2 = self.buildEncoders()
        self.left1, self.left2, self.right1, self.right2 = self.buildDriveMotors()

        self.left = wpilib.MotorControllerGroup(self.left1, self.left2)
        self.right = wpilib.MotorControllerGroup(self.right1, self.right2)

        # # This was in 2022
        # self.left.setInverted(True)
        # self.right.setInverted(True)
        self.dio4 = wpilib.DigitalInput(4)
        self.dio5 = wpilib.DigitalInput(5)

        self.drive = DifferentialDrive(self.left, self.right)
        # self.drive.setExpiration(0.1)

        # self.simStick = self.buildStick(sim=True)
        self.driveStick = self.buildStick()

        self.accel = wpilib.BuiltInAccelerometer()

        self.ds = wpilib.DSControlWord()
        # print('ds attached', self.ds.isDSAttached())
        DASH.putString('git', DEPLOY_INFO.get('git-desc', 'missing'))

        # rotation of the RIO, specified as:
        # 0 = X forward (Y left)
        # 90 = X left (Y back)
        # 180 = X back (Y right)
        # 270 = X right (Y forward)
        # S/N 32363BD has X forward, other one has X left
        PREFS.initInt('rio_rotation', 0)

        # smartTab = Shuffleboard.getTab("Foobar")
        # smartTab.add(title='DIO 5', defaultValue=self.dio4)
        # smartTab.add(title="Potentiometer", defaultValue=self.elevatorPot)

        print(f'stick: {self.driveStick.isConnected()}')


    def updateDashboard(self):
        DASH.putString('State', self.state)

        axes = ','.join(f'{k}={v:.2f}' for k,v in zip('xyz', (self.accel.getX(), self.accel.getY(), self.accel.getX())))
        DASH.putString('accel', axes)

        # if self.simStick.isConnected():
        #     text = f'x={self.simStick.getX():.2f} y={self.simStick.getY():.2f}'
        #     DASH.putString('joy', text)
        # else:
        #     DASH.putString('joy', 'missing')

        # if self.driveStick.isConnected():
        #     # text = f'x={self.driveStick.getLeftX():.2f} y={self.driveStick.getLeftY():.2f}'
        #     text = f'x={self.driveStick.getX():.2f} y={self.driveStick.getY():.2f}'
        #     DASH.putString('xbox', text)
        # else:
        #     DASH.putString('xbox', 'missing')

        DASH.putNumber('batt', DS.getBatteryVoltage())
        # DASH.putString('alliance', 'blue' if DS.getAlliance() else 'red')

        DASH.putBoolean('DIO 5', self.dio4.get())
        DASH.putBoolean('DIO 6', self.dio5.get())

        DASH.putNumber('ENC1', self.enc1.getRate())
        DASH.putNumber('ENC2', self.enc2.getRate())


    def robotPeriodic(self):
        self.updateDashboard()


    def disabledInit(self):
        self.state = 'disabled'
        print('state: disabled')
        self.drive.arcadeDrive(0, 0)


    # def disabledPeriodic(self):
    #     pass


    def autonomousInit(self):
        self.state = 'auto'
        print('state: auto')
        if not self.sim:
            self.drive.setSafetyEnabled(True)
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

    def _phase_initial(self):       self.drive.arcadeDrive(0, 0)
    def _phase_curve_out(self):     self.drive.arcadeDrive(-0.1, -0.7)
    def _phase_pivot_left(self):    self.drive.arcadeDrive(-0.3, -0.5)
    def _phase_straight(self):      self.drive.arcadeDrive(0, -0.80)
    def _phase_back_right(self):    self.drive.arcadeDrive(-0.2, 0.6)

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
            self.drive.arcadeDrive(0, 0)


    def teleopInit(self):
        self.state = 'teleop'
        print('state: teleop')
        self.drive.setSafetyEnabled(not self.sim)
        # self.logger.info('batt: %s', self.getBatteryVoltage())
        # data = wpilib.DriverStation.getGameSpecificMessage()
        # if data:
        #     # Set the robot gamedata property and set a network tables value
        #     self.gameData = data
        #     DASH.putString("gameData", self.gameData)

        print(f'stick: {self.driveStick.isConnected()}')


    def teleopExit(self):
        self.state = 'between'


    def teleopPeriodic(self):
        """Runs the motors with X steering (arcade, tank, curvature)"""
        # TODO: make a class to delegate more cleanly to a joystick configured
        # appropriately for sim or normal mode, so we can use common code here
        # if self.sim:
        #     dstick = self.simStick
        #     if ARCADE:
        dstick = self.driveStick
        speed_scale = 0.7 if dstick.getRightBumper() else 1.0
        rot_scale = 0.0 if dstick.getRightBumper() else 0.6
        # True mean square inputs (higher sensitivity at low values)
        self.drive.arcadeDrive(-dstick.getRightY() * speed_scale,
            -dstick.getRightX() * rot_scale, False)
        #     else:

        #         speed_scale = 0.3 if dstick.getTrigger() else 1.0
        #         rot_scale = 0.4 if dstick.getTrigger() else 0.3
        #         # True means allow turn in place
        #         self.drive.curvatureDrive(
        #             dstick.getX() * rot_scale, dstick.getY() * speed_scale, True)

        # else:
        #     dstick = self.driveStick
        #     self.drive.curvatureDrive(-dstick.getLeftY() * 0.5, dstick.getLeftX() * 0.5, True)


    def testInit(self):
        self.state = 'test'

        if BREAK and self.sim:
            breakpoint()


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

    import sys
    if '--break' in sys.argv:
        BREAK = True
        sys.argv.remove('--break')

    wpilib.run(MyRobot)
