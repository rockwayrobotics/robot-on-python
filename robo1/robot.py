#!/usr/bin/env python3
"""Experimental RobotPy for FRC 2023."""

import asyncio
from enum import Enum
import logging
import re
import threading
import time
# import ntcore

import wpilib
from wpilib.drive import DifferentialDrive
from wpilib.interfaces import GenericHID
from wpilib.shuffleboard import Shuffleboard
from wpimath.controller import PIDController
from wpimath.filter import LinearFilter

# Note: could move this below so we do it only in normal mode, allowing
# tests/sim to run faster if they don't require this.
import ctre
import rev

from cli import CLI
from constants import *

DASH = wpilib.SmartDashboard
DS = wpilib.DriverStation
PREFS = wpilib.Preferences

BREAK = False

class Drive(Enum):
    ARCADE = 1
    CURVATURE = 2
    TANK = 3

DRIVE = Drive.ARCADE


# wpilib.interfaces.GenericHID
# instantiate with port #
# isConnected() true/false
# query getName(), getAxisCount(), getButtonCount()
# Logitech Extreme 3D: 4 axes, 12 buttons
#   0 is x, y is 1, twist is 2 (z), sensitivity is 3 (twist), throttle 4
#   getAxisType(n) returns 0/1 for X/Y, 5 for twist? 6 for sensitivity adjust
#   getType is HIDType.kHIDJoystick: 20
# Keyboard 0: 3 axes, 4 buttons, type is 20
#   axes: type 0 (0, 1, 2)
# Bluetooth LE XINPUT compatible input device:
#   axes: 6, types 0,1,2,3,4,5
#   buttons: 16
#   type: HIDType.kXInputGamepad: 1
#   POV: 1, returns -1 in centre, else 0, 45, 90 etc up to 315 clockwise
# "Wireless Controller" (DualShock 4) from PS4:
#   axes: 6, all type 0
#       x/y: left stick is 0/1, right is 2/5
#       left trigger is 3, right is 4
#   buttons: 14
#   type: HIDType.kHIDGamepad: 21
#   POV: 1, the 4 left-pad buttons, which can also be pressed in adjacent pairs

class Controllers:
    '''Adaptive controller class, allowing the robot to adjust dynamically
    to removal/insertion of different controller types.'''

    def __init__(self, robot):
        # self.mode = 1   # 2 for two sticks (arcade drive)
        self.robot = robot
        self.log = logging.getLogger('ctrl')
        self.hid0 = GenericHID(0)
        self._name0 = None  # force initial change even if nothing attached
        self.check()

    def isConnected(self):
        return bool(self._name0)


    def switch_xbox(self, c):
        '''Map methods to Xbox-style controllers.'''
        self.c0 = c
        self.robot.getX = c.getLeftX
        self.robot.getY = c.getLeftY
        self.robot.getTop = c.getTop
        # self.robot.getTrigger = c.getTrigger
        self.robot.getPOV = c.getPOV


    def switch_dualshock(self, c):
        '''Map methods to DualShock (PlayStation) controllers.'''
        self.c0 = c
        self.robot.getX = c.getLeftX
        self.robot.getY = c.getLeftY
        self.robot.getTop = c.getCrossButton    # or Square/Triangle/Circle
        # self.robot.getTrigger = c.get???
        self.robot.getPOV = c.getPOV


    def switch_joystick(self, c):
        '''Map methods to generic joystick controllers including sim keypad.'''
        self.c0 = c
        self.robot.getX = c.getX
        self.robot.getY = c.getY
        self.robot.getTop = c.getTop
        # self.robot.getTrigger = c.getTrigger
        self.robot.getPOV = c.getPOV


    def adapt(self):
        self.reset()
        name = self._name0.lower()
        if re.match(r'.*(xbox|xinput)', name):
            self.switch_gamepad(wpilib.XboxController(0))
        elif re.match(r'wireless controller', name):
            self.switch_dualshock(wpilib.PS4Controller(0))
        elif self._name0:   # anything else connected?
            self.switch_joystick(wpilib.Joystick(0))
        else:
            self.log.warning('controller 0 missing')

        if self._name0:
            self.log.info('switched to %s', self._name0)


    def reset(self):
        '''Reset to null controller, to ensure we don't mix methods from
        a previous and a new controller.'''
        self.robot.getX = lambda: 0.0
        self.robot.getY = lambda: 0.0
        self.robot.getTop = lambda: False
        self.robot.getTrigger = lambda: False
        self.robot.getPOV = lambda x: -1


    def check(self):
        '''Check if controller(s) attached have changed, and adapt.'''
        name = self.hid0.getName()
        if name != self._name0:
            self._name0 = name
            self.adapt()



class MyRobot(wpilib.TimedRobot):
    state = 'init'


    def buildDriveMotors(self):
        '''Create and return the drive motors for sim or normal mode.'''
        if self.sim:
            lf, lr, rf, rr = (wpilib.PWMSparkMax(x) for x in kMotors)
        else:
            brushed = rev.CANSparkMax.MotorType.kBrushed

            lr = ctre.WPI_VictorSPX(kLeftRear)
            rr = ctre.WPI_VictorSPX(kRightRear)

            lf = rev.CANSparkMax(kLeftFront, brushed)
            rf = rev.CANSparkMax(kRightFront, brushed)

        # TODO: make this auto-configure based on Rio serial number,
        # MAC address, or other identifier, based on which types of
        # controller are in which drive base.  Currently the 2022 robot
        # has WPI_VictorSPX as the rear motors, and CANSparkMax as
        # the front ones (at least, that's what the FRC-2023 code shows,
        # but the wiring suggests the Victors are the front ones).
        # The 2023 drive base has all four as CANSparkMax ... as of now.
        # >>> Path('/etc/machine-id').read_text()
        # '095c5d5cb0544228a51ce3ac2b33a92d\n'

        # lf.setInverted(True)
        # rf.setInverted(True)

        self.lf = lf
        self.lr = lr
        self.rf = rf
        self.rr = rr

        # Critical: keep these in this order so the POV stuff works, below.
        self.motors = (self.lf, self.rf, self.rr, self.lr)

        self.setup_inversion()


    def setup_inversion(self):
        # On Igneous: positive speed (i.e. pass negated Y axis)
        # LF: reverses
        # LR: forward
        # RF: forward, but with right-side inversion that's
        # RR: reverse
        self.lf.setInverted(True)
        self.lr.setInverted(False)
        self.rf.setInverted(True)
        self.rr.setInverted(False)


    def clear_inversion(self):
        '''Clear current motor (not group) inversions, so that we can
        do "raw" motor control when a POV direction is used to select one.'''
        self.lf.setInverted(False)
        self.lr.setInverted(False)
        self.rf.setInverted(False)
        self.rr.setInverted(False)


    def buildEncoders(self):
        enc1 = wpilib.Encoder(kLeftEncoder1, kLeftEncoder2)
        enc1.setDistancePerPulse(DISTANCE_PER_ENCODER_PULSE)
        enc2 = wpilib.Encoder(kRightEncoder1, kRightEncoder2)
        enc2.setDistancePerPulse(-DISTANCE_PER_ENCODER_PULSE)
        return (enc2, enc1)


    def robotInit(self):
        """Robot initialization function"""
        self.log = logging.getLogger('robot')

        self.sim = self.isSimulation()

        # self.buildSticks()
        self.ctrl = Controllers(robot=self)

        self.el, self.er = self.buildEncoders()
        self.buildDriveMotors()

        self.left = wpilib.MotorControllerGroup(self.lf, self.lr)
        self.right = wpilib.MotorControllerGroup(self.rf, self.rr)
        # It's a convention that on FRC robots the right side turns in reverse
        # when given a positive speed, so typically the right motor group is
        # inverted to account for that, allowing both sides to be treated the same.
        self.right.setInverted(True)

        self.dio4 = wpilib.DigitalInput(4)
        self.dio5 = wpilib.DigitalInput(5)

        self.driveMode = Drive.ARCADE

        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)
        self.drive.setSafetyEnabled(not self.sim)

        self.pov = False

        self.pid = PIDController(1.0, 0.0, 0.05)
        self.filter = LinearFilter.singlePoleIIR(timeConstant=0.4, period=0.02)
        self.speed = self.rotation = 0

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

        # print(f'stick: {self.stick1.isConnected()}')
        self.dash_gen = self.dash_generator()

        self.startCLI()


    def startCLI(self):
        '''Run a Command Line Interface that's accessible via telnet to port 13501.
        This allows you to connect into the "live" robot and inspect/tamper
        with the code on the fly. Can't do that easily with another language! :)
        '''
        context = dict(
            r=self,         # robot, shows up in dir() as the 'r' item
            g=globals(),
            )

        async def run_cli():
            cli = await CLI.start_server(locals=context)
            await cli.wait_closed()

        t = threading.Thread(target=asyncio.run, args=(run_cli(),), daemon=True)
        t.start()


    def dash_generator(self):
        '''This is structured as a generator for performance by making it
        possible to step through updates in groups rather than updating
        everything on each cycle.  Although this was done originally to try
        solving some periodic hesitations as the robot drove, and it turned
        out that was caused by running ShuffleBoard on the driver computer
        (it's a very heavy-weight program apparently), this has been left
        here as it's probably a good idea in general.'''
        while True:
            DASH.putString('State', self.state)
            axes = ','.join(f'{k}={v:.2f}' for k,v in zip('xyz', (self.accel.getX(), self.accel.getY(), self.accel.getX())))
            DASH.putString('accel', axes)
            yield 2

            # if self.simStick.isConnected():
            #     text = f'x={self.simStick.getX():.2f} y={self.simStick.getY():.2f}'
            #     DASH.putString('joy', text)
            # else:
            #     DASH.putString('joy', 'missing')

            if self.ctrl.isConnected():
                # text = f'x={self.driveStick.getLeftX():.2f} y={self.driveStick.getLeftY():.2f}'
                text = f'l={self.getY():.2f} r={self.getX():.2f}'
                DASH.putString('joy', text)
            else:
                DASH.putString('joy', 'missing')
            yield 3

            DASH.putNumber('batt', DS.getBatteryVoltage())
            DASH.putBoolean('DIO 5', self.dio4.get())
            DASH.putBoolean('DIO 6', self.dio5.get())
            DASH.putNumberArray('Encoders', [self.el.getRate(), self.er.getRate()])
            yield 8


    def arcadeDrive(self, s, r):
        '''This collects the speed/rotation setpoints, but does not directly
        adjust the motors. Instead, the robotPeriodic() method will do that
        by feeding these values to our filter(s), to smooth out abrupt changes.'''
        # if s != self.speed or r != self.rotation:
        #     print(f'drive s={s:.2f} r={r:.2f}')
        self.speed = s
        self.rotation = r
        # self.driveMode = Drive.ARCADE
        # self.pid.setSetpoint(self.speed)


    prev = 0

    def updateDrive(self):
        '''Called after everything else has finished in each cycle, to update
        the actual motor setpoints.  This allows automatically inserting
        linear filters to smooth out jerkiness.'''

        match self.driveMode:
            case Drive.ARCADE:
                rate = (self.el.getRate() + self.er.getRate()) / 2
                speed = self.filter.calculate(self.speed)
                # speed = self.pid.calculate(rate)
                # True mean square inputs (higher sensitivity at low values)
                self.drive.arcadeDrive(speed, self.rotation, False)

                # delta = time.time() - self.prev
                # if delta >= 0.075:
                #     self.prev += delta
                #     if speed > 0.01:
                #         print(f'setpoint {self.speed:.2f} -> speed {speed:.2f}, rate {rate:.2f}, rot {self.rotation:.2f}')

            # case Drive.CURVATURE:
            #     speed = self.pid.calculate(self.speed)
            #     # True means allow turn in place. With False you can't turn at
            #     # all while stopped.
            #     self.drive.curvatureDrive(speed, self.rotation, False)

            # case Drive.TANK:
            #     self.drive.tankDrive(-self.getLeftY(), -self.getRightY())
            case _:
                pass


    def robotPeriodic(self):
        '''Runs after the mode-specific Periodic methods.'''

        # If a POV is selected, we do not control the motors through the
        # DifferentialDrive but do raw motor speed control.
        if not self.pov:
            start = time.time()
            self.updateDrive()
            elapsed1 = time.time() - start
            if elapsed1 >= 0.02:
                print('updateDrive', elapsed1)

            start = time.time()
            stage = next(self.dash_gen)
            elapsed2 = time.time() - start
            if elapsed2 >= 0.02:
                print('dash_gen', elapsed2, stage)

        self.ctrl.check()   # check if controllers have changed


    def disabledInit(self):
        self.state = 'disabled'
        print('state: disabled')
        self.arcadeDrive(0, 0)
        self.drive.arcadeDrive(0, 0)


    # def disabledPeriodic(self):
    #     pass


    def autonomousInit(self):
        self.state = 'auto'
        print('state: auto')
        # Note that run_phases() is a generator, so this instantiates it but
        # does not actually run it.  It's run in phases by calling next()
        # on it repeatedly.  The way a generator works is roughly that it
        # does not "return" in the usual way, but can yield control at
        # various points, preserving its local state and resuming from where
        # it yielded the next time next() is called on it.
        self.phase = self.run_phases()

        # print(f'stick: {self.stick1.isConnected()}')


    def autonomousExit(self):
        self.state = 'between'


    # Crude substitute for Command stuff.  This is not intended as
    # a real example of how we should do this, but was just a quick and dirty
    # home-brewed approach to avoid having to learn the whole Command API
    # at the same time as everything else.
    PHASES = [
        (2.0, 'curve_out'),
        (1.1, 'pivot_left'),
        (1.2, 'straight'),
        (2.3, 'back_right'),
        (1.0, 'zoom'),
        (3.2, 'curve_in'),
        (2.5, 'realign'),
        (2.0, 'backup'),
        ]

    # With the existing values, the real Igneous drive base will do a big
    # loop over a roughly 15' square area, returning nearly to its original
    # position.  It's not particularly accurate so can end up maybe 5' off
    # and not necessarily pointing the same direction either.
    def _phase_initial(self):       self.arcadeDrive( 0.00,  0.00)
    def _phase_curve_out(self):     self.arcadeDrive( 0.80,  0.04)
    def _phase_pivot_left(self):    self.arcadeDrive( 0.50,  0.17)
    def _phase_straight(self):      self.arcadeDrive( 1.00,  0.00)
    def _phase_back_right(self):    self.arcadeDrive(-0.50,  0.08)
    def _phase_zoom(self):          self.arcadeDrive( 1.00,  0.00)
    def _phase_curve_in(self):      self.arcadeDrive( 0.90,  0.12)
    def _phase_realign(self):       self.arcadeDrive( 0.07, -0.40)
    def _phase_backup(self):        self.arcadeDrive(-0.25,  0.00)

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
            self.arcadeDrive(0, 0)


    def teleopInit(self):
        self.state = 'teleop'
        print('state: teleop')
        # self.logger.info('batt: %s', self.getBatteryVoltage())
        # data = wpilib.DriverStation.getGameSpecificMessage()
        # if data:
        #     # Set the robot gamedata property and set a network tables value
        #     self.gameData = data
        #     DASH.putString("gameData", self.gameData)

        # print(f'stick: {self.stick1.isConnected()}')


    def teleopExit(self):
        self.state = 'between'


    # map POV to motor index in self.motors
    # This is used to allow direct/raw control of individual motors, to assist
    # with troubleshooting inversion and control problems.
    POV = {
        315: 0, 0: 0,
        45: 1, 90: 1,
        135: 2, 180: 2,
        225: 3, 270: 3,
        }

    def teleopPeriodic(self):
        """Runs the motors with X steering (arcade, tank, curvature)"""
        pov = self.getPOV(0)
        if pov >= 0:    # POV selected, so do direct motor control
            if not self.pov:
                self.pov = True
                self.clear_inversion()
                self.log.info('POV, clear inversion')
                self.left.disable()
                self.right.disable()
                self.drive.setSafetyEnabled(False)

            speed = -self.getY()
            indices = {0,1,2,3}
            try:
                i = self.POV[pov]
            except KeyError:
                pass
            else:
                # zero out other motors, then set the selected one
                for j in indices - {i}:
                    self.motors[j].set(0)
                self.motors[i].set(speed)
                self.log.debug(f'motor {i} = {speed:.3f}')

        else:   # POV not selected, so switch back to regular drive control
            if self.pov:
                self.pov = False
                for m in self.motors:
                    m.set(0)
                self.setup_inversion()
                self.drive.setSafetyEnabled(not self.sim)
                self.log.info('restore normal operation')

            # Pressing the "top" button reduces the speeds so we can get more
            # fine-grained control.
            speed_scale = 0.5 if self.getTop() else 1.0
            rot_scale = 0.3 if self.getTop() else 0.6

            # True mean square inputs (higher sensitivity at low values)
            self.arcadeDrive(-self.getY() * speed_scale,
                -self.getX() * rot_scale)
            # FIXME: figure out why we have to negate the X axis in order
            # to get the correct turning direction. This doesn't appear
            # to match any documentation or the actual arrangement of the
            # motors etc.
            # Update: it actually does match the Java docs, and the Python
            # docs for the arcadeDrive() routine, but NOT for the arcadeDriveIK()
            # routine. It's unclear yet why the 2023 drive base does not appear
            # to require the same thing, since it seems to be using positive
            # X to go clockwise instead of negative X as this does.

                # case Drive.CURVATURE:
                #     # True means allow turn in place. With False you can't turn at
                #     # all while stopped.
                #     self.drive.curvatureDrive(
                #         -self.getLeftY() * rot_scale, -self.getLeftX() * speed_scale, True)

                # case Drive.TANK:
                #     self.drive.tankDrive(-self.getLeftY(), -self.getRightY())


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
