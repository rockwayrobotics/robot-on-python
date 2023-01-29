#!/usr/bin/env python3
"""Experimental RobotPy for FRC 2023."""

# import warnings
# warnings.filterwarnings('ignore')

# import ntcore

import wpilib
from wpilib.drive import DifferentialDrive
from wpilib.shuffleboard import Shuffleboard

from wpimath.geometry import Pose2d, Pose3d

from pyfrc.physics.units import units

# Note: could move this below so we do it only in normal mode, allowing
# tests/sim to run faster if they don't require this.
import ctre
import rev
import photonvision as pv
import robotpy_apriltag as at

from constants import * # original code used this... get rid of it
import constants as C   # this is the better way... less namespace pollution


DASH = wpilib.SmartDashboard
DS = wpilib.DriverStation
PREFS = wpilib.Preferences

DRIVE = 'curvature'
BREAK = False


# In simulation, for some reason we currently have to negate the
# rotation argument for both of these drive types. This hot-patches
# the drive instance to replace both methods with ones that
# call the original methods with the rotation negated.
# see https://robotpy.readthedocs.io/projects/wpilib/en/stable/wpilib.drive/DifferentialDrive.html
# Normally you'd just hot-patch one or two methods, but the
# class involved is "sealed" or something to prevent doing so, which is why
# I've had to implement every one and delegate to the original in such a
# cumbersome manner. There is a way to do this automatically but it's a
# wee bit of work, so leaving that till some other time...
class SimDrive:
    def __init__(self, drive):
        self.drive = drive

    # From class MotorSafety
    def check(self): return self.drive.check()
    def checkMotors(self, *args, **kwargs): return self.drive.checkMotors(*args, **kwargs)
    def feed(self, *args, **kwargs): return self.drive.feed(*args, **kwargs)
    def getDescription(self): return self.drive.getDescription()
    def getExpiration(self, *args, **kwargs): return self.drive.getExpiration(*args, **kwargs)
    def isAlive(self): return self.drive.isAlive()
    def isSafetyEnabled(self): return self.drive.isSafetyEnabled()
    def setExpiration(self, *args, **kwargs): return self.drive.setExpiration(*args, **kwargs)
    def setSafetyEnabled(self, x): return self.drive.setSafetyEnabled(x)
    def stopMotor(self): return self.drive.stopMotor()

    # from class RobotDriveBase
    MotorType = DifferentialDrive.MotorType
    def feedWatchdog(self, *args, **kwargs): return self.drive.feedWatchdog(*args, **kwargs)
    def setDeadband(self, *args, **kwargs): return self.drive.setDeadband(*args, **kwargs)
    def setMaxOutput(self, x): return self.drive.setMaxOutput(x)

    # from class DifferentialDrive
    WheelSpeeds = DifferentialDrive.WheelSpeeds
    def arcadeDrive(self, v, r, sqi=True): return self.drive.arcadeDrive(v, -r, sqi)
    def curvatureDrive(self, v, r, tip=True): return self.drive.curvatureDrive(v, -r, tip)
    def initSendable(self, *args, **kwargs): return self.drive.initSendable(*args, **kwargs)
    def tankDrive(self, lv, rv, sqi=True): return self.drive.tankDrive(lv, rv, sqi)
    def tankDriveIK(self, lv, rv, sqi=True): return self.drive.tankDriveIK(lv, rv, sqi)
    # These are untested, and I don't know exactly what they do, so I'm not sure
    # whether or not we should also be negating rotation here. Doing so for now.
    def arcadeDriveIK(self, v, r, sqi=True): return self.drive.arcadeDriveIK(v, -r, sqi)
    def curvatureDriveIK(self, v, r, tip=True): return self.drive.curvatureDriveIK(v, -r, tip)



class MyRobot(wpilib.TimedRobot):
    state = 'init'

    def buildDriveMotors(self):
        '''Create and return the drive motors for sim or normal mode.'''
        if self.sim:
            left1 = wpilib.PWMSparkMax(kLeftMotor1)
            left2 = wpilib.PWMSparkMax(kLeftMotor2)
            right1 = wpilib.PWMSparkMax(kRightMotor1)
            right2 = wpilib.PWMSparkMax(kRightMotor2)
        else:
            brushed = rev.CANSparkMax.MotorType.kBrushed
            # TODO: for the 2023 drive base we'll currently have
            # four *kBrushless* CANSparkMax motors, per
            # https://github.com/rockwayrobotics/FRC-2023/commit/f9c2bc2#diff-8a9b484

            # TODO: make this auto-configure based on Rio serial number,
            # MAC address, or other identifier, based on which types of
            # controller are in which drive base.  Currently the 2022 robot
            # has WPI_VictorSPX as the rear motors, and CANSparkMax as
            # the front ones (at least, that's what the FRC-2023 code shows,
            # but the wiring suggests the Victors are the front ones).
            # The 2023 drive base has all four as CANSparkMax ... as of now.
            left1 = ctre.WPI_VictorSPX(kLeftMotor1)
            left2 = rev.CANSparkMax(kLeftMotor2, brushed)

            right1 = ctre.WPI_VictorSPX(kRightMotor1)
            # Currently this was necesary on the old drive base to resolve
            # the two right motors fighting against each other... we have
            # not checked the wiring or otherwise tried to find the root cause.
            right1.setInverted(True)

            right2 = rev.CANSparkMax(kRightMotor2, brushed)

        return (left1, left2, right1, right2)


    def buildStick(self, sim=False):
        '''Routine create a control device.  This allows us to
        substitute a simulated one when appropriate, etc.  The current
        code doesn't really do that.'''
        if sim:
            return wpilib.Joystick(kSimStick)
        else:
            return wpilib.XboxController(kXbox)


    def setupVision(self):
        # shared with physics... TODO: make util modules for such things
        layout = at.loadAprilTagLayoutField(at.AprilTagField.k2023ChargedUp)

        self.cam1 = pv.PhotonCamera(C.cam1Name)
        self.cam1.setVersionCheckEnabled(False)

        # Python doesn't have ArrayLists, so pybind makes us provide the last param as such.
        self.poseEstimator = pv.RobotPoseEstimator(
            layout,
            pv.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            [(self.cam1, C.cam1ToRobot)],
        )

        # for i in range(1, 9):
        #     pose = layout.getTagPose(i)
            # tag.setPose(pose.toPose2d())


    def getEstimatedGlobalPose(self, prevEstimate: Pose3d):
        self.poseEstimator.setReferencePose(Pose3d(pose=prevEstimate.toPose2d()))
        return self.poseEstimator.update()


    def robotInit(self):
        """Robot initialization function"""
        self.sim = self.isSimulation()

        self.setupVision()
        self.globalPose = Pose3d()

        # need these stored so the simulation can find them (physics.py)
        self.left1, self.left2, self.right1, self.right2 = self.buildDriveMotors()

        self.left = wpilib.MotorControllerGroup(self.left1, self.left2)
        self.right = wpilib.MotorControllerGroup(self.right1, self.right2)

        self.dio4 = wpilib.DigitalInput(4)
        self.dio5 = wpilib.DigitalInput(5)

        self.drive = DifferentialDrive(self.left, self.right)
        # self.drive.setExpiration(0.1)

        # Fix simulation by negating rotation parameter... see SimDrive above
        if self.sim:
            self.drive = SimDrive(self.drive)

        self.simStick = self.buildStick(sim=True)
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


    def updateDashboard(self):
        DASH.putString('State', self.state)

        axes = [self.accel.getX(), self.accel.getY(), self.accel.getX()]
        # axes = ','.join(f'{k}={v:.2f}' for k,v in zip('xyz', axes))
        # DASH.putString('accel', axes)
        DASH.putNumberArray('accel', axes)

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

        DASH.putNumber('batt', DS.getBatteryVoltage())
        # DASH.putString('alliance', 'blue' if DS.getAlliance() else 'red')

        DASH.putBoolean('DIO 4', self.dio4.get())
        DASH.putBoolean('DIO 5', self.dio5.get())

        # with warnings.catch_warnings():
        #     warnings.simplefilter('ignore')
        if True:
            try:
                pose, latency = self.getEstimatedGlobalPose(self.globalPose)
                if True: #pose != self.globalPose:
                    self.globalPose = pose
                    DASH.putString('pose', f'{pose.x:.2f},{pose.y:.2f} {pose.rotation().z_degrees:.0f}')
                    DASH.putNumber('latency', latency)
            except Exception as ex:
                print(ex)


    def robotPeriodic(self):
        self.updateDashboard()


    def disabledInit(self):
        self.state = 'disabled'
        print('state: disabled')
        self.drive.arcadeDrive(0, 0)


    def disabledPeriodic(self):
        pass


    def autonomousInit(self):
        self.state = 'auto'
        print('state: auto')
        if not self.sim:
            self.drive.setSafetyEnabled(True)
        self.phase = self.run_phases()


    def autonomousExit(self):
        self.state = 'between'


    # crude substitute for Command stuff
    PHASES = [
        (1.0, 'initial'),
        (3.0, 'curve_out'),
        (1.1, 'pivot_left'),
        (1.5, 'straight'),
        (3.1, 'back_right'),
        (1.8, 'zoom'),
        (2.4, 'curve_in'),
        (2.2, 'realign'),
        ]

    def _phase_initial(self):       self.drive.arcadeDrive( 0.00,  0.00, False)
    def _phase_curve_out(self):     self.drive.arcadeDrive( 0.80, -0.03, False)
    def _phase_pivot_left(self):    self.drive.arcadeDrive( 0.50, -0.15, False)
    def _phase_straight(self):      self.drive.arcadeDrive( 1.00,  0.00, False)
    def _phase_back_right(self):    self.drive.arcadeDrive(-0.50, -0.04, False)
    def _phase_zoom(self):          self.drive.arcadeDrive( 1.00,  0.00, False)
    def _phase_curve_in(self):      self.drive.arcadeDrive( 0.50, -0.05, False)
    def _phase_realign(self):       self.drive.arcadeDrive( 0.00,  0.25, False)

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


    def teleopExit(self):
        self.state = 'between'


    def teleopPeriodic(self):
        '''Runs the motors with X steering (arcade, tank, curvature)'''
        # TODO: make a class to delegate more cleanly to a joystick configured
        # appropriately for sim or normal mode, so we can use common code here
        dstick = self.simStick
        speed_scale = 0.7 if dstick.getTrigger() else 1.0
        rot_scale = 0.4 if dstick.getTrigger() else 0.3

        if DRIVE == 'arcade':
            # if dstick.getTop():
            #     breakpoint()
            # last arg True mean square inputs (higher sensitivity at low values)
            self.drive.arcadeDrive(-dstick.getY() * speed_scale, dstick.getX() * rot_scale)

        elif DRIVE == 'curvature':
            # last arg True means allow turn in place
            self.drive.curvatureDrive(
                -dstick.getY() * speed_scale, dstick.getX() * rot_scale, True)

        elif DRIVE == 'tank':
            self.drive.tankDrive(
                -dstick.getLeftY() * speed_scale, dstick.getRightY() * speed_scale)


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
