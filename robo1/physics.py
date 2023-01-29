#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

from wpimath.geometry import Transform3d as T3D
import wpilib.simulation

import photonvision as pv
import robotpy_apriltag as at

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel, drivetrains
from pyfrc.physics.units import units

import constants as C

USE_TANK_MODEL = True


SIMCAM = dict(
    camName = C.cam1Name, # Name of the PhotonVision camera to create.
    camDiagFOV = 170,     # Diagonal Field of View of the camera used.
    cameraToRobot = C.cam1ToRobot,# Transform to move from the camera's mount position to robot.
    maxLEDRange = 9000,   # docs say use 9000 on cameras without LEDs
    cameraResWidth = 480, # Width of your camera's image sensor in pixels
    cameraResHeight = 360,# Height of your camera's image sensor in pixels
    minTargetArea = 10,   # Minimum area for recognized targets in pixels
    # TODO: verify that this is total pixels in the blob, not 50x50 pixels
    )


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):

        global P
        P = self    # for easier inspection via python prompt

        self.physics = physics_controller

        self.cam1 = pv.SimVisionSystem(**SIMCAM)

        layout = at.loadAprilTagLayoutField(at.AprilTagField.k2023ChargedUp)
        for i in range(1, 9):
            pose = layout.getTagPose(i)
            tag = self.physics.field.getObject(f'tag{i}')
            tag.setPose(pose.toPose2d())

            target = pv.SimVisionTarget(pose, C.tagsize.m, C.tagsize.m, i)
            self.cam1.addSimVisionTarget(target)
            # breakpoint()

        # Motors
        self.l1 = wpilib.simulation.PWMSim(robot.left1.getChannel())
        self.r1 = wpilib.simulation.PWMSim(robot.right1.getChannel())
        self.l2 = wpilib.simulation.PWMSim(robot.left2.getChannel())
        self.r2 = wpilib.simulation.PWMSim(robot.right2.getChannel())

        # self.dio1 = wpilib.simulation.DIOSim(robot.limit1)
        # self.dio2 = wpilib.simulation.DIOSim(robot.limit2)
        # self.ain2 = wpilib.simulation.AnalogInputSim(robot.position)

        # self.motor = wpilib.simulation.PWMSim(robot.motor.getChannel())

        self.accel = wpilib.simulation.BuiltInAccelerometerSim()

        # Gyro
        # self.gyro = wpilib.simulation.AnalogGyroSim(robot.gyro)

        self.position = 0

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        if USE_TANK_MODEL:
            self.drivetrain = tankmodel.TankModel.theory(
                motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
                110 * units.lbs,                    # robot mass
                10.71,                              # drivetrain gear ratio
                2,                                  # motors per side
                22 * units.inch,                    # robot wheelbase
                23 * units.inch + bumper_width * 2, # robot width
                32 * units.inch + bumper_width * 2, # robot length
                6 * units.inch,                     # wheel diameter
            )
        else:
            # not well tested and doesn't seem to implement momentum
            # as the TankModel does, and besides that it basically
            # has the same behaviour in terms of inversion of one side
            # relative to the real robot, maybe
            self.drivetrain = drivetrains.FourMotorDrivetrain(
                x_wheelbase = 22 * units.inch,
                speed = 2 * units.meter_per_second,
                deadzone = drivetrains.linear_deadzone(0.2)
                )


    _prevt = _prevp = 0

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l1 = self.l1.getSpeed()
        l2 = self.l2.getSpeed()
        r1 = -self.r1.getSpeed()
        r2 = -self.r2.getSpeed()
        # assert l1 == l2
        # assert r1 == r2

        if USE_TANK_MODEL:
            transform = self.drivetrain.calculate(l1, r2, tm_diff)
            pose = self.physics.move_robot(transform)

        else:
            speeds = self.drivetrain.calculate(l1, l2, r1, r2)
            pose = self.physics.drive(speeds, tm_diff)

        self.cam1.processFrame(self.physics.get_pose())

        # pt = f'\tl={l1:.1f} r={r1:.1f} x={pose.x:4.1f} y={pose.y:4.1f} rot={pose.rotation().degrees():.0f}'
        # if pt != self._prevp and now - self._prevt > 0.5:
        #     self._prevt = now
        #     self._prevp = pt
        #     print(pt)

        # # Update the gyro simulation
        # # -> FRC gyros are positive clockwise, but the returned pose is positive
        # #    counter-clockwise
        # self.gyro.setAngle(-pose.rotation().degrees())

        # update position (use tm_diff so the rate is constant)
        # self.position += self.motor.getSpeed() * tm_diff * 3

        # # update limit switches based on position
        # if self.position <= 0:
        #     switch1 = True
        #     switch2 = False

        # elif self.position > 10:
        #     switch1 = False
        #     switch2 = True

        # else:
        #     switch1 = False
        #     switch2 = False

        # set values here
        # self.dio1.setValue(switch1)
        # self.dio2.setValue(switch2)
        # self.ain2.setVoltage(self.position)
