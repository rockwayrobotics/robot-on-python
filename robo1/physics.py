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

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, drivetrains, tankmodel
from pyfrc.physics.units import units

import time
import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot

from constants import *


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):

        self.physics_controller = physics_controller

        # Motors
        self.lr = wpilib.simulation.PWMSim(kLeftRear)
        self.lf = wpilib.simulation.PWMSim(kLeftFront)
        self.rr = wpilib.simulation.PWMSim(kRightRear)
        self.rf = wpilib.simulation.PWMSim(kRightFront)

        self.el = wpilib.simulation.EncoderSim.createForChannel(kLeftEncoder1)
        self.er = wpilib.simulation.EncoderSim.createForChannel(kRightEncoder1)

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

        # # fmt: off
        # self.drivetrain = tankmodel.TankModel.theory(
        #     motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
        #     110 * units.lbs,                    # robot mass
        #     10.71,                              # drivetrain gear ratio
        #     1,                                  # motors per side
        #     22 * units.inch,                    # robot wheelbase
        #     23 * units.inch + bumper_width * 2, # robot width
        #     32 * units.inch + bumper_width * 2, # robot length
        #     6 * units.inch,                     # wheel diameter
        #     20 * units.ms,
        # )
        # # fmt: on

        self.drivetrain = drivetrains.FourMotorDrivetrain(
            x_wheelbase = 22 * units.inch,
            speed = 10 * units.feet / units.s,
            deadzone = drivetrains.linear_deadzone(0.02),
            )


    prev_ts = 0

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # start = time.time()
        # Simulate the drivetrain
        lf = -self.lf.getSpeed()
        rf = -self.rf.getSpeed()
        rr = self.rr.getSpeed()
        lr = self.lr.getSpeed()

        self.el.setRate(lf * 100)
        self.er.setRate(-rf * 100)

        speeds = self.drivetrain.calculate(lf, lr, rf, rr)
        pose = self.physics_controller.drive(speeds, tm_diff)
        # pose = self.physics_controller.move_robot(transform)

        # elapsed = time.time() - start
        # print('sim', elapsed)

        # delta = time.time() - self.prev_ts
        # if delta > 0.25:
        #     self.prev_ts += delta
        #     print(f'L={lf:.3f}/{lr:.3f} R={rf:.3f}/{rr:.3f}', speeds)

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
