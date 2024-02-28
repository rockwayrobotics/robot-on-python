#!/usr/bin/env python3
#

import wpilib

import commands2

from robotcontainer import RobotContainer


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()

        # Get a shortcut to the command scheduler run() method.
        self.cmds = commands2.CommandScheduler.getInstance()


    #----------------------------------
    # This function is called every 20 ms, no matter the mode. Use this for
    # items like diagnostics that you want run during disabled, autonomous,
    # teleoperated and test.
    #
    # This runs after the mode specific periodic functions, but before LiveWindow and
    # SmartDashboard integrated updating.
    def robotPeriodic(self):
        # Runs the Scheduler.  This is responsible for polling buttons, adding
        # newly-scheduled commands, running already-scheduled commands, removing
        # finished or interrupted commands, and running subsystem periodic()
        # methods.  This must be called from the robot's periodic block in order
        # for anything in the Command-based framework to work.
        self.cmds.run()


    #----------------------------------
    def disabledInit(self):
        self.cmds.cancelAll()


    def disabledExit(self):
        pass


    def disabledPeriodic(self):
        pass


    #----------------------------------
    def autonomousInit(self):
        self.cmds.cancelAll()


    def autonomousExit(self):
        pass


    def autonomousPeriodic(self):
        pass


    #----------------------------------
    def teleopInit(self):
        pass


    def teleopExit(self):
        pass


    def teleopPeriodic(self):
        pass
        # # We can read the distance in millimeters
        # distanceMillimeters = self.rangeFinder.getRangeMM()
        # # ... or in inches
        # distanceInches = self.rangeFinder.getRangeInches()

        # # We can also publish the data itself periodically
        # SmartDashboard.putNumber("Distance[mm]", distanceMillimeters)
        # SmartDashboard.putNumber("Distance[in]", distanceInches)

        # # Drive with tank drive.
        # # That means that the Y axis of the left stick moves the left side
        # # of the robot forward and backward, and the Y axis of the right stick
        # # moves the right side of the robot forward and backward.
        # self.robotDrive.tankDrive(
        #     -self.driverController.getLeftY(), -self.driverController.getRightY()
        # )


    #----------------------------------
    def testInit(self):
        self.cmds.cancelAll()

        # # By default, the Ultrasonic class polls all ultrasonic sensors every in a round-robin to prevent
        # # them from interfering from one another.
        # # However, manual polling is also possible -- notes that this disables automatic mode!
        # self.rangeFinder.ping()


    def testExit(self):
        self.cmds.cancelAll()

        # # Enable automatic mode
        # self.rangeFinder.setAutomaticMode(True)


    def testPeriodic(self):
        pass
        # if self.rangeFinder.isRangeValid():
        #     # Data is valid, publish it
        #     SmartDashboard.putNumber("Distance[mm]", self.rangeFinder.getRangeMM())
        #     SmartDashboard.putNumber("Distance[in]", self.rangeFinder.getRangeInches())

        #     # Ping for next measurement
        #     self.rangeFinder.ping()
