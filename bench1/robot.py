#!/usr/bin/env python3
#
import warnings
# 'module' shows only once per module
# 'once' shows only once ever
# 'ignore' never shows
# 'default' shows only the first time too?
warnings.simplefilter('module')

import wpilib

import commands2

import constants
from robotcontainer import RobotContainer


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()

        # Get a shortcut to the command scheduler run() method.
        self.cmds = commands2.CommandScheduler.getInstance()

        # Must be a PWM header, not MXP or DIO
        self.led = wpilib.AddressableLED(constants.LED.port)

        # LED Data
        length = constants.LED.length

        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(length)]
        # Store what the last hue of the first pixel is
        self.rainbowFirstPixelHue = 0

        # Default to a length of 60, start empty output
        # Length is expensive to set, so only set it once, then just update data
        self.led.setLength(length)

        # Set the data
        self.led.setData(self.ledData)
        self.led.start()

        # ['containsKey', 'getBoolean', 'getDouble', 'getFloat', 'getInt',
        # ['getKeys', 'getLong', 'getString', 'initBoolean', 'initDouble',
        # ['initFloat', 'initInt', 'initLong', 'initString', 'remove',
        # ['removeAll', 'setBoolean', 'setDouble', 'setFloat', 'setInt',
        # ['setLong', 'setString']
        wpilib.Preferences.initInt('led-speed', 5)
        self.led_speed = int(wpilib.Preferences.getInt('led-speed'))
        print('LED speed', self.led_speed)
        self._led_changed = True

        self.pref_pacer = 0

        self.hid = wpilib.interfaces.GenericHID(constants.Gamepads.DRIVER)
        self._hid_sig = self.hid_sig()
        print('HID type', self.hid.getType())
        print('axes', self.hid.getAxisCount())
        print('buttons', self.hid.getButtonCount())
        print('POV', self.hid.getPOVCount())
        print('connected', self.hid.isConnected())
        self._hid_prev = None
        self._hid_pacer = 0


    def hid_sig(self):
        return (self.hid.isConnected(), self.hid.getType())

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

        self.pref_pacer += 1
        if self.pref_pacer > 50:
            self.pref_pacer = 0

            speed = max(1, int(wpilib.Preferences.getInt('led-speed')))
            if speed != self.led_speed:
                self.led_speed = speed
                print('LED speed changed:', speed)

        self._hid_pacer += 1
        if self._hid_pacer >= 5:
            self._hid_pacer = 0
            sig = self.hid_sig()
            if sig != self._hid_sig:
                # self.hid = wpilib.interfaces.GenericHID(constants.Gamepads.DRIVER)
                self._hid_sig = self.hid_sig()
                print('HID changed', self._hid_sig)

            n = self.hid.getButtonCount()
            hid = (
                self.hid.isConnected(),
                round(self.hid.getRawAxis(0), 2),
                round(self.hid.getRawAxis(1), 2),
                round(self.hid.getRawAxis(2), 2),
                round(self.hid.getRawAxis(3), 2),
                round(self.hid.getRawAxis(4), 2),
                round(self.hid.getRawAxis(5), 2),
                ''.join(str(int(self.hid.getRawButton(i))) for i in range(1, n+1)),
                self.hid.getPOV(0),
                )

            if hid != self._hid_prev:
                self._hid_prev = hid
                c, ax0, ax1, ax2, ax3, ax4, ax5, bs, pov = hid
                print(f'{c=:d} 0={ax0:+5.2f} 1={ax1:+5.2f} 2={ax2:+5.2f} 3={ax3:+5.2f} 4={ax4:+5.2f} 5={ax5:+5.2f} {bs=} {pov=}')

        if self._led_changed:
            self._led_changed = False
            self.led.setData(self.ledData)


    def led_rainbow(self):
        # For every pixel
        length = constants.LED.length
        for i in range(length):
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + i * 180 // length) % 180

            # Set the value
            self.ledData[i].setHSV(hue, 255, 15)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += self.led_speed

        # Check bounds
        self.rainbowFirstPixelHue %= 180
        self._led_changed = True


    # def led_error(self):
    #     for i in range(constants.LED.length):
    #         self.ledData[i].setRGB(2, 0, 0)

    # When DS not connected
    def led_offline(self):
        for i in range(0, constants.LED.length//3*3, 3):
            self.ledData[i].setRGB(1, 0, 0)
            self.ledData[i+1].setRGB(0, 0, 0)
            self.ledData[i+2].setRGB(0, 0, 0)
        self._led_changed = True

    def led_disabled(self):
        for i in range(0, constants.LED.length//2*2, 2):
            self.ledData[i].setRGB(0, 0, 0)
            self.ledData[i+1].setRGB(1, 1, 0)
        self._led_changed = True

    def led_auto(self):
        for i in range(0, constants.LED.length//2*2, 2):
            self.ledData[i].setRGB(0, 0, 0)
            self.ledData[i+1].setRGB(0, 0, 1)
        self._led_changed = True

    def led_teleop(self):
        for i in range(0, constants.LED.length//2*2, 2):
            self.ledData[i].setRGB(0, 0, 0)
            self.ledData[i+1].setRGB(0, 1, 0)
        self._led_changed = True

    def led_test(self):
        for i in range(0, constants.LED.length//2*2, 2):
            self.ledData[i].setRGB(0, 0, 0)
            self.ledData[i+1].setRGB(1, 0, 1)
        self._led_changed = True

    #----------------------------------
    def disabledInit(self):
        print('disabled init')
        # self.cmds.cancelAll()

    def disabledExit(self):
        pass

    def disabledPeriodic(self):
        if wpilib.DriverStation.isDSAttached():
            self.led_disabled()
        else:
            self.led_offline()

    #----------------------------------

    def autonomousInit(self):
        # self.cmds.cancelAll()
        print('autonomous init')
        self.led_auto()


    def autonomousExit(self):
        pass

    def autonomousPeriodic(self):
        pass


    #----------------------------------
    def teleopInit(self):
        print('teleop init')
        self.led_teleop()

    def teleopExit(self):
        pass

    def teleopPeriodic(self):
        pass
        # self.led_rainbow()

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
        print('test init')
        # self.cmds.cancelAll()
        self.led_test()

        # # By default, the Ultrasonic class polls all ultrasonic sensors every in a round-robin to prevent
        # # them from interfering from one another.
        # # However, manual polling is also possible -- notes that this disables automatic mode!
        # self.rangeFinder.ping()

    def testExit(self):
        pass
        # self.cmds.cancelAll()

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
