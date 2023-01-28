# Robot on Python

Robot implementation using RobotPy.  [Start here.](https://robotpy.readthedocs.io/en/latest/getting_started.html)

In short:

1.  On Windows, install Visual Studio 2019 redistributable.  If you are set
    up to do FRC stuff then this is likely already installed.
2.  `py -m pip install robotpy[all]`
    This should install RobotPy and pretty much every dependency including
    optional ones like cscore, REV, PhotonVision and more.
3.  `py robot.py sim` would run the code in simulator mode, though without
    a finished physics.py that won't work.
4.  `py robot.py test` would run our automated tests, if we have any. (We don't yet.)
5.  `py robot.py deploy` would run the code on the robot, provided someone
    has already [installed RobotPy 2023](https://robotpy.readthedocs.io/en/latest/install/robot.html#install-robotpy).
    Add the `--nc` option to see console output directly rather than having
    to open a separate NetConsole window (whatever that is).
    If we have any automated tests that are failing (since deploy runs them
    automatically), add `--skip-tests`.
6.  If you want the code to run at robot boot time, see
    [this link](https://robotpy.readthedocs.io/en/latest/guide/deploy.html#starting-deployed-code-at-boot).

Note that none of this installs RobotPy on the robot itself.  To do that
follow [Robot Installation](https://robotpy.readthedocs.io/en/stable/install/robot.html)
and note also that unless you use `robotpy_installer install robotpy[all]` you
will have to install various third-party vendor packages separately.  For
example if you need `import rev` you'll want to do `robotpy_install install robotpy[rev]`
and so on.  See also [Package notes](https://robotpy.readthedocs.io/en/stable/install/index.html#package-notes).


## API Basics

### Joysticks

Joystick axis readings range between -1.0 and 1.0.
Generally forward is Y negative, backward is Y positive.
Left is X negative, right is X positive.
Joysticks may have a twist axis, or others e.g. a throttle
or a sensitivity axis.

The axes are associated with channels, which can be queried
or set.  The defaults are e.g. kDefaultThrottleChannel.

Buttons include a trigger button (usually index finger),
a "top" button (thumb), and possibly many others.

The API provides routines to read the current state, e.g. `getTop()`
and also some stateful values such as `getTopPressed()` or
`getTopReleased()` which apparently return True only once, when
first called following the pressing or releasing of the button.

The wpilib.DriverStation API provides a few things to query
the currently attached controllers.  Note that it appears that
attaching a joystick while the code is running may not be
visible without a restart, at least based on running in the simulator.

* DS.isJoystickAttached(n)
* DS.getJoystickName(n) e.g. 'Logitech Extreme 3D' or 'Keyboard 0'
* DS.getJoystickType(n) is useful in theory but in the sim at least
    returns 0 for everything.
* DS.getLocation() returns 1, 2, 3 for position of driver station in the field.
* DS.getAlliance() returns DS.Alliance.kRed/kBlue/kInvalid (0/1/2)
* DS.getMatchTime() may be useful in helping control end-of-period
    autonomous routines, possibly cutting them short if there's not
    enough time for a fancier version...
    Note that it's documented as being not guaranteed to be accurate
    so there may be a better way to do this.
* DS.getStickAxisCount(n) returns a count, 3 for sim keyboard, 4 for Logitech 3D
* DS.getStickButtonCount() also works, 4 for sim kb, 12 for L3D
* DS.isDSAttached() may be useful, though note that it's True for the sim
    as well (though sim can toggle it).  Presumably most useful in
    judging whether these other APIs are going to return useful data,
    and maybe ones like the wpilib.Joystick and similar controller stuff.


### DigitalInputs

`di = wpilib.DigitalInput(n)` gets a channel 0-9 (on the RIO) or 10-25 (MXP).

`di.get()` returns boolean with current state.

Simulation: setSimDevice(n) ... not sure how the interface with simulated
stuff works yet.
