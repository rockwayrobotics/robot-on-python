Robot on Python
===============
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

More to come...
