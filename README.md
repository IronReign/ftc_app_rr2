# ftc_app_rr2

This repository contains the code used by [Iron Reign Robotics](https://github.com/IronReign) during the 2018-2019 FIRST Tech Challenge Rover Ruckus season.

## Developers
- Arjun Vikram ([arjvik](https://github.com/arjvik))
- Abhi Bhattaru ([abhimaster2001](https://github.com/abhimaster2001))

## Features
- Full autonomous and more (85 points) including
    - Delatching (30 points)
    - Sampling using OpenCV Vision (25 points)
    - Scoring sampled gold (5 points)
    - Claiming depot (15 points)
    - Parking in crater (10 points)
- VisionProvider abstractions
    - Allows runtime selection of vision backend
    - All backends developed over the season, best performing selected
    - Choice between
        - TensorFlow Object Detection CNN
        - OpenCV pipeline
        - Vuforia object tracking
        - Custom trained classification CNN
        - Dummy backends which always provide the same answer
    - Choice between different cameras at runtime
- State Machines
    - Allows autonomous code to be written as a sequence of "states" or actions
    - Each state is responsible for performing its action and handing over control when it is complete
    - State Machine keeps track of sequences of states and currently executing state
    - Full autonomous routine can be written in 30 lines of code
- Articulations
    - An articulation is a position of the robot, made up of encoder targets for each of the subsystems
    - Using articulations, the robot can adjust itself into any of a large number of pre-defined positions
    - The articulation manager is responsible for maintaining the current articulation of the robot, as well as managing transitions between articulations
    - Subsystem target-heavy routines during autonomous are encoded as articulation transitions
    - Articulations are also used during the driver-operated period where drivers press single buttons to switch into the next logical articulation to complete a mineral cycle

## Questions and Further Information

Please contact Arjun at arjvik@gmail.com for any questions regarding this code.
