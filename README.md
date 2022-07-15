# Panda TicTacToe 

Playing TicTacToe with the Panda collaborative robot.

![setup](https://cloud.githubusercontent.com/assets/4378663/26005307/ac35d298-3706-11e7-84c7-c278a01fe3b7.jpg)

This code is released under a GNU GPL v2 license (see LICENSE file for more information).

## TOOD:
This is currently set up for the baxter robot and will need to be modified for the panda robot.
 * Update to >=OpenCV 4.2
 * change all baxter control calls to panda control calls
 * change usb camera/hand camera to Realsense
 * delete display stuff or update to run on TV
 * recalibrate color idnetification for playing pieces

## Requirements

 * a Panda robot
 * a realsense camera
 * sudo apt-get install festival v4l-utils

## Usage

### Initialization

Obvious stuff:

 * make sure the Baxter is on
 * make sure the computer is on
 * make sure the Baxter and the computer are connected together
 * make sure the computer has some working speakers
 * make sure the board and tokens are placed on a table in front of Baxter

### Calibration

In order to calibrate the hard coded positions for the pile of objects and the board, just record the end-effector position of the left arm when positioned either on the pile of tiles, or the four corners of the board. To to so, the following command may be useful:

```
rostopic echo -n 1 /robot/limb/right/endpoint_state | grep -A 3 position
```

Then, use this information to populate the corresponding parameters in `tictactoe.launch` : `"ttt_controller/tile_pile_position` and `ttt_controller/board_corner_poss`.

### Run the demo

 * Make sure the board is empty
 * Open a new terminal
 * `cd` to `~/ros_ws`
 * `source devel/setup.bash`
 * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
 * `rosrun baxter_tools tuck_arms.py -u`
 * `roslaunch baxter_tictactoe tictactoe.launch`
 * Exit program

### Shut down the robot

 * Open a terminal:
 * `cd` to `~/ros_ws`
 * `source devel/setup.bash`
 * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
 * `rosrun baxter_tools tuck_arms.py -t`


