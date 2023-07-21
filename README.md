# clearpath_state_machine
# State Machine Bash Script

# Version 14 -- FINAL - 1

This repository contains a Bash script implementing a state machine for controlling the behavior of a system. The state machine goes through different states and performs various actions based on those states. The script is designed to control a system involving a Ridgeback robot with cameras and navigation capabilities, as well as an UR5 robotic arm.
Bash Script (main.sh)

The main.sh script is the main implementation of the state machine. It defines several states and functions to perform actions and transitions based on the current state. The script interacts with ROS (Robot Operating System) nodes to control the robot and perform various tasks.
State Descriptions

# The state machine has the following states:

    RIDGEBACK INITIALIZATION
    CAMERA & NAVIGATION PROGRAM
    ORIENTATION & CORRECTION PROGRAM
    PICK REFERENCE & CONVERSION PROGRAM
    PICKING PROGRAM
    MOVE TO GOAL
    PLACING PROGRAM
    BACK TO BASE
    FAULT::FREEZE

# Files and Functions

'start_faultclient'

This function starts the rosrun server_ur5 fault node responsible for fault detection. It creates a file to store the output of the fault node and saves its process ID (PID) for later termination.

'stop_faultclient'

This function stops the rosrun server_ur5 fault node by killing its process using its PID.

'start_amcl_launch'

This function launches the amcl_demo.launch file from the ridgeback_navigation package, which initializes the AMCL (Adaptive Monte Carlo Localization) for the Ridgeback robot. It also runs a node to publish reference height for the UR5 robotic arm.

'stop_amcl_launch'

This function stops the AMCL launch by killing its process using its PID.

'check_interrupt'

This function checks for a specific condition (UR5 STOPPED!) in the output file of the fault detection node. If the condition is detected, it returns the FAULT::FREEZE state, otherwise, it returns an empty string.

'start_conversion_of_orientation'

This function starts the rosrun vision conversion_of_orientation node, which performs coordinate conversion for the robot's orientation.

'stop_conversion_of_orientation'

This function stops the coordinate conversion node by killing its process using its PID.

'invoke_camera_and_navigation'

This function runs the rosrun vision ridgeback_vision node for camera vision and the rosrun rgb_nav simple_nav node for simple navigation of the robot. It also runs a node to get coordinates for Z adjustment in UR5 server3.

'stop_reference_height_node'

This function stops the node responsible for providing reference height to the UR5 robotic arm.

# State Transitions

The main loop in main.sh continuously checks the current state and performs corresponding actions based on each state. It checks for fault conditions and transitions accordingly. The transitions between states are as follows:

    RIDGEBACK INITIALIZATION -> CAMERA & NAVIGATION PROGRAM
    CAMERA & NAVIGATION PROGRAM -> ORIENTATION & CORRECTION PROGRAM
    ORIENTATION & CORRECTION PROGRAM -> PICK REFERENCE & CONVERSION PROGRAM
    PICK REFERENCE & CONVERSION PROGRAM -> PICKING PROGRAM
    PICKING PROGRAM -> MOVE TO GOAL
    MOVE TO GOAL -> PLACING PROGRAM
    PLACING PROGRAM -> BACK TO BASE
    BACK TO BASE -> RIDGEBACK INITIALIZATION

If a fault is detected at any stage, the state transitions directly to FAULT::FREEZE. After handling the fault, the machine transitions back to the previous state.

# Usage

    Clone this repository to your local machine.

    Ensure that you have the required ROS packages and nodes installed and properly configured.

    Make sure to grant execute permission to the state_machine.sh script if needed: chmod +x state_machine.sh

    Run the script using the following command: ./state_machine.sh

    Follow the on-screen instructions to proceed or terminate the program when prompted.

# Note

This script has undergone multiple versions throughout the development process. Each version represents different stages of development or improvements made to the system. Please review the script carefully to understand the changes between versions.
