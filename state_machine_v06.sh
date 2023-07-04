#!/bin/bash

# Define the states
STATE_1="RIDGEBACK INITIALIZATION"
STATE_2="CAMERA PROGRAM"
STATE_3="NAVIGATION PROGRAM"
STATE_4="ORIENTATION & CORRECTION"
STATE_5="PICKING PROGRAM"
STATE_6="MOVE TO GOAL"
STATE_7="PLACING PROGRAM"
STATE_8="BACK TO BASE"
STATE_9="FAULT::FREEZE"

# Initialize the current state
current_state=$STATE_1

# Flag State to return after fault::freeze
flag_state=$current_state

# File to store the output of rosrun server_ur5 fault
output_file="fault_output.txt"

# Initialize fault_pid variable
fault_pid=""

# Initialize amcl_launch_pid variable
amcl_launch_pid=""

# Function to start the faultclient process
start_faultclient() {
    echo "Starting rosrun server_ur5 fault"
    rm -f "$output_file"
    touch "$output_file"
    # Fault detection node invoking
    rosrun server_ur5 fault > "$output_file" 2>&1 &
    fault_pid=$!
    echo "rosrun server_ur5 fault started with PID: $fault_pid"
}

# Function to stop the faultclient process
stop_faultclient() {
    if [ -n "$fault_pid" ]; then
        echo "Killing process with PID: $fault_pid"
        kill -SIGTERM $fault_pid
        fault_pid=""
    else
        echo "No process to kill."
    fi
}

# Function to start the AMCL launch file
start_amcl_launch() {
    roslaunch ridgeback_navigation amcl_demo.launch map_file:=$HOME/mymap.yaml &
    #rosrun server_tcp_ip urs &
    amcl_launch_pid=$!
    echo "Process launched with PID: $amcl_launch_pid"
    sleep 5
    rosrun rgb_nav init_pose &
    sleep 2
}

# Function to stop the AMCL launch file
stop_amcl_launch() {
    if [ -n "$amcl_launch_pid" ]; then
        echo "Killing process with PID: $amcl_launch_pid"
        kill -SIGINT $amcl_launch_pid
        amcl_launch_pid=""
    else
        echo "No process to kill."
    fi
}

# Function to check for UR5 Stopped interrupt condition
check_interrupt() {
    if grep -qF "UR5 STOPPED!" "$output_file"; then
        echo $STATE_9
    else
        echo ""
    fi
}

# Function to start the camera and navigation node
invoke_camera_and_navigation() {
    # Run the first command in the background and save its process ID
    rosrun vision ridgeback_vision &
    local vision_pid=$!

    # Run the second command in the background and save its process ID
    rosrun rgb_nav simple_nav &
    local nav_pid=$!

    # Initialize flag variable
    local flag=0

    # Infinite loop
    while true; do
        sleep 1  # Delay of 1 second

        # Check if process 1 is still active
        if ps -p $vision_pid > /dev/null; then
            ((flag++))  # Increase flag by one

            # Check if flag exceeds 55
            if ((flag > 55)); then
                echo "Process 1 is still active after 55 seconds. Terminating both processes..."
                kill $vision_pid $nav_pid  # Terminate both processes
                break
            fi
        else
            echo "Process 1 finished before 55 seconds."
        fi

        # Check if process 2 is inactive
        if ! ps -p $nav_pid > /dev/null; then
            echo "Process 2 finished."
            break
        fi
    done
}


# Perform the state transitions
while true; do
    case $current_state in
        "$STATE_1")
            echo "Currently in $STATE_1"
            start_faultclient
            start_amcl_launch
            echo "Transitioning from $STATE_1 to $STATE_2"
            current_state=$STATE_2
            ;;
        "$STATE_2")
            echo "Currently in $STATE_2"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
		 invoke_camera_and_navigation 
                echo "Transitioning from $STATE_2 to $STATE_3"
                current_state=$STATE_3
            fi
            ;;
        "$STATE_3")
            echo "Currently in $STATE_3"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Transitioning from $STATE_3 to $STATE_4"
                current_state=$STATE_4
            fi
            ;;
        "$STATE_4")
            echo "Currently in $STATE_4"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Transitioning from $STATE_4 to $STATE_5"
                current_state=$STATE_5
            fi
            ;;
        "$STATE_5")
            echo "Currently in $STATE_5"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Transitioning from $STATE_5 to $STATE_6"
                current_state=$STATE_6
            fi
            ;;
        "$STATE_6")
            echo "Currently in $STATE_6"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Transitioning from $STATE_6 to $STATE_7"
                current_state=$STATE_7
            fi
            ;;
        "$STATE_7")
            echo "Currently in $STATE_7"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Transitioning from $STATE_7 to $STATE_8"
                current_state=$STATE_8
            fi
            ;;
        "$STATE_8")
            echo "Currently in $STATE_8"
            flag_state=$current_state
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Stopping rosrun server_tcp_ip urs"
                echo "Stopping AMCL Launch"
                stop_faultclient
                stop_amcl_launch
                while true; do
                    echo "Enter 'EXIT' to TERMINATE or 'CONTINUE' to PROCEED"
                    read -r user_input
                    if [[ $user_input == "EXIT" ]]; then
                        echo "TERMINATING PROGRAM"
                        current_state=$STATE_1
                        exit 0
                    elif [[ $user_input == "CONTINUE" ]]; then
                        echo "PROCEEDING PROGRAM"
                        current_state=$STATE_1
                        break
                    else 
                        echo "INVALID INPUT"
                    fi
                done
            fi
            ;;
        "$STATE_9")
            echo "RESET MACHINE :: FREEZE"
            while true; do
                echo "Enter 'DONE' to transition to $STATE_1"
                read -r user_input
                if [[ $user_input == "DONE" ]]; then
                    echo "Transitioning from $STATE_9 to $flag_state"
                    current_state=$flag_state
                    start_faultclient
                    break
                fi
            done
            ;;
    esac

    sleep 1
done
