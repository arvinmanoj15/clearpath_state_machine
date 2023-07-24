#!/bin/bash

# Define the states
STATE_1="RIDGEBACK INITIALIZATION"
STATE_2="CAMERA & NAVIGATION PROGRAM"
STATE_3="ORIENTATION & CORRECTION PROGRAM"
STATE_4="PICK REFERNCE & CONVERSION PROGRAM"
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
output_file_1="fault_output.txt"

# File to store the output of rosrun conversion
output_file_2="conversion_output.txt"

# File to store the output of rosrun goal cancel node
output_file_3="cancel_output.txt"

# Initialize fault_pid variable
fault_pid=""

# Initialize fault_pid variable
ref_height_pid=""

# Initialize amcl_launch_pid variable
amcl_launch_pid=""

# Initialize conversion_pid variable
conversion_pid=""

# Initialize goal_planner variable
goal_planner_pid=""

# Initialize goal_planner variable
goal_cancel_pid=""

# Initialize goal_planner variable
simple_nav_pid=""


# Function to start the faultclient process
start_faultclient() {
    echo "Starting rosrun server_ur5 fault"
    rm -f "$output_file_1"
    touch "$output_file_1"
    # Fault detection node invoking
    rosrun server_ur5 fault > "$output_file_1" 2>&1 &
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
    if grep -qF "UR5 STOPPED!" "$output_file_1"; then
        echo $STATE_9
    else
        echo ""
    fi
}

# Function to check for UR5 Stopped interrupt condition
check_bottle_detected() {
    if grep -qF "Bottle Detected!" "$output_file_3"; then
        current_state=$STATE_3
        echo "True"
    else
        echo "False"
    fi
}

# Function to invoke the Coordinate conversion node
start_conversion_of_orientation() {
    rm -f "$output_file_2"
    touch "$output_file_2"
    rosrun vision conversion_of_orientation > "$output_file_2" &
    conversion_pid=$!
}

stop_conversion_of_orientation() {
    if [ -n "$conversion_pid" ]; then
        echo "Killing process with PID: $conversion_pid"
        kill -SIGTERM $conversion_pid
        conversion_pid=""
    else
        echo "No process to kill."
    fi
}

# Waiting for the navigation process to complete
wait_for_navigation() {
    while ps -p "$simple_nav_pid" > /dev/null
    do
        sleep 1
    done
}


# Function to start the planner and cancellation
invoke_main_task() {
    
    rosrun rgb_nav simple_nav &
    
    sleep 1
    
    # To get the coordinates for Z adjustment in UR5 server3
    rosrun server_ur5 height_publisher &
    ref_height_pid=$!

    rm -f "$output_file_3"
    touch "$output_file_3"
    
    rosrun rgb_nav goal_cancel_node > "$output_file_3" &
    goal_cancel_pid=$!
    count=0
    
    while [ "$(check_bottle_detected)" = "False" ] && [ "$count" -lt 6 ]
    do
    	if [ "$current_state" = "$STATE_3" ]; then
    		echo "Break Above"
    		break
	fi
        rosrun rgb_nav motion_plan &
        motion_plan_pid=$!
        ((count++))
        echo "$count"
        while ps -p $motion_plan_pid > /dev/null
	do
	    	if [ "$(check_bottle_detected)" = "True" ]; then
		    kill -SIGTERM $motion_plan_pid
		    current_state=$STATE_3
		    echo "STATE_3"
		    break
		fi
		if [ "$count" -eq 6 ]; then
		    current_state=$STATE_8
		fi
	done
        if [ "$current_state" = "$STATE_3" ]; then
        	echo "Break Below"
    		break
	fi
    done

    
}

# Function to stop the faultclient process
stop_reference_height_node() {
    if [ -n "$ref_height_pid" ]; then
        echo "Killing process with PID: $ref_height_pid"
        kill -SIGTERM $ref_height_pid
        ref_height_pid=""
    else
        echo "No process to kill."
    fi
}

# Function to stop the goal planner process
stop_goal_planner() {
    if [ -n "$goal_planner_pid" ]; then
        echo "Killing process with PID: $goal_planner_pid"
        kill -SIGTERM $goal_planner_pid
        goal_planner_pid=""
    else
        echo "No process to kill."
    fi
}

# Function to stop the goal cancel process
stop_goal_cancel() {
    if [ -n "$goal_cancel_pid" ]; then
        echo "Killing process with PID: $goal_cancel_pid"
        kill -SIGTERM $goal_cancel_pid
        goal_cancel_pid=""
    else
        echo "No process to kill."
    fi
}



# Perform the state transitions
while true; do
    case $current_state in
        "$STATE_1")
            echo "Currently in $STATE_1"
            start_faultclient
            rosrun server_ur5 server2
            start_amcl_launch
            rosrun rgb_nav goal_planner_node &
            goal_planner_pid=$!
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
                sleep 3
		 invoke_main_task
                echo "Transitioning from $STATE_2"
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
                wait_for_navigation
                rosrun server_ur5 server3
                sleep 7
                rosrun vision orient_to_target
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
                start_conversion_of_orientation
                rosrun	rgb_nav position_adjustment
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
                rosrun server_ur5 server4
                sleep 30
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
                
                stop_conversion_of_orientation
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
                rosrun server_ur5 server1
                stop_reference_height_node
                echo "Transitioning from $STATE_7 to $STATE_8"
                current_state=$STATE_2
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
                stop_goal_planner
                stop_goal_cancel
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
