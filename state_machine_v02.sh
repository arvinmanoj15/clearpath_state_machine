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

# File to store the output of rosrun server_ur5 fault
output_file="fault_output.txt"

# Initialize fault_pid variable
fault_pid=""

# Function to start the faultclient process
start_faultclient() {
    echo "Starting rosrun server_ur5 fault"
    rm -f "$output_file"
    touch "$output_file"
    # Fault detection node invoking
    rosrun server_tcp_ip server2 > "$output_file" 2>&1 &
    fault_pid=$!
    echo "rosrun server_ur5 fault started with PID: $fault_pid"
}

# Function to stop the faultclient process
stop_faultclient() {
    echo "Stopping rosrun server_ur5 fault"
    kill -SIGTERM $fault_pid
    rm -f "$output_file"
    echo "rosrun server_ur5 fault stopped"
    fault_pid=""
}

# Function to check for UR5 Stopped interrupt condition
check_interrupt() {
    if grep -qF "UR5 STOPPED!" "$output_file"; then
        echo $STATE_9
    else
        echo ""
    fi
}

# Perform the state transitions
while true; do
    case $current_state in
        "$STATE_1")
            echo "Currently in $STATE_1"
            start_faultclient
            echo "Transitioning from $STATE_1 to $STATE_2"
            current_state=$STATE_2
            ;;
        "$STATE_2")
            echo "Currently in $STATE_2"
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Transitioning from $STATE_2 to $STATE_3"
                current_state=$STATE_3
            fi
            ;;
        "$STATE_3")
            echo "Currently in $STATE_3"
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
            next_state=$(check_interrupt)
            if [[ -n $next_state ]]; then
                echo "Fault detected. Transitioning directly to $next_state"
                stop_faultclient
                current_state=$next_state
            else
                echo "Stopping rosrun server_tcp_ip urs"
                stop_faultclient
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
                    echo "Transitioning from $STATE_9 to $STATE_1"
                    current_state=$STATE_1
                    break
                fi
            done
            ;;
    esac

    sleep 1
done
