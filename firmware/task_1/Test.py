'''
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 1B of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Test.py
*  Created:		    24/08/2025
*  Last Modified:	24/08/2025
*  Author:		    e-Yantra Team
*  Team ID:		    [ CB_XXXX ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
'''
'''You can Modify the this file,add more functions According to your usage.
   You are not allowed to add any external packges,Beside the included Packages.You can use Built-in Python modules.'''
import time
import signal
import sys

# Import required classes for simulation and Q-learning
from Connector import CoppeliaClient      
from Qlearning import QLearningController 

# Global flag to handle safe interruption (Ctrl+C)
stop_requested = False

def signal_handler(sig, frame):
    """
    Signal handler to catch keyboard interrupt (Ctrl+C).
    It sets a flag to stop the main testing loop gracefully.
    """
    global stop_requested
    print("\n[TEST] Interrupt received. Stopping testing gracefully...")
    stop_requested = True

# Register the signal handler to handle SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

def main():
    global stop_requested

    # ===  Initialize the Q-learning controller and load the Q-table ===
    ql = QLearningController()
    loaded = ql.load_q_table()
    if not loaded:
        print("[TEST] No Q-table found. Exiting.")
        return

    # ===  Connect to the CoppeliaSim simulation environment ===
    client = CoppeliaClient()
    client.connect()

    print("[TEST] Starting test loop...")

    # === Start the testing loop ===
    while not stop_requested:
        # Step Receive current sensor data from simulator
        sensor_data = client.receive_sensor_data()
        if not sensor_data:
            continue

        # Preprocess sensor_data to binary values
        binary_sensor_data = []
        for val in sensor_data:
            if str(val).lower() in ['1', 'white', 'center', 'on_line']:
                binary_sensor_data.append(1)
            else:
                binary_sensor_data.append(0)

        # Convert sensor data into a discrete state
        state = ql.Get_state(binary_sensor_data)

        # Calculate reward just for logging/debug (not used in real Q-learning here)
        reward = ql.Calculate_reward(binary_sensor_data)

        # Choose an action based on the current state (pure exploitation, not training)
        action = ql.choose_action(state)

        # Convert action into motor commands
        left_speed, right_speed = ql.perform_action(action)
        # Send motor command to robot 
        client.send_motor_command(left_speed, right_speed, state=state, action=action, reward=reward)

        #  Wait for a short period before next iteration (to control speed of testing)
        time.sleep(0.05)

    # ===  Stop the robot and close the connection after exiting loop (Ctrl+C) ===
    client.send_motor_command(0, 0)  # Stop the robot
    client.close()  # Disconnect from simulator
    print("[TEST] Testing stopped.")


# Entry point for the script
if __name__ == "__main__":
    main()
