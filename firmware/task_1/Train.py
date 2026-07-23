'''
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 1B of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Train.py
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
from Connector import CoppeliaClient
from Qlearning import QLearningController

stop_requested = False
def signal_handler(sig, frame):
    global stop_requested
    print("\n[TRAIN] Ctrl+C pressed. Stopping gracefully...")
    stop_requested = True
signal.signal(signal.SIGINT, signal_handler)

def main():
    global stop_requested
    ql = QLearningController()
    ql.load_q_table()
    client = CoppeliaClient()
    client.connect()

    iteration = 0
    prev_state = None
    prev_action = None
    SAVE_INTERVAL = 1
    prev_sensor_vals = None
    smooth_alpha = 0.05 # lower -> more responsive to current readings (0..1)
    # Pivot/timing configuration
    pivot_end_time = 0.0
    pivot_left_speed = 0.0
    pivot_right_speed = 0.0
    PIVOT_DURATION = 1.8  # seconds (500 ms)
    PIVOT_TYPE = 'soft'  # 'soft'|'spin'|'stop'
    PIVOT_OUTER = 3.0
    PIVOT_INNER = 0.2
    # You can tune `threshold` and `smooth_alpha` below if the bot drifts.

    print("[TRAIN] Training started...")
    while not stop_requested:
        sensor_data = client.receive_sensor_data()
        if not sensor_data:
            time.sleep(0.02)
            continue

        print(f"[TRAIN] Raw sensor_data: {sensor_data}")

        # Preprocess sensor_data to binary values using a fixed sensor order.
        # If the simulator returns a dict, we map keys to this order. If it
        # returns a list/tuple, we use the first five values.
        sensor_keys = ['left_corner', 'left', 'middle', 'right', 'right_corner']
        threshold = 0.2  # values lower than this are considered "on line"; tune if too many false positives

        if isinstance(sensor_data, dict):
            sensor_vals = [sensor_data.get(k, 1.0) for k in sensor_keys]
        else:
            # Convert iterable to list and take up to 5 values
            sensor_vals = list(sensor_data)[:5]
            # Fill up to 5 with high values (off-line) if needed
            while len(sensor_vals) < 5:
                sensor_vals.append(1.0)

        # Smooth numeric sensor values to reduce jitter-driven oscillation
        if prev_sensor_vals is None:
            smoothed_vals = sensor_vals[:]
        else:
            smoothed_vals = []
            for prev_v, cur_v in zip(prev_sensor_vals, sensor_vals):
                # If values are numeric, apply exponential smoothing; otherwise keep as-is
                if isinstance(cur_v, (int, float)) and isinstance(prev_v, (int, float)):
                    sm_v = smooth_alpha * prev_v + (1 - smooth_alpha) * cur_v
                else:
                    sm_v = cur_v
                smoothed_vals.append(sm_v)
        prev_sensor_vals = smoothed_vals[:]

        # Normalize sensor readings so that higher == on-line (1.0).
        # Support two modes: ADC-scale (>1.0) and normalized 0..1 readings.
        ADC_BLACK = 1500.0
        ADC_WHITE = 4096.0

        # Helper that maps a single raw value to 0..1 given an `invert` flag
        def _map_value(v, invert=False):
            if isinstance(v, (int, float)) and v > 1.0:
                # ADC scale: map [ADC_BLACK, ADC_WHITE] -> [0,1]
                if ADC_WHITE == ADC_BLACK:
                    nv = 0.0
                else:
                    nv = (float(v) - ADC_BLACK) / (ADC_WHITE - ADC_BLACK)
                return max(0.0, min(1.0, nv))
            elif isinstance(v, (int, float)):
                # Normalized 0..1: either use as-is or invert
                if invert:
                    nv = 1.0 - float(v)
                else:
                    nv = float(v)
                return max(0.0, min(1.0, nv))
            else:
                sval = str(v).lower()
                return 1.0 if sval in ['1', 'white', 'center', 'on_line', 'on line', 'true', 't'] else 0.0

        # Build two candidate normalizations (invert=False and invert=True)
        candidate_noinv = [_map_value(v, invert=False) for v in smoothed_vals]
        candidate_inv = [_map_value(v, invert=True) for v in smoothed_vals]

        # Choose the mapping that makes the middle sensor more "on-line"
        # relative to its immediate neighbors. This avoids transient flips
        # when a side sensor briefly crosses 0.5 and would otherwise invert
        # the entire mapping.
        def _center_advantage(nlist):
            try:
                center = nlist[2]
                left = nlist[1]
                right = nlist[3]
            except Exception:
                # If not enough sensors, fall back to simple center value
                return nlist[2] if len(nlist) > 2 else 0.0
            return center - max(left, right)

        # If all smoothed values are very small, it's likely the line is
        # absent (or readings are near-zero). In that case prefer the
        # non-inverted mapping so we don't mistakenly treat zeros as
        # high on-line values after inversion.
        MIN_FOR_INVERT = 0.2
        if max(smoothed_vals) < MIN_FOR_INVERT:
            norm_vals = candidate_noinv
        else:
            if _center_advantage(candidate_inv) > _center_advantage(candidate_noinv):
                norm_vals = candidate_inv
            else:
                norm_vals = candidate_noinv

        # Build binary bits from normalized values (higher==on-line)
        threshold_norm = 0.6
        binary_sensor_data = [1 if v >= threshold_norm else 0 for v in norm_vals]

        print(f"[TRAIN] Sensor values used (ordered): {sensor_vals}")
        print(f"[TRAIN] Smoothed sensor values: {smoothed_vals} (smooth_alpha={smooth_alpha})")
        print(f"[TRAIN] Normalized on-line values: {norm_vals} (ADC map {ADC_BLACK}->{ADC_WHITE})")
        print(f"[TRAIN] Binary sensor_data: {binary_sensor_data} (threshold_norm={threshold_norm})")

        # Special stop condition: if pattern [0,1,1,1,0] is seen, save and exit
        if binary_sensor_data == [0, 1, 1, 1, 0]:
            print("[TRAIN] Detected stop pattern [0,1,1,1,0] -> saving Q-table and stopping.")
            try:
                ql.save_q_table()
            except Exception as e:
                print(f"[TRAIN] Warning: failed to save Q-table: {e}")
            try:
                client.send_motor_command(0, 0)
            except Exception:
                pass
            try:
                client.close()
            except Exception:
                pass
            return

        state = ql.Get_state(binary_sensor_data)

        if prev_state is not None and prev_action is not None:
            # Use normalized analog values for reward computation (higher==on-line)
            reward = ql.Calculate_reward(norm_vals)
            ql.update_q_table(prev_state, prev_action, reward, state)

        # Rule-based override to keep the robot centered on the line.
        # - If exactly center sensor is on: go forward.
        # - If any left-side sensor is on: turn left (hard if far-left corner, else soft).
        # - If any right-side sensor is on: turn right (hard if far-right corner, else soft).
        # This takes priority over the Q-learner's chosen action to improve stability.
        rule_action = None
        # binary_sensor_data indices: 0=left_corner,1=left,2=middle,3=right,4=right_corner
        if binary_sensor_data == [0, 0, 1, 0, 0]:
            rule_action = 2  # forward
        else:
            # Prefer left correction if any left sensor sees the line
            if binary_sensor_data[0] == 1:
                rule_action = 0  # hard left
            elif binary_sensor_data[1] == 1:
                rule_action = 1  # soft left
            elif binary_sensor_data[4] == 1:
                rule_action = 4  # hard right
            elif binary_sensor_data[3] == 1:
                rule_action = 3  # soft right

        if rule_action is not None:
            action = rule_action
            # For corner sensors prefer a pivot: start or continue a timed pivot.
            now = time.time()
            if now < pivot_end_time:
                # If center regains dominance during the pivot, abort early
                # to avoid overshoot. Use a small margin to avoid flapping.
                center = norm_vals[2] if len(norm_vals) > 2 else None
                side_max = max(norm_vals[1], norm_vals[3]) if len(norm_vals) > 3 else 0.0
                ABORT_MARGIN = 0.12
                if center is not None and center >= threshold_norm and (center > side_max + ABORT_MARGIN):
                    # Cancel pivot and go forward to stabilize
                    pivot_end_time = 0.0
                    action = 2
                    left_speed, right_speed = ql.perform_action(action)
                    print(f"[TRAIN] Aborting pivot early -> center regained (center={center:.2f}, side_max={side_max:.2f})")
                else:
                    # Continue the active timed pivot
                    left_speed, right_speed = pivot_left_speed, pivot_right_speed
                    remaining = pivot_end_time - now
                    print(f"[TRAIN] Continuing timed pivot -> speeds: {left_speed},{right_speed} (remaining {remaining:.2f}s)")
            else:
                # No active pivot; if this rule_action requests a corner pivot,
                # set pivot speeds and start the timer so the pivot runs for
                # `PIVOT_DURATION` seconds across iterations.
                if action == 0 and binary_sensor_data[0] == 1:
                    # pivot left
                    if PIVOT_TYPE == 'spin':
                        pivot_left_speed, pivot_right_speed = -PIVOT_OUTER, PIVOT_OUTER
                    elif PIVOT_TYPE == 'soft':
                        pivot_left_speed, pivot_right_speed = PIVOT_INNER, PIVOT_OUTER
                    else:
                        pivot_left_speed, pivot_right_speed = 0.0, 2.8
                    pivot_end_time = now + PIVOT_DURATION
                    left_speed, right_speed = pivot_left_speed, pivot_right_speed
                    print(f"[TRAIN] Starting timed pivot LEFT -> speeds: {left_speed},{right_speed} (duration {PIVOT_DURATION}s)")
                elif action == 4 and binary_sensor_data[4] == 1:
                    # pivot right
                    if PIVOT_TYPE == 'spin':
                        pivot_left_speed, pivot_right_speed = PIVOT_OUTER, -PIVOT_OUTER
                    elif PIVOT_TYPE == 'soft':
                        pivot_left_speed, pivot_right_speed = PIVOT_OUTER, PIVOT_INNER
                    else:
                        pivot_left_speed, pivot_right_speed = 2.8, 0.0
                    pivot_end_time = now + PIVOT_DURATION
                    left_speed, right_speed = pivot_left_speed, pivot_right_speed
                    print(f"[TRAIN] Starting timed pivot RIGHT -> speeds: {left_speed},{right_speed} (duration {PIVOT_DURATION}s)")
                else:
                    left_speed, right_speed = ql.perform_action(action)

            print(f"[TRAIN] Rule override engaged -> action {action} based on binary {binary_sensor_data}")
        else:
            action = ql.choose_action(state)
            left_speed, right_speed = ql.perform_action(action)
        # Use normalized analog values for reward
        reward = ql.Calculate_reward(norm_vals)

        # Debug: show which motor speeds are being sent for this sensor reading
        print(f"[TRAIN] Chosen Action: {action}, Left speed: {left_speed}, Right speed: {right_speed}, Reward: {reward}")
        client.send_motor_command(left_speed, right_speed, state=state, action=action, reward=reward)

        prev_state = state
        prev_action = action
        iteration += 1

        if iteration % SAVE_INTERVAL == 0:
            ql.save_q_table()
            print(f"[TRAIN] Saved Q-table at iteration {iteration}")

        time.sleep(0.05)

    ql.save_q_table()
    client.send_motor_command(0, 0)
    client.close()
    print("[TRAIN] Training stopped and Q-table saved.")

if __name__ == "__main__":
    main()
