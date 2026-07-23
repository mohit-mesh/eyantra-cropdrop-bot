/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 2b of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Task2b.c
*  Created:		    19/08/2025
*  Last Modified:	17/09/2025
*  Author:		    Mohit,Aaryan,Sanyog
*  Team ID:		    [ CB_4775 ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
**********************************************

*/

// Platform-specific includes for Windows compatibility
#ifdef _WIN32
    #define WINVER 0x0600
    #define _WIN32_WINNT 0x0600
    #include <winsock2.h>
    #include <ws2tcpip.h>
#endif

#include "coppeliasim_client.h"  // Include our header
#include <sys/time.h>
#include <math.h>
#include <string.h>

// PID constants for line following (tuned for higher-speed behavior)
// Increased Kp/Kd slightly for faster response; Ki increased slightly for stability
const float Kp = 35.0f;  // Proportional gain
const float Ki = 0.002f; // Integral gain
const float Kd = 12.2f;   // Derivative gain

// Color swap constants
const int COLOR_SWAP_RB = 0; // 0 = no swap, 1 = swap R and B channels
const int COLOR_SWAP_RG = 1; // 0 = no swap, 1 = swap R and G channels
const int COLOR_SWAP_BG = 1; // 0 = no swap, 1 = swap B and G channels

// PID variables
float previous_error = 0.0;
float integral = 0.0;

// Robot states
typedef enum {
    WHITE_FOLLOW,
    TRANSITION_TO_BLACK,
    WHITE_EXIT,
    DETECT_BOXES,
    PICK_THIRD,
    BLACK_FOLLOW,
    DECIDE_TURN,
    DROP
} RobotState;

// Global variables for state management
RobotState current_state = WHITE_FOLLOW;
int box_count = 0;
char box_colors[3][10];
int picked_index = -1;
char picked_color[10] = "";
int junction_count = 0;
bool box_detected = false; // To avoid multiple detections per box
double last_detection_time = 0.0; // To add delay between detections
double white_exit_black_start_time = 0.0;
const double WHITE_EXIT_BLACK_DEBOUNCE_SEC = 0.75;
bool at_junction_flag = false; // Debounce flag for junctions
double last_junction_time = 0.0;
const double JUNCTION_DEBOUNCE_SEC = 1.0; // Ignore junctions within 1 second
// Lockout to avoid double-detection of the same black junction
double junction_lock_time = 0.0; // start time of junction lock
const double JUNCTION_LOCK_SEC = 25.0; // lockout duration in seconds
// Milliseconds to settle (stop) at a detected junction before turning
const int JUNCTION_SETTLE_MS = 350; // ms
// Junction approach smoothing to reduce wobble
bool junction_approach_flag = false;
const float JUNCTION_APPROACH_CENTER = 0.60f; // center sensor averaged threshold
const float JUNCTION_APPROACH_SIDE = 0.50f;   // side sensors averaged threshold
const float BLACK_APPROACH_BASE = 0.62f;      // slow forward speed when approaching junction (increased)
const float BLACK_APPROACH_MAX_CORR = 0.12f;  // maximum differential during approach
const float BLACK_APPROACH_CORR_GAIN = 0.62f; // scale for computed correction from averaged error (increased for speed)
// Post-turn smoothing (reduce PID aggressiveness for a short window after a turn)
bool post_turn_flag = false;
double post_turn_lock_time = 0.0;
const double POST_TURN_LOCK_SEC = 0.60; // seconds to apply reduced-PID after a turn
const float POST_TURN_KP_SCALE = 0.30f; // scale Kp for smoother recovery
const float POST_TURN_KI_SCALE = 0.15f; // scale Ki
const float POST_TURN_KD_SCALE = 0.35f; // scale Kd
const float POST_TURN_BASE = 0.82f;      // reduced base speed during post-turn smoothing (increased)
// Alignment before turning
const int ALIGN_MAX_ATTEMPTS = 10;
const float ALIGN_CENTER_TOL = 0.08f; // target error magnitude for alignment
const float ALIGN_SIDE_TOL = 0.12f;   // side-balance tolerance
const float ALIGN_ADJ_SPEED = 0.28f;  // spin speed used for fine alignment
const int ALIGN_SLEEP_MS = 80;        // ms between alignment adjustments
// Debounce helper for switching from white to black after picking 3rd box
double black_detect_start_time = 0.0;
bool black_detecting = false;
const double BLACK_DETECT_DEBOUNCE_SEC = 0.5; // require ~350ms sustained detection
// After picking the 3rd box, ignore transitions for this many seconds
double pick_complete_time = 0.0;
// Enforce at least 30 seconds of white-follow after picking 3rd box
// Enforce at least 50 seconds of white-follow after picking 3rd box
// TUNE: Change this value for faster testing or to match mission timing
const double PICK_TRANSITION_LOCK_SEC = 25.0; // lockout after pick (seconds)
// Sliding-window averaging for IR sensors (robust transition)
#define IR_AVG_N 5
float ir_hist[5][IR_AVG_N] = {0};
int ir_hist_idx = 0;
float avg_ir[5] = {0};
// previous averaged center value (for gradient check)
float prev_avg_ir3 = 0.0f;
// Candidate sustain debounce: require candidate region to persist before entering transition
double candidate_start_time = 0.0;
const double CAND_SUSTAIN_SEC = 0.22; // ~200ms sustain required
// Transition state timing
double transition_start_time = 0.0;
const double TRANSITION_TIMEOUT_SEC = 0.9;   // abort transition if not confirmed
const double TRANSITION_CONFIRM_SEC = 0.35;  // sustained confirm inside transition state
const float TRANS_WHITE_INV_THRESH = 0.6f;   // inverted threshold for white detection
const float TRANS_BLACK_ENTER = 0.6f;        // center threshold for black entry
const float TRANS_CENTER_DELTA = 0.08f;      // required rise in center average
// Transition correction tuning
float last_transition_correction = 0.0f;
// Make the transition correction less aggressive and smoother to avoid sway
const float TRANS_CORR_ALPHA = 0.12f;   // smoothing factor for correction (low-pass)
const float TRANS_CORRECT_GAIN = 0.45f; // scale for computed correction during transition
const float TRANS_MAX_CORR = 0.12f;     // maximum differential speed adjustment during transition
// ...existing code...

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations
// ----------------------
void* control_loop(void* arg);

/**
 * @brief Establishes connection to the CoppeliaSim server
 */
int connect_to_server(SocketClient* c, const char* ip, int port) {
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        (void)0;
        return 0;
    }
#endif
    
    // Create TCP socket
    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) {
        (void)0;
        return 0;
    }

    // Setup server address structure
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    // Attempt to connect to server
    if (connect(c->sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        (void)0;
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

    // Start the receive thread to handle incoming sensor data
#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return 1;
}

/**
 * @brief Get current time in seconds
 */
double get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

/**
 * @brief Detect color based on RGB values with channel swapping
 */
const char* detect_color(float r, float g, float b) {
    // Apply color channel swapping based on constants
    if (COLOR_SWAP_RB) {
        float temp = r;
        r = b;
        b = temp;
    }
    if (COLOR_SWAP_RG) {
        float temp = r;
        r = g;
        g = temp;
    }
    if (COLOR_SWAP_BG) {
        float temp = b;
        b = g;
        g = temp;
    }

    // Normalize RGB values
    float max_val = fmaxf(fmaxf(r, g), b);
    if (max_val == 0) return "unknown";

    r /= max_val;
    g /= max_val;
    b /= max_val;

    // Color detection thresholds (adjusted for cross-talk)
    if (r > 0.8 && g < 0.3 && b < 0.4) return "red";
    if (r < 0.3 && g > 0.8 && b < 0.4) return "green";
    if (r < 0.3 && g < 0.3 && b > 0.8) return "blue";

    return "unknown";
}

// ...existing code...

/**
 * @brief Follow white line (low sensor values = line detected)
 */
void follow_white_line(float ir1, float ir2, float ir3, float ir4, float ir5, float* left_speed, float* right_speed) {
    float base_speed = 10.5;

    // Calculate error based on sensor positions
    float error = 0.0;
    float total_weight = 0.0;

    // Sensor positions: -2, -1, 0, 1, 2 (normalized)
    float weights[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    float sensors[5] = {ir1, ir2, ir3, ir4, ir5};

    // For white line: invert sensor values (1.0 - sensor)
    for (int i = 0; i < 5; i++) {
        float line_strength = 1.0 - sensors[i]; // Higher values = more on white line
        error += weights[i] * line_strength;
        total_weight += line_strength;
    }

    if (total_weight > 0) {
        error /= total_weight; // Normalize error
    }

    // PID calculation
    integral += error;
    float derivative = error - previous_error;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Apply correction to motor speeds
    *left_speed = base_speed - correction;
    *right_speed = base_speed + correction;

    // Clamp speeds to reasonable range
    if (*left_speed > 1.0) *left_speed = 1.0;
    if (*left_speed < -1.0) *left_speed = -1.0;
    if (*right_speed > 1.0) *right_speed = 1.0;
    if (*right_speed < -1.0) *right_speed = -1.0;
}

/**
 * @brief Follow black line (high sensor values = line detected)
 */
void follow_black_line(float ir1, float ir2, float ir3, float ir4, float ir5, float* left_speed, float* right_speed) {
    float base_speed = 3.0;

    // Calculate error based on sensor positions
    float error = 0.0;
    float total_weight = 0.0;

    // Sensor positions: -2, -1, 0, 1, 2 (normalized)
    float weights[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    float sensors[5] = {ir1, ir2, ir3, ir4, ir5};

    // For black line: use sensor values directly
    for (int i = 0; i < 5; i++) {
        float line_strength = sensors[i]; // Higher values = more on black line
        error += weights[i] * line_strength;
        total_weight += line_strength;
    }

    if (total_weight > 0) {
        error /= total_weight; // Normalize error
    }

    // PID calculation
    integral += error;
    float derivative = error - previous_error;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Apply correction to motor speeds
    *left_speed = base_speed - correction;
    *right_speed = base_speed + correction;

    // Clamp speeds to reasonable range
    if (*left_speed > 1.0) *left_speed = 1.0;
    if (*left_speed < -1.0) *left_speed = -1.0;
    if (*right_speed > 1.0) *right_speed = 1.0;
    if (*right_speed < -1.0) *right_speed = -1.0;
}

/**
 * @brief Detect if robot is at a junction (all sensors detect line)
 */
bool at_junction(float ir1, float ir2, float ir3, float ir4, float ir5) {
    // At a junction, most/all sensors should detect the line
    // For black line: high sensor values indicate line detected
    float threshold = 0.6; // Adjust based on your calibration
    
    int sensors_on_line = 0;
    float sensors[5] = {ir1, ir2, ir3, ir4, ir5};
    
    for (int i = 0; i < 5; i++) {
        if (sensors[i] > threshold) {
            sensors_on_line++;
        }
    }
    
    // Junction detected if at least 4 out of 5 sensors detect the line
    return (sensors_on_line >= 4);
}
bool is_black_line_detected(float ir1, float ir2, float ir3, float ir4, float ir5) {
    // For black line: sensor values are HIGH (>0.6)
    // For white line: sensor values are LOW (<0.4)
    float threshold = 0.6;
    
    int sensors_on_line = 0;
    float sensors[5] = {ir1, ir2, ir3, ir4, ir5};
    
    for (int i = 0; i < 5; i++) {
        if (sensors[i] > threshold) {
            sensors_on_line++;
        }
    }
    
    // Black line detected if at least 3 out of 5 sensors show high values
    return (sensors_on_line >= 3);
}

// Robust junction detection using averaged IR values (reduces noise)
bool at_junction_avg() {
    const float thresh = 0.55f; // averaged threshold
    int sensors_on_line = 0;
    for (int i = 0; i < 5; ++i) {
        if (avg_ir[i] > thresh) sensors_on_line++;
    }
    // Consider a junction when at least 3 sensors read the line (averaged)
    return (sensors_on_line >= 3);
}

// ...existing code...



/**
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    while (c->running) {
        // ========================================
        // LINE FOLLOWING SENSORS (IR SENSORS)
        // ========================================
        // These sensors detect black lines on white surface
        // Values typically range from 0.0 to 1.0
        // Lower values indicate darker surface (line detected)
        // Higher values indicate lighter surface (no line)
        float ir1 = c->line_sensors[0];  // left_corner sensor
        float ir2 = c->line_sensors[1];  // left sensor
        float ir3 = c->line_sensors[2];  // middle sensor (center)
        float ir4 = c->line_sensors[3];  // right sensor
        float ir5 = c->line_sensors[4];  // right_corner sensor

        // Update sliding-window history and compute averaged IRs
        float sensors_tmp[5] = {ir1, ir2, ir3, ir4, ir5};
        for (int si = 0; si < 5; ++si) {
            ir_hist[si][ir_hist_idx] = sensors_tmp[si];
            float ssum = 0.0f;
            for (int k = 0; k < IR_AVG_N; ++k) ssum += ir_hist[si][k];
            avg_ir[si] = ssum / (float)IR_AVG_N;
        }
        // compute center delta (gradient) for transition checks
        float center_delta = avg_ir[2] - prev_avg_ir3;
        prev_avg_ir3 = avg_ir[2];
        ir_hist_idx = (ir_hist_idx + 1) % IR_AVG_N;

         float proximity = c->proximity_distance;
         const char* current_color = detect_color(c->color_r, c->color_g, c->color_b);

        // ========================================
        // STATE MACHINE LOGIC
        // ========================================
        float left_speed = 0.0;
        float right_speed = 0.0;
        double current_time = get_current_time(); // Declare here to avoid scope issues in switch

        switch (current_state) {
            case PICK_THIRD: {
                (void)0;
                // Stop and pick the third box using approach and pick logic from Task2a.c
                left_speed = 0.0;
                right_speed = 0.0;
                set_motor(c, left_speed, right_speed);

                // Constants from Task2a.c
                const float PROX_GRIP = 0.10f; // gripping distance (m)
                const float APPROACH_SPEED = 0.5f;
                const int MAX_PICK_ATTEMPTS = 6;

                // Approach if needed
                float prox = c->proximity_distance;
                if (prox > PROX_GRIP && prox < 1.0f) { // Assume valid if <1m
                    (void)0;
                    int pulses = 0;
                    while (c->running && pulses < 20) {
                        prox = c->proximity_distance;
                        if (prox <= PROX_GRIP) break;
                        set_motor(c, APPROACH_SPEED, APPROACH_SPEED);
                        SLEEP(80);
                        set_motor(c, 0.0f, 0.0f);
                        SLEEP(80);
                        pulses++;
                    }
                }

                // Settle and read color (optional, since already detected)
                SLEEP(180);

                // Attempt pick multiple times
                int pick_ok = 0;
                for (int pa = 0; pa < MAX_PICK_ATTEMPTS && c->running && !pick_ok; ++pa) {
                    pick_ok = pick_box(c);
                    if (!pick_ok) SLEEP(150);
                }

                if (pick_ok) {
                    picked_index = 2; // Third box (0-indexed)
                    strcpy(picked_color, box_colors[2]);
                    // Print only the held color (lowercase)
                    printf("%s\n", picked_color);
                    SLEEP(2000); // Delay to allow picking
                    current_state = WHITE_FOLLOW;  // Continue WHITE_FOLLOW after picking
                    pick_complete_time = get_current_time(); // record time of pick completion
                    integral = 0.0;
                    previous_error = 0.0;
                } else {
                    (void)0;
                    set_motor(c, -0.8f, -0.8f);
                    SLEEP(280); 
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(150);
                    // Reset and try again or handle error
                    current_state = WHITE_FOLLOW; // Or retry
                }
                break;
            }

            case WHITE_FOLLOW:
                (void)0;
                
                // If the 3rd box was picked, use the robust transition logic
                if (picked_index == 2) {
                    (void)0;

                    // Normal white-line following while waiting for a candidate
                    follow_white_line(ir1, ir2, ir3, ir4, ir5, &left_speed, &right_speed);

                    // If lockout elapsed, allow direct averaged-pattern checks to switch
                    // Specific strong-corner pattern: corners high (>0.8) and center low (<0.15)
                    // Example: avg::[0.83,0.83,0.09,0.83,0.83] -> switch to BLACK_FOLLOW after lockout
                    // TUNE: adjust CORNER_HIGH and CENTER_LOW thresholds below
                    const float CORNER_HIGH = 0.80f;
                    const float CENTER_LOW = 0.15f;
                    bool corners_high = (avg_ir[0] > CORNER_HIGH) && (avg_ir[1] > CORNER_HIGH) && (avg_ir[3] > CORNER_HIGH) && (avg_ir[4] > CORNER_HIGH);
                    bool center_low = (avg_ir[2] < CENTER_LOW);
                    if ((current_time - pick_complete_time) >= PICK_TRANSITION_LOCK_SEC && corners_high && center_low) {
                        (void)0;
                        current_state = BLACK_FOLLOW;
                        junction_count = 0;
                        integral = 0.0f;
                        previous_error = 0.0f;
                        last_transition_correction = 0.0f;
                        set_motor(c, 0.0f, 0.0f);
                        SLEEP(200);
                        break; // exit WHITE_FOLLOW handling this loop
                    }

                    // Fallback: majority averaged check (>=3 sensors > 0.80)
                    int avg_black_count = 0;
                    for (int ai = 0; ai < 5; ++ai) {
                        if (avg_ir[ai] > 0.80f) avg_black_count++;
                    }
                    if ((current_time - pick_complete_time) >= PICK_TRANSITION_LOCK_SEC && avg_black_count >= 3) {
                        (void)0;
                        current_state = BLACK_FOLLOW;
                        junction_count = 0;
                        integral = 0.0f;
                        previous_error = 0.0f;
                        last_transition_correction = 0.0f;
                        set_motor(c, 0.0f, 0.0f);
                        SLEEP(200);
                        break; // exit WHITE_FOLLOW handling this loop
                    }

                    // Use averaged IR values to detect a candidate region
                    float w1 = 1.0f - avg_ir[0];
                    float w2 = 1.0f - avg_ir[1];
                    float w4 = 1.0f - avg_ir[3];
                    float w5 = 1.0f - avg_ir[4];
                    float c3 = avg_ir[2];

                    // Candidate thresholds (looser than final confirm)
                    const float CAND_CORNER = 0.55f;
                    const float CAND_CENTER_BLACK = 0.55f;

                    bool candidate = (w1 > CAND_CORNER) && (w2 > CAND_CORNER) && (w4 > CAND_CORNER) && (w5 > CAND_CORNER) && (c3 > CAND_CENTER_BLACK);

                    // Candidate sustain debounce: require the candidate to persist
                    if (candidate) {
                        if (candidate_start_time <= 0.0) candidate_start_time = current_time;
                    } else {
                        candidate_start_time = 0.0;
                    }

                    // Only allow transition attempts after pick lockout AND sustained candidate
                    if ((current_time - pick_complete_time) >= PICK_TRANSITION_LOCK_SEC && candidate_start_time > 0.0 && (current_time - candidate_start_time) >= CAND_SUSTAIN_SEC) {
                        // Enter a dedicated transition state to confirm reliably
                        (void)0;
                        current_state = TRANSITION_TO_BLACK;
                        transition_start_time = current_time;
                        black_detecting = false;
                        black_detect_start_time = 0.0;
                        candidate_start_time = 0.0;
                        // reset PID to avoid aggressive steering during transition
                        integral = 0.0f;
                        previous_error = 0.0f;
                    }
                } else {
                    // Normal box detection phase (before picking)
                    follow_white_line(ir1, ir2, ir3, ir4, ir5, &left_speed, &right_speed);

                    // Detect boxes using proximity sensor
                    if (proximity < 0.15 && !box_detected && (current_time - last_detection_time) > 2.0) {
                        const char* color = detect_color(c->color_r, c->color_g, c->color_b);
                        if (strcmp(color, "unknown") != 0 && box_count < 3) {
                            // Print only the detected color (lowercase)
                            printf("%s\n", color);
                            strcpy(box_colors[box_count], color);
                            send_color(c, color);
                            box_count++;
                            box_detected = true;
                            last_detection_time = current_time;
                        }
                    } else if (proximity > 0.2) {
                        box_detected = false;  // Reset flag when box is far away
                    }

                    // Switch to pick third box after detecting 3 boxes and stop immediately
                    if (box_count >= 3) {
                        current_state = PICK_THIRD;
                        set_motor(c, 0.0, 0.0); // Stop the bot immediately
                    }
                }
                break;

            case DETECT_BOXES:
                // This state is handled in WHITE_FOLLOW now
                current_state = WHITE_FOLLOW;
                break;

            case TRANSITION_TO_BLACK: {
                (void)0;

                // Move slowly forward while applying a small, bounded center correction
                // computed from averaged sensors. This prevents the robot from
                // violently steering left/right on the white background and helps it
                // slide onto the black line smoothly.
                float base_forward = 0.36f; // slightly faster forward speed for transition

                // Compute a white-line error using averaged sensors (invert values)
                float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
                float total_w = 0.0f;
                float err = 0.0f;
                for (int i = 0; i < 5; ++i) {
                    float line_strength = 1.0f - avg_ir[i]; // white-line strength
                    err += weights[i] * line_strength;
                    total_w += line_strength;
                }
                if (total_w > 0.0001f) err /= total_w; else err = 0.0f;

                // Map error to a small differential correction, smooth and clamp it
                float raw_corr = TRANS_CORRECT_GAIN * err;
                // low-pass filter the correction to avoid oscillation
                float corr = last_transition_correction * (1.0f - TRANS_CORR_ALPHA) + raw_corr * TRANS_CORR_ALPHA;
                // clamp
                if (corr > TRANS_MAX_CORR) corr = TRANS_MAX_CORR;
                if (corr < -TRANS_MAX_CORR) corr = -TRANS_MAX_CORR;
                last_transition_correction = corr;

                // apply correction: left = forward - corr, right = forward + corr
                left_speed = base_forward - corr;
                right_speed = base_forward + corr;

                // Use averaged IRs for confirmation
                float w1 = 1.0f - avg_ir[0];
                float w2 = 1.0f - avg_ir[1];
                float w4 = 1.0f - avg_ir[3];
                float w5 = 1.0f - avg_ir[4];
                float c3 = avg_ir[2];
                // Use the precomputed `center_delta` (computed earlier in loop)
                float center_rise = center_delta;

                bool corners_white = (w1 > TRANS_WHITE_INV_THRESH) && (w2 > TRANS_WHITE_INV_THRESH) && (w4 > TRANS_WHITE_INV_THRESH) && (w5 > TRANS_WHITE_INV_THRESH);
                bool center_black = (c3 > TRANS_BLACK_ENTER);
                bool rising = (center_rise > TRANS_CENTER_DELTA);

                // Allow confirmation either via rising edge OR a strong center black
                bool confirm_candidate = corners_white && center_black && (rising || (c3 > (TRANS_BLACK_ENTER + 0.06f)));

                (void)0;

                if (confirm_candidate) {
                    if (!black_detecting) {
                        black_detecting = true;
                        black_detect_start_time = current_time;
                    } else if ((current_time - black_detect_start_time) >= TRANSITION_CONFIRM_SEC) {
                        (void)0;
                        current_state = BLACK_FOLLOW;
                        junction_count = 0;
                        integral = 0.0f;
                        previous_error = 0.0f;
                        last_transition_correction = 0.0f; // clear residual correction
                        set_motor(c, 0.0f, 0.0f);
                        SLEEP(200);
                        black_detecting = false;
                    }
                } else {
                    // Abort or wait: if we've been trying too long, go back to white follow
                    if ((current_time - transition_start_time) > TRANSITION_TIMEOUT_SEC) {
                        (void)0;
                        current_state = WHITE_FOLLOW;
                        black_detecting = false;
                        black_detect_start_time = 0.0;
                        last_transition_correction = 0.0f; // reset correction to avoid carry-over
                    } else {
                        // keep trying; reset short-lived detection
                        black_detecting = false;
                        black_detect_start_time = 0.0;
                    }
                }

                break;
            }

            case BLACK_FOLLOW:{
                (void)0;

                // Determine if we are approaching a junction using averaged IR values
                bool approaching_junction = false;
                if (avg_ir[2] > JUNCTION_APPROACH_CENTER) {
                    int sides = 0;
                    if (avg_ir[1] > JUNCTION_APPROACH_SIDE) sides++;
                    if (avg_ir[3] > JUNCTION_APPROACH_SIDE) sides++;
                    if (avg_ir[0] > JUNCTION_APPROACH_SIDE) sides++;
                    if (avg_ir[4] > JUNCTION_APPROACH_SIDE) sides++;
                    // consider approaching when center is high and at least two sides are elevated
                    if (sides >= 2) approaching_junction = true;
                }

                if (approaching_junction) {
                    // Enter approach smoothing mode: slow forward with small, smoothed correction
                    if (!junction_approach_flag) {
                        // on entry, clear PID integrators to avoid windup and sudden jerks
                        integral = 0.0f;
                        previous_error = 0.0f;
                        junction_approach_flag = true;
                    }

                    // compute averaged error using averaged sensors (black line uses direct values)
                    float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
                    float total_w = 0.0f;
                    float err = 0.0f;
                    for (int i = 0; i < 5; ++i) {
                        float line_strength = avg_ir[i]; // averaged black strength
                        err += weights[i] * line_strength;
                        total_w += line_strength;
                    }
                    if (total_w > 0.0001f) err /= total_w; else err = 0.0f;

                    // small correction scaled and clamped
                    float raw_corr = BLACK_APPROACH_CORR_GAIN * err;
                    if (raw_corr > BLACK_APPROACH_MAX_CORR) raw_corr = BLACK_APPROACH_MAX_CORR;
                    if (raw_corr < -BLACK_APPROACH_MAX_CORR) raw_corr = -BLACK_APPROACH_MAX_CORR;

                    left_speed = BLACK_APPROACH_BASE - raw_corr;
                    right_speed = BLACK_APPROACH_BASE + raw_corr;
                } else {
                    // Normal black line following, but when just after a turn use reduced-PID smoothing
                    junction_approach_flag = false;
                    if ((current_time - post_turn_lock_time) < POST_TURN_LOCK_SEC) {
                        // Enter post-turn reduced PID mode
                        if (!post_turn_flag) {
                            integral = 0.0f;
                            previous_error = 0.0f;
                            post_turn_flag = true;
                        }

                        // Compute error similar to follow_black_line but use averaged/current sensors
                        float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
                        float sensors[5] = {ir1, ir2, ir3, ir4, ir5};
                        float err = 0.0f;
                        float total_weight = 0.0f;
                        for (int i = 0; i < 5; ++i) {
                            float line_strength = sensors[i];
                            err += weights[i] * line_strength;
                            total_weight += line_strength;
                        }
                        if (total_weight > 0.0001f) err /= total_weight; else err = 0.0f;

                        // Reduced PID
                        float scaled_Kp = Kp * POST_TURN_KP_SCALE;
                        float scaled_Ki = Ki * POST_TURN_KI_SCALE;
                        float scaled_Kd = Kd * POST_TURN_KD_SCALE;

                        integral += err;
                        float derivative = err - previous_error;
                        float correction = scaled_Kp * err + scaled_Ki * integral + scaled_Kd * derivative;
                        previous_error = err;

                        left_speed = POST_TURN_BASE - correction;
                        right_speed = POST_TURN_BASE + correction;
                        // Clamp speeds
                        if (left_speed > 1.0f) left_speed = 1.0f;
                        if (left_speed < -1.0f) left_speed = -1.0f;
                        if (right_speed > 1.0f) right_speed = 1.0f;
                        if (right_speed < -1.0f) right_speed = -1.0f;
                    } else {
                        // Normal follow
                        post_turn_flag = false;
                        follow_black_line(ir1, ir2, ir3, ir4, ir5, &left_speed, &right_speed);
                    }
                }

                // Check for junctions using averaged sensors (more robust)
                if (at_junction_avg()) {
                    // Require normal debounce AND that we're not inside a longer lockout window
                    if (!at_junction_flag && (current_time - last_junction_time) >= JUNCTION_DEBOUNCE_SEC
                        && (current_time - junction_lock_time) >= JUNCTION_LOCK_SEC) {
                        junction_count++;
                        at_junction_flag = true;
                        last_junction_time = current_time;
                        if (junction_count == 1) {
                            // First junction: ignore (continue following)
                        } else if (junction_count == 2) {
                            // Second junction: stop, settle, then perform the turn decision
                            set_motor(c, 0.0f, 0.0f);
                            SLEEP(JUNCTION_SETTLE_MS);
                            // Start a longer lockout to avoid detecting this same junction again
                            junction_lock_time = current_time;
                            current_state = DECIDE_TURN;
                        } else if (junction_count == 3) {
                            // Third junction: drop the box
                            current_state = DROP;
                        }
                    }
                } else {
                    at_junction_flag = false;
                }
                break;
            }

            case DECIDE_TURN:{
                (void)0;
                
                // Stop before turning for better alignment
                set_motor(c, 0.0, 0.0);
                SLEEP(200);

                // Small alignment loop: spin in place to reduce center error and side imbalance
                for (int aa = 0; aa < ALIGN_MAX_ATTEMPTS && c->running; ++aa) {
                    // compute averaged-centered error using averaged IRs
                    float weightsA[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
                    float totalA = 0.0f;
                    float errA = 0.0f;
                    for (int i = 0; i < 5; ++i) {
                        float ls = avg_ir[i];
                        errA += weightsA[i] * ls;
                        totalA += ls;
                    }
                    if (totalA > 0.0001f) errA /= totalA; else errA = 0.0f;

                    // side-balance check (left vs right side difference)
                    float left_sum = avg_ir[0] + avg_ir[1];
                    float right_sum = avg_ir[3] + avg_ir[4];
                    float side_diff = left_sum - right_sum;

                    if (fabs(errA) < ALIGN_CENTER_TOL && fabs(side_diff) < ALIGN_SIDE_TOL) {
                        break; // already aligned
                    }

                    // spin to reduce error: negative errA -> rotate left, positive -> rotate right
                    float spin = errA * ALIGN_ADJ_SPEED;
                    if (spin > ALIGN_ADJ_SPEED) spin = ALIGN_ADJ_SPEED;
                    if (spin < -ALIGN_ADJ_SPEED) spin = -ALIGN_ADJ_SPEED;
                    set_motor(c, -spin, spin);
                    SLEEP(ALIGN_SLEEP_MS);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(30);
                }

                // small settle
                set_motor(c, 0.0, 0.0);
                SLEEP(200);

                // Decide turn direction based on picked box color
                // Use a higher turn speed for faster movement but keep duration tuned
                float turn_speed = 0.88f;
                if (strcmp(picked_color, "red") == 0 || strcmp(picked_color, "green") == 0) {
                    // Turn left
                    left_speed = -turn_speed;
                    right_speed = turn_speed;
                    (void)0;
                    set_motor(c, left_speed, right_speed);
                    // Slightly shorter duration due to higher turn speed
                    SLEEP(700);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(140);
                } else if (strcmp(picked_color, "blue") == 0) {
                    // Turn right
                    left_speed = turn_speed;
                    right_speed = -turn_speed;
                    (void)0;
                    set_motor(c, left_speed, right_speed);
                    SLEEP(700);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(140);
                }

                // Stop and settle
                set_motor(c, 0.0, 0.0);
                SLEEP(300);

                // Start post-turn smoothing window and resume black following
                post_turn_lock_time = get_current_time();
                junction_lock_time = post_turn_lock_time; // also protect from immediate junction re-detection
                current_state = BLACK_FOLLOW;
                integral = 0.0;  // Reset PID integral for smooth transition
                previous_error = 0.0;  // Reset PID error
                break;
            }
            
            case DROP:{
                // Stop the robot
                set_motor(c, 0.0, 0.0);
                SLEEP(200);

                // Drop the box (silent)
                drop_box(c);
                SLEEP(500);

                // End the control loop silently
                c->running = false;
                break;
            }

            
        } // end switch (current_state)

        // Set motor speeds for this iteration
        set_motor(c, left_speed, right_speed);
        SLEEP(5); // Control loop delay
    } // end while (c->running)

    // Ensure motors are stopped when loop exits
    set_motor(c, 0.0, 0.0);
    c->running = false;
    return NULL;
}
// ...existing code...

/**
 * @brief Main function - Entry point of the program
 */
int main()
{
    // Attempt to connect to CoppeliaSim server
#if 1
    if (!connect_to_server(&client, "127.0.0.1", 50002)) {
        return -1;
    }

    (void)0; // connection established (silent)

    // Start the control thread for robot behavior
#ifdef _WIN32
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, control_loop, &client);
#endif

    // Main loop: keep process alive; control loop runs separately
    while (1) {
        SLEEP(100);
    }

    // Cleanup (never reached in normal run)
    disconnect(&client);
    return 0;
#endif
}




















































