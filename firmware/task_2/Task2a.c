// ...existing code...
/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 2a of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Task2a.c
*  Created:		    19/08/2025
*  Last Modified:	12/11/2025
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
#include <stdio.h>

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
        printf("WSAStartup failed\n");
        return 0;
    }
#endif
    
    // Create TCP socket
    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) {
        printf("Socket creation failed\n");
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
        printf("Connection failed\n");
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
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;

    // PID params (tune these if needed)
    const float BASE_SPEED = 4.5f;   // base forward speed (left/right)
    const float KP = 8.6f;
    const float KI = 0.001f;
    const float KD = 3.3f;
    const int LOOP_DELAY_MS = 30;    // main loop delay

    // proximity / pick/drop thresholds & timings
    const float PROX_DETECT = 0.35f; // detection distance (m)
    const float PROX_GRIP   = 0.10f; // gripping distance (m)
    const float NODE_SUM_THRESH = 3.4f; // sum of inverted sensors at junction
    const float NODE_SUM_CLEAR = 1.2f;  // require sum to drop below this to clear a junction
    const int TURN_MS = 600;
    const int FORWARD_MS = 1100;
    const float TURN_SPEED = 4.7f;
    const float FORWARD_SPEED = 1.4f;
    const float APPROACH_SPEED = 0.5f;

    // Simulator reports blue in color_r and red in color_b, set to 1 to swap R<->B
    const int COLOR_SWAP_RB = 1; // 0 = no swap, 1 = swap R and G channels
    const int COLOR_SWAP_RG = 1;
    const int COLOR_SWAP_BG = 0;

    float previous_error = 0.0f;
    float integral = 0.0f;
    bool pause_pid = false;   // when true, PID won't send motor commands
    bool carrying = false;
    char carried_color[16] = "none";

    // routing state: 0 = waiting for first junction, 1 = first junction passed; drop at next
    int routing_stage = 0;

    // ignore junctions until this time (seconds) after pickup/turn so we don't re-detect same node
    double node_skip_until = 0.0;

    // debounce for junction detection (avoid immediate re-detect)
    bool junction_active = false;

    // scales for safer line-follow while carrying
    const float CARRY_SPEED_SCALE = 0.85f;
    const float CARRY_CORR_SCALE = 0.50f;

    while (c->running) {
        // Read raw sensors (0..1). For black-line the line is darker -> smaller value,
        // so invert so line => large value (1.0)
        float s0 = c->line_sensors[0];
        float s1 = c->line_sensors[1];
        float s2 = c->line_sensors[2];
        float s3 = c->line_sensors[3];
        float s4 = c->line_sensors[4];

        float lc = 1.0f - s0;
        float l  = 1.0f - s1;
        float m  = 1.0f - s2;
        float r  = 1.0f - s3;
        float rc = 1.0f - s4;

        // Debug occasionally
        static int dbg_count = 0;
        if ((++dbg_count % 200) == 0) {
            printf("LINE RAW: [%.3f %.3f %.3f %.3f %.3f] (inv) prox=%.3f carrying=%d routing_stage=%d\n",
                   s0,s1,s2,s3,s4, c->proximity_distance, carrying, routing_stage);
        }

        // -------------------------
        // PID controller (apply reduced aggressiveness while carrying)
        // -------------------------
        float numerator = (-2.0f * lc) + (-1.0f * l) + (0.0f * m) + (1.0f * r) + (2.0f * rc);
        float denominator = lc + l + m + r + rc;
        float error = 0.0f;
        if (denominator > 0.001f) error = numerator / denominator;

        integral += error;
        float derivative = error - previous_error;
        float correction = (KP * error) + (KI * integral) + (KD * derivative);
        previous_error = error;

        // Apply scaling when carrying to reduce oscillation
        float speed_scale = carrying ? CARRY_SPEED_SCALE : 1.0f;
        float corr_scale = carrying ? CARRY_CORR_SCALE : 1.0f;

        float left_speed = BASE_SPEED * speed_scale + correction * corr_scale;
        float right_speed = BASE_SPEED * speed_scale - correction * corr_scale;

        // Clamp speeds to reasonable range (non-negative forward)
        const float MAX_SPEED = 2.8f;
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;

        // Only send PID motor commands when not paused for pickup/routing
        if (!pause_pid) {
            set_motor(c, left_speed, right_speed);
        } else {
            // keep motors stopped while paused
            set_motor(c, 0.0f, 0.0f);
        }

        // -------------------------
        // Box detection / pickup
        // -------------------------
        float prox = c->proximity_distance;
        bool prox_valid = (prox > 1e-6f && prox < 10.0f);
        bool detected = prox_valid && (prox < PROX_DETECT);

        if (!carrying && detected) {
            // Pause PID so it doesn't overwrite our stop/approach commands
            pause_pid = true;
            routing_stage = 0; // reset routing state on new pickup
            printf("Box detected (prox=%.3f) — pausing PID and attempting pickup\n", prox);

            // full stop
            set_motor(c, 0.0f, 0.0f);
            SLEEP(150);

            // gentle approach in short pulses until within grip distance
            if (prox_valid && prox > PROX_GRIP) {
                int pulses = 0;
                while (c->running && pulses < 20) {
                    prox = c->proximity_distance;
                    if (!(prox > 1e-6f)) break;
                    if (prox <= PROX_GRIP) break;
                    set_motor(c, APPROACH_SPEED, APPROACH_SPEED);
                    SLEEP(80);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(80);
                    pulses++;
                }
            }

            // settle and read color median-only (no ambient subtraction)
            SLEEP(180);
            #define COL_SAMPLES 25
            float r_samples[COL_SAMPLES], g_samples[COL_SAMPLES], b_samples[COL_SAMPLES];
            for (int i = 0; i < COL_SAMPLES; ++i) {
                r_samples[i] = c->color_r;
                g_samples[i] = c->color_g;
                b_samples[i] = c->color_b;
                SLEEP(20);
            }
            // sort and median
            for (int i = 0; i < COL_SAMPLES-1; ++i) {
                int mi = i;
                for (int j = i+1; j < COL_SAMPLES; ++j) if (r_samples[j] < r_samples[mi]) mi = j;
                float tmp = r_samples[i]; r_samples[i] = r_samples[mi]; r_samples[mi] = tmp;
                mi = i; for (int j = i+1; j < COL_SAMPLES; ++j) if (g_samples[j] < g_samples[mi]) mi = j;
                tmp = g_samples[i]; g_samples[i] = g_samples[mi]; g_samples[mi] = tmp;
                mi = i; for (int j = i+1; j < COL_SAMPLES; ++j) if (b_samples[j] < b_samples[mi]) mi = j;
                tmp = b_samples[i]; b_samples[i] = b_samples[mi]; b_samples[mi] = tmp;
            }
            float med_r = r_samples[COL_SAMPLES/2];
            float med_g = g_samples[COL_SAMPLES/2];
            float med_b = b_samples[COL_SAMPLES/2];

            // Simple median-only classification (no ambient)
            if (COLOR_SWAP_RB) { float tmp = med_r; med_r = med_b; med_b = tmp; }
            if (COLOR_SWAP_RG) { float tmp = med_r; med_r = med_g; med_g = tmp; }
            if (COLOR_SWAP_BG) { float tmp = med_b; med_b = med_g; med_g = tmp; }

            float total_med = med_r + med_g + med_b + 1e-6f;
            float chr = med_r / total_med;
            float chg = med_g / total_med;
            float chb = med_b / total_med;

            const float DOM_THRESHOLD = 0.45f;
            const char* color = "unknown";
            if (total_med < 0.01f) {
                color = "unknown";
            } else if (chr >= DOM_THRESHOLD && chr > chg && chr > chb) {
                color = "red";
            } else if (chg >= DOM_THRESHOLD && chg > chr && chg > chb) {
                color = "green";
            } else if (chb >= DOM_THRESHOLD && chb > chr && chb > chg) {
                color = "blue";
            } else {
                // fallback: pick largest channel
                if (chr >= chg && chr >= chb) color = "red";
                else if (chg >= chr && chg >= chb) color = "green";
                else color = "blue";
            }

            // debug print
            printf("Color med RGB=(%.3f,%.3f,%.3f) chr=(%.3f,%.3f,%.3f) => %s\n",
                   med_r, med_g, med_b, chr, chg, chb, color);

            // ensure motors stopped before pick
            set_motor(c, 0.0f, 0.0f);
            SLEEP(120);

            // attempt pick (try multiple times)
            int pick_ok = 0;
            const int MAX_PICK_ATTEMPTS = 6;
            for (int pa=0; pa<MAX_PICK_ATTEMPTS && c->running && !pick_ok; ++pa) {
                pick_ok = pick_box(c); // expected to block/return status
                if (!pick_ok) SLEEP(150);
            }

            if (!pick_ok) {
                printf("pick_box failed — backing off and resuming\n");
                set_motor(c, -0.8f, -0.8f);
                SLEEP(280);
                set_motor(c, 0.0f, 0.0f);
                SLEEP(150);
                // reset PID integral to avoid sudden spike
                integral = 0.0f;
                previous_error = 0.0f;
                pause_pid = false;
            } else {
                // success
                carrying = true;
                strncpy(carried_color, color, sizeof(carried_color)-1);
                carried_color[sizeof(carried_color)-1] = '\0';
                routing_stage = 0; // waiting to detect first junction
                node_skip_until = get_current_time() + 0.9;
                junction_active = false;
                printf("Picked box color=%s, ignoring junctions until %.3f\n", carried_color, node_skip_until);

                // small forward to clear pickup area
                set_motor(c, FORWARD_SPEED, FORWARD_SPEED);
                SLEEP(350);
                set_motor(c, 0.0f, 0.0f);
                SLEEP(120);

                // resume PID (reset integrator to avoid overshoot)
                integral = 0.0f;
                previous_error = 0.0f;
                pause_pid = false;
            }

            // continue loop
            continue;
        }

        // -------------------------
        // When carrying: detect junction and route
        // -------------------------
        if (carrying) {
            double now = get_current_time();

            // require either time skip or cleared node to continue
            float sum_inv = lc + l + m + r + rc;

            // detect rising edge of junction (debounced)
            if (sum_inv >= NODE_SUM_THRESH && !junction_active && now >= node_skip_until) {
                junction_active = true;
                // handle junction event below
            }

            // clear active flag when line sum drops sufficiently
            if (sum_inv < NODE_SUM_CLEAR) junction_active = false;

            if (junction_active && sum_inv >= NODE_SUM_THRESH) {
                // Junction detected (on rising edge)
                set_motor(c, 0.0f, 0.0f);
                SLEEP(150);
                printf("Junction detected (stage=%d), carried color=%s\n", routing_stage, carried_color);

                if (routing_stage == 0) {
                    // First junction handling:
                    if (strcmp(carried_color, "red") == 0) {
                        // turn left at first junction
                        set_motor(c, -TURN_SPEED, TURN_SPEED);
                        SLEEP(TURN_MS);
                        set_motor(c, 0.0f, 0.0f);
                        SLEEP(80);
                    } else if (strcmp(carried_color, "green") == 0) {
                        // turn right at first junction
                        set_motor(c, TURN_SPEED, -TURN_SPEED);
                        SLEEP(80);
                        set_motor(c, 0.0f, 0.0f);
                        SLEEP(TURN_MS);
                    } else {
                        // blue -> go straight through first junction (no turn)
                        SLEEP(80);
                    }

                    // drive forward a bit to clear this junction and set stage to wait for next
                    set_motor(c, FORWARD_SPEED, FORWARD_SPEED);
                    SLEEP(500);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(120);

                    // set skip time and routing stage
                    node_skip_until = get_current_time() + 0.6;
                    routing_stage = 1;
                    junction_active = false; // require re-detection later
                    printf("First junction handled, will drop at next junction (skip until %.3f)\n", node_skip_until);
                } else if (routing_stage == 1) {
                    // This is the next junction -> drop here
                    printf("Reached drop junction for color=%s — attempting drop\n", carried_color);

                    // move a bit into drop zone if needed
                    set_motor(c, FORWARD_SPEED, FORWARD_SPEED);
                    SLEEP(300);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(120);

                    int ok = drop_box(c);
                    if (ok) printf("Dropped color=%s\n", carried_color);
                    else printf("drop_box failed for color=%s\n", carried_color);

                    // back off and resume line following
                    set_motor(c, -1.0f, -1.0f);
                    SLEEP(320);
                    set_motor(c, 0.0f, 0.0f);
                    SLEEP(120);

                    // reset carrying and routing
                    carrying = false;
                    strncpy(carried_color, "none", sizeof(carried_color));
                    routing_stage = 0;
                    integral = 0.0f;
                    previous_error = 0.0f;
                    junction_active = false;
                }
            }
        }

        SLEEP(LOOP_DELAY_MS);
    }

    // ensure motors stopped when exiting
    set_motor(c, 0.0f, 0.0f);
    return NULL;
}
// ...existing code...

/**
 * @brief Main function - Entry point of the program
 */
int main() {
    // Attempt to connect to CoppeliaSim server
    if (!connect_to_server(&client, "127.0.0.1", 50002)) {
        printf("Failed to connect to CoppeliaSim server. Make sure:\n");
        printf("1. CoppeliaSim is running\n");
        printf("2. The simulation scene is loaded\n");
        printf("3. The ZMQ remote API is enabled on port 50002\n");
        return -1;
    }
    
    printf("Successfully connected to CoppeliaSim server!\n");
    printf("Starting control thread...\n");
    
    // Start the control thread for robot behavior
#ifdef _WIN32
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        SLEEP(100);  // Update display every 100ms
    }

    // Cleanup
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}