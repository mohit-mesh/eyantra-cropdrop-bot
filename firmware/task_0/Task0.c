/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Bonus Task 0 of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		task0.c
*  Created:		    19/08/2025
*  Last Modified:	19/08/2025
*  Author:		    e-Yantra Team
*  Team ID:		    [ CB_2202 ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    typedef SOCKET SocketType;
    #define CLOSESOCKET closesocket
    #define READ(s, buf, len) recv(s, buf, len, 0)
    #define SLEEP(ms) Sleep(ms)
    #pragma comment(lib, "Ws2_32.lib")
#else
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <pthread.h>
    typedef int SocketType;
    #define CLOSESOCKET close
    #define READ(s, buf, len) read(s, buf, len)
    #define SLEEP(ms) usleep((ms) * 1000)
#endif

// Structure to hold socket client data and sensor information
typedef struct {
    SocketType sock;                    // Socket handle for communication
    bool running;                       // Flag to control thread execution
    float sensor_values[32];            // Array to store sensor readings (max 32 sensors)
    int sensor_count;                   // Actual number of sensors received
#ifdef _WIN32
    HANDLE recv_thread;                 // Windows thread handle for receiving data
    HANDLE control_thread;              // Windows thread handle for control logic
#else
    pthread_t recv_thread;              // POSIX thread for receiving data
    pthread_t control_thread;           // POSIX thread for control logic
#endif
} SocketClient;

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations
// ----------------------
void* receive_loop(void* arg);          // Thread function to receive sensor data
void* control_loop(void* arg);          // Thread function to handle robot control logic

/**
 * @brief Establishes connection to the CoppeliaSim server
 * @param c Pointer to SocketClient structure
 * @param ip IP address of the server (typically "127.0.0.1" for localhost)
 * @param port Port number of the server (typically 50002)
 * @return 1 if connection successful, 0 if failed
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
 * @brief Cleanly disconnects from the server and cleans up resources
 * @param c Pointer to SocketClient structure
 */
void disconnect(SocketClient* c) {
    c->running = false;  // Signal threads to stop
    
    // Wait for receive thread to finish
#ifdef _WIN32
    WaitForSingleObject(c->recv_thread, INFINITE);
#else
    pthread_join(c->recv_thread, NULL);
#endif
    
    // Close socket if open
    if (c->sock != -1) {
        CLOSESOCKET(c->sock);
        c->sock = -1;
    }
    
    // Cleanup Windows socket library
#ifdef _WIN32
    WSACleanup();
#endif
}

/**
 * @brief Sends motor control commands to the robot
 * @param c Pointer to SocketClient structure
 * @param left Left motor speed (-1.0 to 1.0, where negative values reverse direction)
 * @param right Right motor speed (-1.0 to 1.0, where negative values reverse direction)
 * 
 * Command format: "L:<left_speed>;R:<right_speed>\n"
 * Example: "L:0.5;R:0.3\n" sets left motor to 50% forward, right motor to 30% forward
 */
void set_motor(SocketClient* c, float left, float right) {
    if (c->sock != -1) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "L:%f;R:%f\n", left, right);
        send(c->sock, cmd, strlen(cmd), 0);
    }
}


/**
 * @brief Thread function that continuously receives sensor data from the server
 * @param arg Pointer to SocketClient structure (cast from void*)
 * @return NULL when thread exits
 * 
 * This function runs in a separate thread and parses incoming sensor data.
 * Expected data format: "S:<sensor1>,<sensor2>,<sensor3>,...\n"
 * Example: "S:0.125,0.0,1.0,0.5\n" represents 4 sensor values
 */
void* receive_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    char buffer[2048];
    
    while (c->running) {
        // Read data from socket
        int n = READ(c->sock, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';  // Null-terminate the received string
            
            // Check if this is sensor data (starts with "S:")
            if (strncmp(buffer, "S:", 2) == 0) {
                char* values = buffer + 2;  // Skip the "S:" prefix
                char* token = strtok(values, ",");  // Split by commas
                int idx = 0;
                
                // Parse each sensor value
                while (token && idx < 32) {
                    c->sensor_values[idx++] = (float)atof(token);
                    token = strtok(NULL, ",");
                }
                c->sensor_count = idx;  // Store the number of sensors received
            }
        }
        SLEEP(50);  // Small delay to prevent excessive CPU usage
    }
    return NULL;
}

/**
 * @brief Main control loop thread for robot behavior
 * @param arg Pointer to SocketClient structure (cast from void*)
 * @return NULL when thread exits
 * 
 * This is where you should implement your robot's control logic.
 * The function runs continuously while the client is connected.
 * 
 * Available functions for control:
 * - set_motor(c, left_speed, right_speed): Control motor speeds
 * - Access sensor data via: c->sensor_values[index] and c->sensor_count
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;


    while (c->running) {
        {
        if (c->sensor_count<5){
            set_motor(c,0.0,0.0);
            SLEEP(50);
        }
        float threshold=0.03f;
        int s0=(c->sensor_values[0]<threshold);
        int s1=(c->sensor_values[1]<threshold);
        int s2=(c->sensor_values[2]<threshold);
        int s3=(c->sensor_values[3]<threshold);
        int s4=(c->sensor_values[4]<threshold);
        float right_speed=0.0f, left_speed=0.0f;
        if (s2 && !s1 && !s3){
            left_speed=0.5f, right_speed=0.5f;
        }
        else if (s1&&s2){
            left_speed=0.1f, right_speed=0.6f;
        }
        else if (s2&&s3){
            left_speed=0.6f, right_speed=0.1f;
        }
        else if (s0||s1){
            left_speed=-0.9f, right_speed=0.9f;
        }
        else if (s3||s4){
            left_speed=0.9f, right_speed=-0.9f;    
        }
        else{
            left_speed=0.0f, right_speed=0.0f;
        }
        set_motor(c,left_speed,right_speed);
        }

        SLEEP(2000);  // Wait 2 seconds before next iteration
    }
    return NULL;
}

/**
 * @brief Main function - Entry point of the program
 * @return 0 if successful, -1 if connection failed
 * 
 * This function:
 * 1. Connects to the CoppeliaSim server
 * 2. Starts the control thread for robot behavior
 * 3. Continuously displays sensor data
 * 4. Handles cleanup when program exits
 */
int main() {
    // Attempt to connect to CoppeliaSim server
    // Default: localhost (127.0.0.1) on port 50002
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
    client.control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_create(&client.control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        /*
         * TODO: You can add additional monitoring or logging here
         * 
         * Current functionality:
         * - Displays all received sensor values
         * - Updates every 200ms
         * 
         * You might want to add:
         * - Data logging to file
         * - Specific sensor value monitoring
         * - Performance metrics
         * - User input handling
         */
        
        // Display sensor data if available
        if (client.sensor_count > 0) {
            printf("Sensors (%d): ", client.sensor_count);
            for (int i = 0; i < client.sensor_count; i++) {
                printf("%.3f ", client.sensor_values[i]);
            }
            printf("\n");
        } else {
            printf("Waiting for sensor data...\n");
        }
        
        SLEEP(200);  // Update display every 200ms
    }

    // Cleanup (this code won't be reached due to infinite loop above)
    // In a real application, you'd have a way to break the loop
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}