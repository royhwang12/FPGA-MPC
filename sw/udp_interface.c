#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

#define SIMULATOR_PORT 12345
#define MPC_PORT 12346
#define BUFFER_SIZE 1024

// Structure to hold the quadrotor state
typedef struct {
    float position[3];    // x, y, z
    float quaternion[4];  // w, x, y, z
    float velocity[3];    // vx, vy, vz
    float angular_vel[3]; // wx, wy, wz
} QuadState;

// Structure to hold control inputs
typedef struct {
    float motor_cmd[4];  // Motor commands
} ControlInput;

// Function prototypes
int setup_udp_socket(int port, struct sockaddr_in *addr, const char *ip);
int receive_state(int sock_fd, struct sockaddr_in *server_addr, QuadState *state);
int send_control(int sock_fd, struct sockaddr_in *server_addr, const ControlInput *control);
void compute_mpc_control(const QuadState *state, ControlInput *control);

int main(int argc, char *argv[]) {
    // Default simulator IP
    char *simulator_ip = "127.0.0.1";
    
    // Parse command line arguments
    if (argc > 1) {
        simulator_ip = argv[1];
    }
    
    // Setup socket and address structures
    int sock_fd;
    struct sockaddr_in server_addr, client_addr;
    
    // Initialize UDP socket
    sock_fd = setup_udp_socket(MPC_PORT, &client_addr, "0.0.0.0");
    if (sock_fd < 0) {
        fprintf(stderr, "Failed to setup UDP socket\n");
        return 1;
    }
    
    // Setup server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SIMULATOR_PORT);
    
    if (inet_pton(AF_INET, simulator_ip, &server_addr.sin_addr) <= 0) {
        fprintf(stderr, "Invalid simulator IP address\n");
        close(sock_fd);
        return 1;
    }
    
    printf("UDP Interface initialized - connecting to simulator at %s:%d\n", 
           simulator_ip, SIMULATOR_PORT);
    
    // Initialize state and control structures
    QuadState state;
    ControlInput control;
    
    // Main control loop
    while (1) {
        // Receive current state from simulator
        if (receive_state(sock_fd, &server_addr, &state) > 0) {
            // Print received state for debugging
            printf("Position: [%.2f, %.2f, %.2f]\n", 
                   state.position[0], state.position[1], state.position[2]);
            
            // Compute control input using MPC
            compute_mpc_control(&state, &control);
            
            // Send control back to simulator
            if (send_control(sock_fd, &server_addr, &control) < 0) {
                fprintf(stderr, "Failed to send control\n");
            } else {
                printf("Sent control: [%.2f, %.2f, %.2f, %.2f]\n",
                       control.motor_cmd[0], control.motor_cmd[1], 
                       control.motor_cmd[2], control.motor_cmd[3]);
            }
        }
        
        // Short sleep to prevent tight loop
        usleep(10000); // 10ms
    }
    
    close(sock_fd);
    return 0;
}

int setup_udp_socket(int port, struct sockaddr_in *addr, const char *ip) {
    int sock_fd;
    
    // Create UDP socket
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        return -1;
    }
    
    // Set socket to non-blocking
    int flags = fcntl(sock_fd, F_GETFL, 0);
    fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK);
    
    // Configure address structure
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    
    if (inet_pton(AF_INET, ip, &addr->sin_addr) <= 0) {
        perror("Invalid address");
        close(sock_fd);
        return -1;
    }
    
    // Bind socket to the specified port
    if (bind(sock_fd, (const struct sockaddr *)addr, sizeof(*addr)) < 0) {
        perror("bind failed");
        close(sock_fd);
        return -1;
    }
    
    return sock_fd;
}

int receive_state(int sock_fd, struct sockaddr_in *server_addr, QuadState *state) {
    char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(*server_addr);
    
    // Receive data from the socket
    int n = recvfrom(sock_fd, buffer, BUFFER_SIZE, 0, 
                    (struct sockaddr *)server_addr, &len);
                    
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("recvfrom failed");
        }
        return -1;
    }
    
    // Expected size: 12 floats (3 position + 4 quaternion + 3 velocity + 3 angular velocity)
    if (n != 12 * sizeof(float)) {
        fprintf(stderr, "Received invalid data size: %d bytes\n", n);
        return -1;
    }
    
    // Parse received data - convert from network byte order
    float *float_data = (float *)buffer;
    for (int i = 0; i < 3; i++) {
        state->position[i] = ntohf(float_data[i]);
    }
    
    for (int i = 0; i < 4; i++) {
        state->quaternion[i] = ntohf(float_data[3 + i]);
    }
    
    for (int i = 0; i < 3; i++) {
        state->velocity[i] = ntohf(float_data[7 + i]);
    }
    
    for (int i = 0; i < 3; i++) {
        state->angular_vel[i] = ntohf(float_data[10 + i]);
    }
    
    return n;
}

int send_control(int sock_fd, struct sockaddr_in *server_addr, const ControlInput *control) {
    // Prepare buffer for sending
    float buffer[4];
    
    // Convert to network byte order
    for (int i = 0; i < 4; i++) {
        buffer[i] = htonf(control->motor_cmd[i]);
    }
    
    // Send data to server
    int n = sendto(sock_fd, buffer, sizeof(buffer), 0,
                 (struct sockaddr *)server_addr, sizeof(*server_addr));
                 
    if (n < 0) {
        perror("sendto failed");
        return -1;
    }
    
    return n;
}

// Utility functions for float network byte order conversion
// Note: These are not standard functions and would need to be implemented
float ntohf(float netf) {
    uint32_t netl;
    memcpy(&netl, &netf, sizeof(netl));
    uint32_t hostl = ntohl(netl);
    float hostf;
    memcpy(&hostf, &hostl, sizeof(hostf));
    return hostf;
}

float htonf(float hostf) {
    uint32_t hostl;
    memcpy(&hostl, &hostf, sizeof(hostl));
    uint32_t netl = htonl(hostl);
    float netf;
    memcpy(&netf, &netl, sizeof(netf));
    return netf;
}

// Example implementation - in real use, this would call your MPC solver
void compute_mpc_control(const QuadState *state, ControlInput *control) {
    // Example hover controller using constants from the Python implementation
    const float mass = 0.035;
    const float g = 9.81;
    const float scale = 65535;
    const float kt = 2.245365e-6 * scale;
    
    // Calculate hover thrust
    float hover_thrust = (mass * g / kt / 4.0);
    
    // Set hover thrust for all motors
    for (int i = 0; i < 4; i++) {
        control->motor_cmd[i] = hover_thrust;
    }
    
    // In a real implementation, this would call your hardware MPC solver
    // This might involve:
    // 1. Converting state to the format needed by your MPC solver
    // 2. Calling the solver (e.g., admm.c functions)
    // 3. Converting the output to motor commands
}
