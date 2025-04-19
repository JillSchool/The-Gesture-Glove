#include <webots/device.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#ifdef _WIN32
    #include <winsock.h>
#else
    #include <arpa/inet.h>  /* definition of inet_ntoa */
    #include <netdb.h>      /* definition of gethostbyname */
    #include <netinet/in.h> /* definition of struct sockaddr_in */
    #include <sys/socket.h>
    #include <sys/time.h>
    #include <unistd.h> /* definition of close */
#endif

#define SOCKET_PORT               (10020)
#define TIME_STEP                 (50)
#define SPEED_MAX                 (120.0)
#define SPEED_MIN                 (-40.0)
#define STEERING_ANGLE_STEP       (0.1)
#define STEERING_ANGLE_MAX        (0.5)
#define STEERING_ANGLE_MIN        (-0.5)
#define STEERING_ANGLE_MULTIPLIER (0.02)
#define USE_STEERING_ANGLE_LIMITS (0)

// misc variables
double speed           = 0.0;
double steering_angle  = 0.0;
int    manual_steering = 0;

static int    fd;
static fd_set rfds;

static int accept_client(int server_fd)
{
    int                cfd;
    struct sockaddr_in client;
#ifndef _WIN32
    socklen_t asize;
#else
    int asize;
#endif
    const struct hostent* client_info;

    asize = sizeof(struct sockaddr_in);

    cfd = accept(server_fd, (struct sockaddr*)&client, &asize);
    if (cfd == -1)
    {
        printf("cannot accept client\n");
        return -1;
    }
    client_info = gethostbyname((char*)inet_ntoa(client.sin_addr));
    printf("Accepted connection from: %s \n", client_info->h_name);

    return cfd;
}

static int create_socket_server(int port)
{
    int                sfd, rc;
    struct sockaddr_in address;

#ifdef _WIN32
    /* initialize the socket api */
    WSADATA info;

    rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
    if (rc != 0)
    {
        printf("cannot initialize Winsock\n");
        return -1;
    }
#endif
    /* create the socket */
    sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd == -1)
    {
        printf("cannot create socket\n");
        return -1;
    }

    /* fill in socket address */
    memset(&address, 0, sizeof(struct sockaddr_in));
    address.sin_family      = AF_INET;
    address.sin_port        = htons((unsigned short)port);
    address.sin_addr.s_addr = INADDR_ANY;

    /* bind to port */
    rc = bind(sfd, (struct sockaddr*)&address, sizeof(struct sockaddr));
    if (rc == -1)
    {
        printf("cannot bind port %d\n", port);
#ifdef _WIN32
        closesocket(sfd);
#else
        close(sfd);
#endif
        return -1;
    }

    /* listen for connections */
    if (listen(sfd, 1) == -1)
    {
        printf("cannot listen for connections\n");
#ifdef _WIN32
        closesocket(sfd);
#else
        close(sfd);
#endif
        return -1;
    }
    printf("Waiting for a connection on port %d...\n", port);

    return accept_client(sfd);
}

void print_help()
{
    printf("[A,value] - [A]ngle: Positive is steer right, negative is steer left, 0 means go straight\n");
    printf("[D,value] - [D]rive: Positive value is forward, negative value is backwards, 0 means stop\n");
    printf("[P]       - [P]ark : start braking\n");
}

// set target speed
void set_speed(double kmh)
{
    // max speed
    if (kmh > SPEED_MAX) { kmh = SPEED_MAX; }
    else if (kmh < SPEED_MIN) { kmh = SPEED_MIN; }

    speed = kmh;

    printf("setting speed to %g km/h (%s)\n", kmh, kmh > 0 ? "forward" : (kmh < 0 ? "backward" : "parking"));
    wbu_driver_set_cruising_speed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle_deg)
{
    double wheel_angle = wheel_angle_deg * M_PI / 180.0;  // convert to radians

#if USE_STEERING_ANGLE_LIMITS
    // limit the difference with previous steering_angle
    if (wheel_angle - steering_angle > STEERING_ANGLE_STEP) { wheel_angle = steering_angle + STEERING_ANGLE_STEP; }
    if (wheel_angle - steering_angle < -STEERING_ANGLE_STEP) { wheel_angle = steering_angle - STEERING_ANGLE_STEP; }
    steering_angle = wheel_angle;
    // limit range of the steering angle
    if (wheel_angle > STEERING_ANGLE_MAX) { wheel_angle = STEERING_ANGLE_MAX; }
    else if (wheel_angle < STEERING_ANGLE_MIN) { wheel_angle = STEERING_ANGLE_MIN; }
#else
    steering_angle = wheel_angle;
#endif

    // set the steering angle
    if (steering_angle == 0) { printf("going straight\n"); }
    else { printf("turning %.2f deg (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right"); }
    wbu_driver_set_steering_angle(steering_angle);
}

void change_manual_steer_angle(int inc)
{
    double new_manual_steering = manual_steering + inc;
    if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0)
    {
        manual_steering = new_manual_steering;
        set_steering_angle(manual_steering * STEERING_ANGLE_MULTIPLIER);
    }

    if (manual_steering == 0) { printf("going straight\n"); }
    else { printf("turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right"); }
}

static void initialize()
{
    // start engine
    set_speed(0.0);  // km/h
    wbu_driver_set_hazard_flashers(true);
    wbu_driver_set_dipped_beams(true);
    wbu_driver_set_antifog_lights(true);
    wbu_driver_set_wiper_mode(SLOW);

    print_help();

    fd = create_socket_server(SOCKET_PORT);
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    // allow to switch to manual control
    wb_keyboard_enable(TIME_STEP);
}

/* Communication Protocol:
 * A: Steering [A]ngle
 * P: [P]ark, stop the car
 * D: [D]rive, start driving at given speed
 */
void command_protocol_decode(char* buffer)
{
    int   ret;
    char* command = strtok(buffer, ";");  // Split commands by semicolon

    while (command != NULL)
    {
        printf("command: %s\n", command);

        char cmd = toupper(command[0]);
        if (cmd == 'P')
        {
            /* Park */
            manual_steering = 0;
            set_speed(0.0);
            send(fd, command, strlen(command), 0);
        }
        else if (cmd == 'A')
        {
            /* Steering Angle */
            float input_value = 0.0;
            sscanf(command, "A,%f", &input_value);
            set_steering_angle(input_value);
            send(fd, command, strlen(command), 0);
        }
        else if (cmd == 'D')
        {
            /* Drive */
            float input_value = 0.0;
            sscanf(command, "D,%f", &input_value);
            set_speed(input_value);
            send(fd, command, strlen(command), 0);
        }
        else if (strncmp(command, "exit", 4) == 0)
        {
            printf("connection closed\n");
#ifdef _WIN32
            closesocket(fd);
            ret = WSACleanup();
#else
            ret = close(fd);
#endif
            if (ret != 0) { printf("Cannot close socket\n"); }
            fd = 0;
        }
        else { send(fd, "\n", 1, 0); }

        command = strtok(NULL, ";");  // Get the next command
    }
}

static void run()
{
    int            n;
    char           buffer[256];
    struct timeval tv = {0, 0};
    int            number;

    /* Set up the parameters used for the select statement */

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    /*
     * Watch TCPIP file descriptor to see when it has input.
     * No wait - polling as fast as possible
     */
    number = select(fd + 1, &rfds, NULL, NULL, &tv);

    /* If there is no data at the socket, then redo loop */
    if (number == 0) { return; }

    /* ...otherwise, there is data to read, so read & process. */
    n = recv(fd, buffer, 256, 0);
    if (n < 0)
    {
        printf("error reading from socket\n");
        return;
    }
    buffer[n] = '\0';
    printf("Received %d bytes: %s\n", n, buffer);

    command_protocol_decode(buffer);
}

int main(int argc, char** argv)
{
    wbu_driver_init();

    initialize();

    // main loop
    while (wbu_driver_step() != -1) { run(); }

    wbu_driver_cleanup();

    return 0;  // ignored
}
