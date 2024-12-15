#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// Pin definitions
#define UART_SEL0 22
#define UART_SEL1 17
#define ULTRASONIC_PWR_EN 6

// Function to initialize UART
int init_uart(const char *port, int baudrate) {
    int uart_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("Unable to open UART");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);

    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;  // 8-bit, local connection, enable receiver
    options.c_iflag = IGNPAR;                          // Ignore parity errors
    options.c_oflag = 0;                               // Raw output
    options.c_lflag = 0;                               // Raw input

    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);

    return uart_fd;
}

// Function to send a command and read the response
void send_message(int uart_fd, const char *command, char *response, size_t response_size, int timeout) {
    char command_with_newline[256];
    snprintf(command_with_newline, sizeof(command_with_newline), "%s\r", command);
    write(uart_fd, command_with_newline, strlen(command_with_newline));

    usleep(timeout * 1000000);  // Convert seconds to microseconds

    int bytes_read = read(uart_fd, response, response_size - 1);
    if (bytes_read > 0) {
        response[bytes_read] = '\0';
    } else {
        strcpy(response, "No response");
    }
}

int main() {
    // Initialize wiringPi
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Failed to initialize wiringPi\n");
        return 1;
    }

    // Set up GPIO pins
    pinMode(UART_SEL0, OUTPUT);
    pinMode(UART_SEL1, OUTPUT);
    pinMode(ULTRASONIC_PWR_EN, OUTPUT);

    // Disable power to ultrasonic sensors
    digitalWrite(ULTRASONIC_PWR_EN, LOW);
    digitalWrite(UART_SEL1, LOW);
    digitalWrite(UART_SEL0, LOW);

    // Open UART
    int uart_fd = init_uart("/dev/ttyAMA5", B9600);
    if (uart_fd == -1) {
        return 1;
    }

    // Sensor 1 communication
    char response[256];
    int count = 0, loop = 2;

    while (count < loop) {
        send_message(uart_fd, "test_1", response, sizeof(response), 1);
        printf("Read from UART5 Sensor 1: %s\n", response);
        usleep(1500000);  // 1.5 seconds
        count++;
    }

    // Select Sensor 2
    digitalWrite(UART_SEL1, LOW);
    digitalWrite(UART_SEL0, HIGH);

    count = 0;
    while (count < loop) {
        send_message(uart_fd, "test_2", response, sizeof(response), 1);
        printf("Read from UART5 Sensor 2: %s\n", response);
        usleep(1500000);  // 1.5 seconds
        count++;
    }

    // Close UART
    close(uart_fd);

    return 0;
}
