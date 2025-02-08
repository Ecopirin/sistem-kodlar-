#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define UART_PORT "/dev/serial0"  //rpi UART portu
#define MAX_RETRY 5               //maksimum retry 

//UART başlatma fonksiyonu
int init_uart() {
    int uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("UART bağlantısı açılamadı");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);

    return uart_fd;
}

//veriyi gönder
int send_lora_data(int uart_fd, char *data) {
    int attempts = 0;
    while (attempts < MAX_RETRY) {
        int bytes_written = write(uart_fd, data, strlen(data));
        if (bytes_written < 0) {
            perror("veri gönderme hatası, yeniden dene");
            attempts++;
            sleep(1);
        } else {
            printf("veri başarıyla gönderildi: %s\n", data);
            return 0;
        }
    }
    return -1;  //başarısız olduysa -1 döndür
}

int main() {
    int uart_fd = init_uart();
    if (uart_fd == -1) {
        return -1;
    }

    char buffer[256];
    float altitude = altitude;
    float pressure = current_pressure;
    char gps[] = "40.7128N,74.0060W";

    snprintf(buffer, sizeof(buffer), "#START#GPS:%s ALT:%.2f PRES:%.2f#END#", gps, altitude, pressure);

    if (send_lora_data(uart_fd, buffer) == -1) {
        printf("veri gönderilemedi\n");
    }

    close(uart_fd);
    return 0;
}
