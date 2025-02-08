#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define UART_PORT "/dev/serial0"
#define BUFFER_SIZE 256



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




void read_lora_data(int uart_fd) {
    char buffer[BUFFER_SIZE];
    int bytes_read;

    while (1) {
        memset(buffer, 0, BUFFER_SIZE);
        bytes_read = read(uart_fd, buffer, BUFFER_SIZE - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("Alınan Veri: %s\n", buffer);

            // paket bütünlüğünü kontrol 
            if (strstr(buffer, "#START#") && strstr(buffer, "#END#")) {
                char gps[128];
                float altitude, pressure;
                


                if (sscanf(buffer, "#START#GPS:%[^ ] ALT:%f PRES:%f#END#", gps, &altitude, &pressure) == 3) {
                    printf("GPS: %s\n", gps);
                    printf("İrtifa: %.2f metre\n", altitude);
                    printf("Basınç: %.2f hPa\n", pressure);
                } else {
                    printf("Geçersiz veri formatı\n");
                }
            } else {
                printf("Eksik veya bozuk veri paketi \n");
            }
        }
        usleep(500000);
        
    }
}




int main() {
    int uart_fd = init_uart();
    if (uart_fd == -1) {
        return -1;
    }


    printf("LoRa alıcı başlatıldı, veri bekleniyor\n");
    read_lora_data(uart_fd);

    close(uart_fd);
    return 0;

}
