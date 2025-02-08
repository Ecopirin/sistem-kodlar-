#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define UART_PORT "/dev/serial0"
#define BUFFER_SIZE 256

// UART bağlantısını başlatan fonksiyon
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

// UART üzerinden gelen veriyi okuyan ve ayrıştıran fonksiyon
void read_lora_data(int uart_fd) {
    char buffer[BUFFER_SIZE];
    int bytes_read;

    while (1) {
        // Buffer'ı temizle ve veriyi oku
        memset(buffer, 0, BUFFER_SIZE);
        bytes_read = read(uart_fd, buffer, BUFFER_SIZE - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';  // String sonlandırıcı ekle
            printf("Alınan Veri: %s\n", buffer);

            // Paket bütünlüğü kontrolü: hem "#START#" hem "#END#" içeriyor mu?
            if (strstr(buffer, "#START#") && strstr(buffer, "#END#")) {
                /* 
                  Önce genel mesajı ayrıştırıyoruz:
                  - gps_data: GPS bilgilerini içeren string (enlem, boylam, yön bilgileri ve fix kalitesi)
                  - altitude: İrtifa (metre)
                  - pressure: Basınç (hPa)
                  - accel_x, accel_y, accel_z: Akselerometre verileri
                  
                  Mesajın formatı:
                  "#START#GPS:%[^|]| ALT:%f | PRES:%f | ACCEL:[X:%f, Y:%f, Z:%f]#END#"
                */
                char gps_data[128];
                float altitude, pressure, accel_x, accel_y, accel_z;
                int parsed = sscanf(buffer,
                                    "#START#GPS:%[^|]| ALT:%f | PRES:%f | ACCEL:[X:%f, Y:%f, Z:%f]#END#",
                                    gps_data, &altitude, &pressure, &accel_x, &accel_y, &accel_z);
                if (parsed == 6) {
                    printf("Genel GPS Verisi: %s\n", gps_data);
                    printf("İrtifa: %.2f metre\n", altitude);
                    printf("Basınç: %.2f hPa\n", pressure);
                    printf("Akselerometre: X: %.2f, Y: %.2f, Z: %.2f\n", accel_x, accel_y, accel_z);

                    /*
                      Şimdi gps_data içindeki detaylı GPS bilgilerini ayrıştıralım.
                      Beklenen format:
                      "Enlem: <enlem> <enlem_yön>, Boylam: <boylam> <boylam_yön>, Fix Kalitesi: <fix>"
                    */
                    char enlem[32], enlem_direction[8];
                    char boylam[32], boylam_direction[8];
                    char fix_quality[16];
                    
                    /* char enlem[16], boylam[16], fix_quality[2];
                    char enlem_direction[2], boylam_direction[2];*/
                    
                    int parsedGPS = sscanf(gps_data,
                                             "Enlem: %s %s, Boylam: %s %s, Fix Kalitesi: %s",
                                             enlem, enlem_direction, boylam, boylam_direction, fix_quality);
                    if (parsedGPS == 5) {
                        printf("Ayrıştırılmış GPS Bilgileri:\n");
                        printf("  Enlem: %s %s\n", enlem, enlem_direction);
                        printf("  Boylam: %s %s\n", boylam, boylam_direction);
                        printf("  Fix Kalitesi: %s\n", fix_quality);
                    } else {
                        printf("GPS verisi ayrıştırılırken hata oluştu. Beklenen format:\n");
                        printf("  'Enlem: <enlem> <enlem_yön>, Boylam: <boylam> <boylam_yön>, Fix Kalitesi: <fix>'\n");
                    }
                } else {
                    printf("Genel veri formatı hatalı. Ayrıştırılan değer sayısı: %d\n", parsed);
                }
            } else {
                printf("Eksik veya bozuk veri paketi\n");
            }
        }
        // Döngü arasında 500 ms bekle
        usleep(500000);
    }
}

int main() {
    int uart_fd = init_uart();
    if (uart_fd == -1) {
        return -1;
    }

    printf("LoRa alıcı başlatıldı, veri bekleniyor...\n");
    read_lora_data(uart_fd);

    close(uart_fd);
    return 0;
}
