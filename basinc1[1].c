#include <stdio.h>
#include <math.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <unistd.h>

#define BMP388 0x77

// UART portu
#define UART_PORT "/dev/serial0"
#define UART_BAUD 9600

#define R 287.0
#define G 9.81
#define M 0.029

float read_pressure(int fd);
float calculate_altitude(float pressure, float reference_pressure);

int main() {
    int fd, uart_fd;
    float reference_pressure, current_pressure, altitude;

    fd = wiringPiI2CSetup(BMP388);
    if (fd == -1) {
        printf("BMP388 sensörüne bağlanılamadı.\n");
        return -1;
    }

    uart_fd = serialOpen(UART_PORT, UART_BAUD);
    if (uart_fd == -1) {
        printf("UART bağlantısı başlatılamadı.\n");
        return -1;
    }

    printf("Sensör ve UART başlatılıyor...\n");
    sleep(1);

    reference_pressure = read_pressure(fd);
    if (reference_pressure <= 0) {
        printf("Başlangıç basıncı okunamadı.\n");
        return -1;
    }

    printf("Başlangıç basıncı: %.2f Pa\n", reference_pressure);

    while (1) {
        current_pressure = read_pressure(fd);
        if (current_pressure <= 0) {
            printf("Basınç okuma hatası.\n");
            continue;
        }

        altitude = calculate_altitude(current_pressure, reference_pressure);

        char data[100];
        snprintf(data, sizeof(data), "Basınç: %.2f Pa, İrtifa: %.2f m\n", current_pressure, altitude);
        printf("%s", data);
        serialPuts(uart_fd, data); 

        usleep(100000);
    }

    serialClose(uart_fd);

    return 0;
}

// BMP388 basınç okuma fonksiyonu
float read_pressure(int fd) {
    // Basınç okuma işlemi (örnek olarak 3 bayt okuma varsayımı)
    // Gerçek uygulamada sensörün datasheet'ine göre ayarlayın
    int msb = wiringPiI2CReadReg8(fd, 0x04); // Örnek adres
    int lsb = wiringPiI2CReadReg8(fd, 0x05); // Örnek adres
    int xlsb = wiringPiI2CReadReg8(fd, 0x06); // Örnek adres

    if (msb == -1 || lsb == -1 || xlsb == -1) {
        return -1;
    }

    int raw_pressure = (msb << 16) | (lsb << 8) | xlsb;
    return raw_pressure / 256.0; // Basınç birimini doğru ölçekleyin (örneğin Pascal)
}

// İrtifa hesaplama fonksiyonu
float calculate_altitude(float pressure, float reference_pressure) {
    return (R * 288.15 / G) * log(reference_pressure / pressure);
}
