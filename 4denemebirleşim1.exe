#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "pigpio.h"
#include "LORA.h"
#include "BNO055.h"

// Sensör ve iletişim tanımlamaları
#define BNO055_I2C_ADDR 0x28
#define BMP388_I2C_ADDR 0x77
#define UART_PORT "/dev/serial0"
#define UART_BAUD 9600
#define GPS_BUFFER_SIZE 128

#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08

// BMP388 Basınç Okuma Registerları
#define PRESSURE_MSB_REG 0x04
#define PRESSURE_LSB_REG 0x05
#define PRESSURE_XLSB_REG 0x06

// Irtifa hesaplama için sabitler
#define R 287.0
#define G 9.81
#define MAX_RETRY 5

// Veri yapısı: ivme ve irtifa bilgileri
typedef struct {
    float altitude;
    float accel_x;
    float accel_y;
    float accel_z;
} TelemetryData;

// Global değişkenler
char gps_buffer[GPS_BUFFER_SIZE];
int buffer_index = 0;
int uart_fd;

// Fonksiyon Prototipleri
void init_sensors(void);
int init_uart(void);
char uart_read_char(void);
int uart_available(void);
int validate_checksum(const char *nmea_sentence);
void process_gps_sentence(const char *nmea_sentence);
float read_pressure(int fd);
float calculate_altitude(float pressure, float reference_pressure);
void read_telemetry_data(TelemetryData *data);
void send_combined_data(TelemetryData *data, const char *gps_data, float pressure, float enlem, char enlem_direction, float boylam, char boylam_direction, float fix_qualit);
void process_gps(void);

//
// 1. Sensörlerin Başlatılması
//
void init_sensors() {
    // BNO055: İvmeölçer verisi için gerekli modları ayarla
    if(i2c_write_byte(BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF) < 0) {
        fprintf(stderr, "BNO055 çalışma modu ayarlanamadı\n");
    }
    if(i2c_write_byte(BNO055_I2C_ADDR, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0) {
        fprintf(stderr, "BNO055 güç modu ayarlanamadı\n");
    }
    if(i2c_write_byte(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 0x00) < 0) {
        fprintf(stderr, "BNO055 sistem tetikleyici ayarlanamadı\n");
    }

    // BMP388: Basınç sensörünün konfigürasyonu
    if(i2c_write_byte(BMP388_I2C_ADDR, 0x1B, 0x33) < 0) {
        fprintf(stderr, "BMP388 konfigürasyonu yapılamadı\n");
    }
}

//
// 2. UART Başlatma
//
int init_uart() {
    uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
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

//
// 3. UART Okuma Fonksiyonları
//
char uart_read_char() {
    if (uart_available()) {
        return serialGetchar(uart_fd);
    }
    return -1;
}

int uart_available() {
    return serialDataAvail(uart_fd);
}

//
// 4. GPS NMEA Checksum Doğrulaması
//
int validate_checksum(const char *nmea_sentence) {
    char checksum = 0;
    const char *ptr = nmea_sentence + 1; // '$' karakterinden sonraki kısmı oku
    while (*ptr && *ptr != '*') {
        checksum ^= *ptr++;
    }
    char received_checksum[3];
    snprintf(received_checksum, sizeof(received_checksum), "%02X", checksum);
    return strncmp(received_checksum, ptr + 1, 2) == 0;
}

//
// 5. GPS Verisini İşle: GPGGA cümlesini ayrıştır ve logla
//
void parse_gpgga(const char *nmea_sentence) {
    char enlem[16], boylam[16], fix_quality[2];
    char enlem_direction[2], boylam_direction[2];
    const char *delim = ",";
    char sentence_copy[GPS_BUFFER_SIZE];
    strncpy(sentence_copy, nmea_sentence, GPS_BUFFER_SIZE);

    char *token = strtok(sentence_copy, delim);
    int field_index = 0;

    while (token != NULL) {
        field_index++;
        if (field_index == 3) {
            strncpy(enlem, token, sizeof(enlem));
        } else if (field_index == 4) {
            strncpy(enlem_direction, token, sizeof(enlem_direction));
        } else if (field_index == 5) {
            strncpy(boylam, token, sizeof(boylam));
        } else if (field_index == 6) {
            strncpy(boylam_direction, token, sizeof(boylam_direction));
        } else if (field_index == 7) {
            strncpy(fix_quality, token, sizeof(fix_quality));
        }

        token = strtok(NULL, delim);
    }

    *enlem = atof(enlem);
    *boylam = atof(boylam);
    *enlem_direction = enlem_direction;
    *boylam_direction = boylam_direction;
    *fix_quality = atof(fix_quality);

    //printf("GPS -> Enlem: %s %s, Boylam: %s %s, Fix Kalitesi: %s\n",
    //       enlem, enlem_direction, boylam, boylam_direction, fix_quality);

    //printf("Enlem: %s %s\n", enlem, enlem_direction);
    //printf("Boylam: %s %s\n", boylam, boylam_direction);
    //printf("Fix Kalitesi: %s\n", fix_quality);
}

//
// 6. BMP388 Basınç Okuma ve İrtifa Hesaplama
//
float read_pressure(int fd) {
    int msb = wiringPiI2CReadReg8(fd, PRESSURE_MSB_REG);
    int lsb = wiringPiI2CReadReg8(fd, PRESSURE_LSB_REG);
    int xlsb = wiringPiI2CReadReg8(fd, PRESSURE_XLSB_REG);
    if (msb == -1 || lsb == -1 || xlsb == -1) {
        fprintf(stderr, "BMP388: Basınç okuma hatası\n");
        return -1;
    }
    int raw_pressure = (msb << 16) | (lsb << 8) | xlsb;
    return raw_pressure / 256.0;
}

float calculate_altitude(float pressure, float reference_pressure) {
    // Basit logaritmik model kullanılarak irtifa hesaplanır
    return (R * 288.15 / G) * log(reference_pressure / pressure);
}

//
// 7. BNO055 İvme Verisi Okuma
//
void read_telemetry_data(TelemetryData *data) {
    uint8_t buffer[6];
    if(i2c_read_bytes(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6) < 0) {
        fprintf(stderr, "BNO055: İvme verisi okunamadı\n");
        return;
    }
    data->accel_x = (float)((int16_t)((buffer[1] << 8) | buffer[0])) / 100.0;
    data->accel_y = (float)((int16_t)((buffer[3] << 8) | buffer[2])) / 100.0;
    data->accel_z = (float)((int16_t)((buffer[5] << 8) | buffer[4])) / 100.0;
}

//
// 8. Tüm Sensör Verilerini Tek Paket Halinde Gönderme
//
void send_combined_data(TelemetryData *data, const char *gps_data, float pressure, float enlem, char enlem_direction, float boylam, char boylam_direction, float fix_quality) {
    char message[256];

printf("GPS -> Enlem: %s %s, Boylam: %s %s, Fix Kalitesi: %s\n",
       enlem, enlem_direction, boylam, boylam_direction, fix_quality);

    snprintf(message, sizeof(message),
             "#START#GPS:%s | ALT:%.2f | PRES:%.2f | ACCEL:[X:%.2f, Y:%.2f, Z:%.2f]#END#",
             gps_data, data->altitude, pressure,
             data->accel_x, data->accel_y, data->accel_z);
    // LoRa kütüphanesinin gönderim fonksiyonu kullanılıyor -gps daha detaylı olmalı
    if(lora_send_message(message) < 0) {
        fprintf(stderr, "LoRa: Veri gönderilemedi: %s\n", message);
    } else {
        printf("LoRa: Veri başarıyla gönderildi: %s\n", message);
    }
}

//
// 9. GPS Verisinin Sürekli Okunması ve Buffer Güncellemesi
//
void process_gps(void) {
    while (uart_available()) {
        char c = uart_read_char();
        // Satır sonu, bir NMEA cümlesinin tamamlandığını belirtir.
         if (c == '\n') {
                gps_buffer[buffer_index] = '\0';
                buffer_index = 0;

                if (validate_checksum(gps_buffer)) {
                    if (strncmp(gps_buffer, "$GPGGA", 6) == 0) {
                        parse_gpgga(gps_buffer);
                    }
                } else {
                    printf("Geçersiz checksum: %s\n", gps_buffer);
                }
            } else if (buffer_index < GPS_BUFFER_SIZE - 1) {
                gps_buffer[buffer_index++] = c;
            }
        }
    }


//
// Ana Fonksiyon: Sensörleri Başlat, Verileri Oku ve Gönder
//
int main() {
    TelemetryData telemetry_data = {0};
    int bmp_fd, reference_pressure;
    float current_pressure;
    
    // Sensörleri başlat
    init_sensors();
    
    // UART başlatma (GPS için)
    if (init_uart() == -1) {
        exit(EXIT_FAILURE);
    }
    
    // LoRa modülünü başlat (lora_init fonksiyonu, lora_library.h'den geliyor)
    if(lora_init() < 0) {
        fprintf(stderr, "LoRa modülü başlatılamadı\n");
        exit(EXIT_FAILURE);
    }
    
    // BMP388 sensörü için I2C bağlantısı kuruluyor
    bmp_fd = wiringPiI2CSetup(BMP388_I2C_ADDR);
    if (bmp_fd == -1) {
        fprintf(stderr, "BMP388 sensörüne bağlanılamadı\n");
        exit(EXIT_FAILURE);
    }
    
    // Başlangıç basıncı (referans) okunuyor
    reference_pressure = read_pressure(bmp_fd);
    if (reference_pressure <= 0) {
        fprintf(stderr, "Başlangıç basıncı okunamadı\n");
        return -1;
        //exit(EXIT_FAILURE);
    }
    printf("Başlangıç Basınç: %.2f Pa\n", (float)reference_pressure);
    
    // Ana döngü: sensör verilerini topla, GPS verisini güncelle ve tüm verileri gönder
    while (1) {
        // İvme verisini oku
        read_telemetry_data(&telemetry_data);
        
        // Basınç oku ve irtifa hesapla
        current_pressure = read_pressure(bmp_fd);
        if (current_pressure <= 0) {
            fprintf(stderr, "Basınç okuma hatası\n");
            continue;
        }
        telemetry_data.altitude = calculate_altitude(current_pressure, reference_pressure);
        
        // GPS verisini sürekli oku (buffer güncelleniyor)
        process_gps();
        
        // GPS verilerini ayrıştır
        char enlem[16] = {0}, boylam[16] = {0}, fix_quality[2] = {0};
        char enlem_direction[2] = {0}, boylam_direction[2] = {0};
        parse_gpgga(gps_buffer);                       //burayı sonradan ekledm
        
        // Tüm verileri tek paket halinde gönder
        send_combined_data(&telemetry_data, gps_buffer, current_pressure, atof(enlem), 
                           enlem_direction[0], atof(boylam), boylam_direction[0], atof(fix_quality));
        
        // Bir sonraki döngüden önce kısa bekleme (örneğin 1 saniye)
        usleep(1000000);
    }
    
    close(uart_fd);
    return 0;
}
