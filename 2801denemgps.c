#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define UART_PORT  "/dev/serial0"  // UART device file
#define BAUD_RATE  9600            // Baud rate for GPS communication - nee  
#define GPS_BUFFER_SIZE 128

char gps_buffer[GPS_BUFFER_SIZE];
int buffer_index = 0;
int uart_fd;

void uart_init(int baud_rate) {
    uart_fd = serialOpen(UART_PORT, BAUD_RATE);
    if (uart_fd == -1) {
        fprintf(stderr, "Unable to open UART port\n");
        exit(1);
    }
    printf("UART initialized at %d baud\n", BAUD_RATE);
}

char uart_read_char() {
    if (serialDataAvail(uart_fd)) {
        return serialGetchar(uart_fd);
    }
    return -1;  // No data available
}

int uart_available() {
    return serialDataAvail(uart_fd);  // Returns 1 if data is available
}

int validate_checksum(const char *nmea_sentence) {
    char checksum = 0;
    const char *ptr = nmea_sentence + 1; 
    while (*ptr && *ptr != '*') {
        checksum ^= *ptr++;
    }

    char received_checksum[3];
    snprintf(received_checksum, 3, "%02X", checksum);

    return strncmp(received_checksum, nmea_sentence + (ptr - nmea_sentence) + 1, 2) == 0;
}

void parse_gpgga(const char *nmea_sentence) {
    char enlem[16], boylam[16], fix_quality[2];  //char enlem[16] = {0}, boylam[16] = {0}, fix_quality[2] = {0};
    char enlem_direction[2], boylam_direction[2];

    const char *delim = ",";

    char sentence_copy[GPS_BUFFER_SIZE]; 
    strncpy(sentence_copy, nmea_sentence, GPS_BUFFER_SIZE); //strncpy(sentence_copy, nmea_sentence, sizeof(sentence_copy)); // NMEA cümlesinin kopyalanması - YA NEDEN

    char *token = strtok(sentence_copy, delim);
    int field_index = 0;

    while (token != NULL) {
        field_index++;
        if (field_index == 3) { 
            strncpy(enlem, token, sizeof(enlem));  // -1 gerekli mi
        } else if (field_index == 4) {
            strncpy(enlem_direction, token, sizeof(enlem_direction));
            //strncat(enlem, token, sizeof(enlem) - strlen(enlem) - 1);
        } else if (field_index == 5) { 
            strncpy(boylam, token, sizeof(boylam));
        } else if (field_index == 6) { 
            strncpy(boylam_direction, token, sizeof(boylam_direction));
            //strncat(boylam, token, sizeof(boylam) - strlen(boylam) - 1);
        } else if (field_index == 7) { 
            strncpy(fix_quality, token, sizeof(fix_quality));
        }

        token = strtok(NULL, delim);
    }

    printf("Enlem: %s %s\n", enlem, enlem_direction);
    printf("Boylam: %s %s\n", boylam, boylam_direction);
    printf("Fix Kalitesi: %s\n", fix_quality);
    }

int main() {
    uart_init(BAUD_RATE);  // Initialize UART - 9600 demem gereken diğer yer mi? 

    while (1) {
        if (uart_available()) {
            char c = uart_read_char();
            if (c == '\n') { 
                gps_buffer[buffer_index] = '\0';
                buffer_index = 0;

                if (validate_checksum(gps_buffer)) { 
                    if (strncmp(gps_buffer, "$GPGGA", 6) == 0) {
                        parse_gpgga(gps_buffer);
                    }
                } else {
                    printf("Gecersiz checksum: %s\n", gps_buffer);
                }
            } else if (buffer_index < GPS_BUFFER_SIZE - 1) {
                gps_buffer[buffer_index++] = c;
            }
        }
    }

    return 0;
}
