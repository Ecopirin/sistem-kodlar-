#include <stdint.h>
#include <stdio.h>
#include "i2c_library.h"
#include "lora_library.h" 
#include "bno055_registers.h"

#define BNO055_I2C_ADDR 0x28
#define BMP388_I2C_ADDR 0x77

typedef struct {
    float altitude;
    float accel_x;
    float accel_y;
    float accel_z;
} TelemetryData;

void init_sensors() {
    i2c_write_byte(BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    i2c_write_byte(BNO055_I2C_ADDR, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    i2c_write_byte(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 0x00);

    i2c_write_byte(BMP388_I2C_ADDR, 0x1B, 0x33); 
}

void read_telemetry_data(TelemetryData *data) {
    uint8_t buffer[6];

    i2c_read_bytes(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
    data->accel_x = (float)((int16_t)((buffer[1] << 8) | buffer[0])) / 100.0;
    data->accel_y = (float)((int16_t)((buffer[3] << 8) | buffer[2])) / 100.0;
    data->accel_z = (float)((int16_t)((buffer[5] << 8) | buffer[4])) / 100.0;

    data->altitude = read_bmp388_altitude();
}

void send_telemetry_data(TelemetryData *data) {
    char message[64];
    snprintf(message, sizeof(message), 
             "Altitude: %.2f m, Accel: [X: %.2f, Y: %.2f, Z: %.2f] m/s^2",
             data->altitude, data->accel_x, data->accel_y, data->accel_z);
    
    lora_send_message(message);
}

int main() {
    TelemetryData telemetry_data;

    init_sensors();
    lora_init();

    while (1) {
        read_telemetry_data(&telemetry_data);

        send_telemetry_data(&telemetry_data);

        delay(1000);
    }

    return 0;
}
