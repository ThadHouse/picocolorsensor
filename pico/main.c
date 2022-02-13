#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define SENSOR_ADDRESS 0x52

#define MAIN_CTRL_REGISTER 0x00
#define PROXIMITY_SENSOR_RATE_REGISTER 0x03
#define PROXIMITY_SENSOR_PULSES_REGISTER 0x02
#define PART_ID_REGISTER 0x06
#define MAIN_STATUS_REGISTER 0x07
#define PROXIMITY_DATA_REGISTER 0x08

#define MAIN_CTRL_RGB_MODE 0x04
#define MAIN_CTRL_LIGHT_SENSOR_ENABLE 0x02
#define MAIN_CTRL_PROXIMITY_SENSOR_ENABLE 0x01

#define PROXIMITY_SENSOR_RATE_100MS 0x05
#define PROXIMITY_SENSOR_RES_11BIT 0x18

#define PROXIMITY_SENSOR_PULSES 32

#define EXPECTED_PART_ID 0xC2

bool read_i2c_data(i2c_inst_t *i2c, uint8_t reg, uint8_t* buffer, uint8_t readLen) {
    buffer[0] = reg;
    int result = i2c_write_timeout_us(i2c, SENSOR_ADDRESS, buffer, 1, true, 25000);
    if (result != 1) {
        return false;
    }
    result = i2c_read_timeout_us(i2c, SENSOR_ADDRESS, buffer, readLen, false, 25000);
    return result == readLen;
}

void init_device(i2c_inst_t* i2c, uint8_t* i2cBuffer) {
    i2cBuffer[0] = MAIN_CTRL_REGISTER;
    i2cBuffer[1] = MAIN_CTRL_RGB_MODE | MAIN_CTRL_LIGHT_SENSOR_ENABLE | MAIN_CTRL_PROXIMITY_SENSOR_ENABLE;
    i2c_write_timeout_us(i2c, SENSOR_ADDRESS, i2cBuffer, 2, false, 25000);

    i2cBuffer[0] = PROXIMITY_SENSOR_RATE_REGISTER;
    i2cBuffer[1] = PROXIMITY_SENSOR_RATE_100MS | PROXIMITY_SENSOR_RES_11BIT;
    i2c_write_timeout_us(i2c, SENSOR_ADDRESS, i2cBuffer, 2, false, 25000);

    i2cBuffer[0] = PROXIMITY_SENSOR_PULSES_REGISTER;
    i2cBuffer[1] = PROXIMITY_SENSOR_PULSES;
    i2c_write_timeout_us(i2c, SENSOR_ADDRESS, i2cBuffer, 2, false, 25000);
}

int main() {
    // Init uart
    uart_init(uart0, 115200);
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);

    // Init I2c 0
    i2c_init(i2c0, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Init I2c 1
    i2c_init(i2c1, 400000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);

    // Channel 0 output
    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);
    gpio_put(14, 1);

    // Channel 1 output
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 1);

    char outputBuffer[512];

    unsigned int values[10];
    uint8_t i2cBuffer[20];
    bool currentValid0 = false;
    bool currentValid1 = false;
    absolute_time_t loopTime;
    bool reset;

    sleep_ms(100);

    init_device(i2c0, i2cBuffer);
    read_i2c_data(i2c0, MAIN_STATUS_REGISTER, i2cBuffer, 1);

    init_device(i2c1, i2cBuffer);
    read_i2c_data(i2c1, MAIN_STATUS_REGISTER, i2cBuffer, 1);

    memset(values, 0, sizeof(values));

    while (1) {
        loopTime = make_timeout_time_ms(100);

        currentValid0 = read_i2c_data(i2c0, MAIN_STATUS_REGISTER, i2cBuffer, 15);
        if (currentValid0) {
            if ((i2cBuffer[0] & 0x20) != 0) {
                init_device(i2c0, i2cBuffer);
            } else {
                values[4] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
                values[3] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
                values[2] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
                values[1] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
                values[0] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
            }
        }

        currentValid1 = read_i2c_data(i2c1, MAIN_STATUS_REGISTER, i2cBuffer, 15);
        if (currentValid1) {
            if ((i2cBuffer[0] & 0x20) != 0) {
                init_device(i2c1, i2cBuffer);
            } else {
                values[9] = ((i2cBuffer[1] & 0xFF) | ((i2cBuffer[2] & 0xFF) << 8)) & 0x7FF;
                values[8] = ((i2cBuffer[3] & 0xFF) | ((i2cBuffer[4] & 0xFF) << 8) | ((i2cBuffer[5] & 0xFF) << 16)) & 0x03FFFF;
                values[7] = ((i2cBuffer[6] & 0xFF) | ((i2cBuffer[7] & 0xFF) << 8) | ((i2cBuffer[8] & 0xFF) << 16)) & 0x03FFFF;
                values[6] = ((i2cBuffer[9] & 0xFF) | ((i2cBuffer[10] & 0xFF) << 8) | ((i2cBuffer[11] & 0xFF) << 16)) & 0x03FFFF;
                values[5] = ((i2cBuffer[12] & 0xFF) | ((i2cBuffer[13] & 0xFF) << 8) | ((i2cBuffer[14] & 0xFF) << 16)) & 0x03FFFF;
            }
        }

        snprintf(outputBuffer, sizeof(outputBuffer), "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
            currentValid0, currentValid1, values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9]);

        uart_puts(uart0, outputBuffer);

        sleep_until(loopTime);
    }
}
