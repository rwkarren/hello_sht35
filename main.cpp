
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Seeed_SHT35.h"

#define I2C_BUS i2c0
#define SDA_PIN 4
#define SCL_PIN 5

using namespace std;

SHT35 sensor(I2C_BUS, SCL_PIN, SDA_PIN);

bool
init() {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    sleep_ms(10);
    printf("UART initialized\n");
    if (sensor.init()) {
        printf("Sensor init failed!\n");
        return true;
    } else {
        printf("Sensor init success!\n");
    }
    sleep_ms(1000);
    return false;

}

float
temp_f(float temp_c) {
    return temp_c * 1.8f + 32.0f;
}

int
main() {
    if (init()) {
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, true);
            sleep_ms(250);
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            sleep_ms(250);
        }
    } else {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        while (true) {
            uint16_t value = 0;
            uint8_t data[6] = {0};
            float temp = 0;
            float hum = 0;

            if (sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum) != NO_ERROR) {
                printf("Read temp failed\n");
                printf("\n");
                printf("\n");
                printf("\n");
            } else {
                printf("read data:\n");
                printf("temperature = %4.2f deg F\n", temp_f(temp));
                printf("humidity = %4.1f %%\n", hum);
                printf("\n");
            }
            sleep_ms(1000);
        }
    }
}

