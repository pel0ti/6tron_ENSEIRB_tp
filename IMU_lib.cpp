/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "bme280.h"

using namespace sixtron;

constexpr auto PERIOD = 2000ms;
I2C i2c(I2C1_SDA, I2C1_SCL);
BME280 sensor(&i2c, BME280::I2CAddress::Address1);

int main()
{
    sensor.initialize();
    sensor.set_sampling();

    while (true) {
        float t = sensor.temperature();
        float h = sensor.humidity();
        float p = sensor.pressure();
        printf("Temp : %.2f C// Humi : %.2f %  // Pres : %.0f Pa \n", t, h ,p);
        ThisThread::sleep_for(PERIOD/2);
    }
}
