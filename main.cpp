/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"

using namespace std::chrono;

DigitalOut   myled(LED1);
InterruptIn  button(BUTTON1);
Timer        t;

volatile int flag = 0;

constexpr auto PERIOD = 2000ms;

void LEDset()
{
    myled = 1;
    t.reset();
    t.start();
    flag = 1;
}

void LEDclr()
{
    myled = 0;
    t.stop();
    flag = 0;
}

int main()
{
    if (myled.is_connected()) {
        printf(">> myled is initialized and connected!\n\r");
    }

    button.rise(&LEDset);
    button.fall(&LEDclr);

    while (true) {
        if (flag == 1) {
            printf("[Timer ON] %lld us\n\r", t.elapsed_time().count());
        } else {
            t.reset();
        }

        ThisThread::sleep_for(PERIOD / 5);
    }
}
