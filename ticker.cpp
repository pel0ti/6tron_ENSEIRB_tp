/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"

using namespace std::chrono;

DigitalOut   myled(LED1);
InterruptIn  button(BUTTON1);
Ticker       ledTicker;

constexpr auto PERIOD = 2000ms;
int divider = 1;
milliseconds period = PERIOD;

void toggleLED()
{
    myled = !myled;
}

void changePeriod()
{
    divider = (divider * 2) % 16;
    if (divider == 0) {
        divider = 1;
    }

    period = PERIOD / divider;

    ledTicker.detach();
    ledTicker.attach(&toggleLED, period);
}

int main()
{
    ledTicker.attach(&toggleLED, period);
    button.fall(&changePeriod);

    while (true) {
        ThisThread::sleep_for(PERIOD);
    }
}
