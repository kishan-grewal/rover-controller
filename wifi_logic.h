#ifndef WIFI_LOGIC_H
#define WIFI_LOGIC_H

#include <Arduino.h>

extern float pid_values[3];
extern bool pid_updated;

void setupWiFi();

bool handleWiFi();

#endif // WIFI_LOGIC_H