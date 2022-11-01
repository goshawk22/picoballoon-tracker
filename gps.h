#pragma once

#include <TinyGPS++.h>

extern TinyGPSPlus gps_parser;

void gps_loop(bool print_it);
void gps_time(char *buffer, uint8_t size);
void display_gps_info(void);