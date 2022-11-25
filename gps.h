#pragma once

#include <TinyGPS++.h>

extern TinyGPSPlus gps_parser;
extern bool ack_rec;

void gps_loop(bool print_it);
void gps_time(char *buffer, uint8_t size);
void display_gps_info(void);
void init_gps(void);
bool wait_for_ack(char c);
// PMTK strings
#define NMEA_CONFIG_STRING "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
#define PMTK_SET_BALLOON_MODE "$PMTK886,3*2B\r\n"
#define PMTK_SET_BALLOON_MODE_RESPONSE "$PMTK001,886,3*36"