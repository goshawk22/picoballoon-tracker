#pragma once

#include <TinyGPS++.h>

extern TinyGPSPlus gps_parser;
extern bool ack_rec;

void gps_loop(void);
void gps_time(char *buffer, uint8_t size);
void display_gps_info(void);
void init_gps(void);
bool wait_for_ack(char c);
bool enter_gps_standby(void);
void exit_gps_standby(void);
bool get_need_longer_sleep(void);
void set_need_longer_sleep(bool set_bool);

// PMTK strings
#define STANDBY_STRING "$PMTK161,0*28\r\n"
#define WAKEUP_STRING "WAKE UP\r\n"
#define NMEA_CONFIG_STRING "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
#define PMTK_SET_BALLOON_MODE "$PMTK886,3*2B\r\n"
#define PMTK_SET_BALLOON_MODE_RESPONSE "$PMTK001,886,3*36"