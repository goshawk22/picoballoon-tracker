#include "gps.h"
#include "mbed.h"
#include <TinyGPS++.h>

/**
 * GPS Setup
 */

static BufferedSerial gps(PB_6, PB_7, 9600);
TinyGPSPlus gps_parser;

char buf[100] = {0};
uint8_t offset = 0;
bool ack_rec = false; // Have we recieved an ack from the gps
bool ack = false; // Are waiting for an ack from gps

void gps_time(char* buffer, uint8_t size) {
    printf(buffer, size, "%02d:%02d:%02d", gps_parser.time.hour(), gps_parser.time.minute(), gps_parser.time.second());
}

void gps_loop(void) {
    char incoming;
    if (uint32_t num = gps.read(&incoming, 1)) {
        // If waiting for ack, also check for ack.
        if (ack)
            ack_rec = wait_for_ack(incoming);

        gps_parser.encode(incoming);
    }
}

//Display new GPS info
void display_gps_info(void)  {
    //We have new GPS data to deal with!
    printf("\n");

    // display date time
    if (gps_parser.time.isValid()) {
        printf("Datum: %d/%02d/%02d  --  %02d:%02d:%02d \n",
                         gps_parser.date.year(), gps_parser.date.month(), gps_parser.date.day(),
                         gps_parser.time.hour(), gps_parser.time.minute(), gps_parser.time.second());
    } else {
        printf("Time not yet valid\n");
    }

    // display location
    if (gps_parser.location.isValid()) {
        printf("Location: %f, %f\n", gps_parser.location.lat(), gps_parser.location.lng());
    } else {
        printf("Location not yet valid\n");
    }

    // display altitude
    if (gps_parser.altitude.isValid()) {
        printf("Altitude in meters: %f in feet: %f\n", gps_parser.altitude.meters(), gps_parser.altitude.feet());
    }

    // display satellite stats
    if (gps_parser.satellites.isValid()) {
        printf(" Satellites in View: %d", gps_parser.satellites.value());
    }

    // display HDOP
    if (gps_parser.hdop.isValid()) {
        printf(" HDOP: %.2f\n", (gps_parser.hdop.value()/100.0));
    }
}

void init_gps(void) {
    if (gps.writable()) {
        gps.write(NMEA_CONFIG_STRING, sizeof(NMEA_CONFIG_STRING));
        gps.write(PMTK_SET_NMEA_UPDATE_1HZ, sizeof(PMTK_SET_NMEA_UPDATE_1HZ));
        gps.write(PMTK_SET_BALLOON_MODE, sizeof(PMTK_SET_BALLOON_MODE));
        ack = true;
    }
}

bool wait_for_ack(char c) {
    switch(c) {
        case '$':
            memset(buf, 0, sizeof(buf));
            offset = 0;
            buf[offset] = c;
            break;
        case '\n':
        case '\r':
            break;
        default:
            offset++;
            buf[offset] = c;
            break;
    }
    if (!strcmp(buf, PMTK_SET_BALLOON_MODE_RESPONSE)) {
        ack = false;
        printf("\r\n ACK Recieved! \r\n");
        return true;
    } else {
        return false;
    }
}

bool enter_gps_standby(void) {
    if (gps.writable()) {
        gps.write(STANDBY_STRING, sizeof(STANDBY_STRING));
        return true;
    } else {
        printf("GPS is not writeable, cannot enter standby");
        return false;
    }
}

void exit_gps_standby(void) {
    gps.write(WAKEUP_STRING, sizeof(WAKEUP_STRING));
    init_gps();
}