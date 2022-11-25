#include "gps.h"
#include "mbed.h"
#include <TinyGPS++.h>

/**
 * GPS Setup
 */

static BufferedSerial gps(PB_6, PB_7, 9600);
TinyGPSPlus gps_parser;

// PC
static BufferedSerial pc(USBTX, USBRX);

void gps_time(char* buffer, uint8_t size) {
    printf(buffer, size, "%02d:%02d:%02d", gps_parser.time.hour(), gps_parser.time.minute(), gps_parser.time.second());
}

void gps_loop(bool print_it) {
    char incoming;
    if (uint32_t num = gps.read(&incoming, 1)) {
        // Echo the input back to the terminal.
        if (print_it)
            pc.write(&incoming, num);

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
