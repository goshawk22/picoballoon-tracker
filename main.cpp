/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>

#include "mbed.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "TinyGPSPlus.h"
#include "gps.h"
#include "BMP280.h"

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        30s

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue lora_ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Thread to run the gps loop. Allows us to continuously feed the TinyGPSPlus object
 * while not blocking any LoRa events.
 */
Thread gps_thread;

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Pin to control power to GPS
 */
DigitalOut p_vcc(PB_5);

/**
 * BMP280 I2C
 */
BMP280 bmp280(PA_11, PA_12, 0x76);

// Store Lat & Long in six bytes of payload
void pack_lat_lon(double lat, double lon) {
  uint32_t LatitudeBinary;
  uint32_t LongitudeBinary;
  LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
  LongitudeBinary = ((lon + 180) / 360.0) * 16777215;

  tx_buffer[0] = (LatitudeBinary >> 16) & 0xFF;
  tx_buffer[1] = (LatitudeBinary >> 8) & 0xFF;
  tx_buffer[2] = LatitudeBinary & 0xFF;
  tx_buffer[3] = (LongitudeBinary >> 16) & 0xFF;
  tx_buffer[4] = (LongitudeBinary >> 8) & 0xFF;
  tx_buffer[5] = LongitudeBinary & 0xFF;
}

/**
 * Entry point for application
 */
int main(void)
{
    // setup tracing
    setup_trace();

    // Turn on GPS
    p_vcc.write(1);
    // Wait for it to turn on and then initialize it
    ThisThread::sleep_for(500ms);
    init_gps();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&lora_ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.disable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n disable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Disabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // Start feeding the gps object
    gps_thread.start(gps_loop);

    // Initialize the BMP280
    bmp280.initialize();

    // make event queue dispatching events forever
    lora_ev_queue.dispatch_forever();

    return 0;
}

/**
 * Sends a message to the Network Server
 */
static void send_message() {
    uint16_t packet_len = 15;
    int16_t retcode;

    double lat;
    double lon;
    uint16_t altitude;
    uint8_t sats;
    uint8_t speed;

    float raw_temp;
    uint16_t temp;
    uint32_t pressure;

    // Packet all the GPS information
    lat = gps_parser.location.lat();
    lon = gps_parser.location.lng();
    pack_lat_lon(lat, lon);

    altitude = (uint16_t)gps_parser.altitude.meters();
    if (altitude < 0)
        altitude = 0; // avoid negatives, they are most likely a result of a poor fix or bug in the code and it's easier to use unsigned integers.

    speed = (uint16_t)gps_parser.speed.kmph();  // convert from double
    if (speed > 255)
        speed = 255;  // don't wrap around.
    
    sats = gps_parser.satellites.value();

    tx_buffer[6] = (altitude >> 8) & 0xFF;
    tx_buffer[7] = altitude & 0xFF;
    tx_buffer[8] = speed & 0xFF;
    tx_buffer[9] = sats & 0xFF;
    
    raw_temp = bmp280.getTemperature();
    pressure = bmp280.getPressure();
    
    temp = int(raw_temp * 10 + 0.5) + 128; // Encode the temperature to avoid negatives.

    tx_buffer[10] = (pressure >> 16) & 0xFF;
    tx_buffer[11] = (pressure >> 8) & 0xFF;
    tx_buffer[12] = pressure & 0xFF;

    tx_buffer[13] = (temp >> 8) & 0xFF;
    tx_buffer[14] = temp & 0xFF;
    
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                lora_ev_queue.call_in(3s, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
    if (rx_buffer[0] == 0x00) {
        enter_gps_standby();
    } else if (rx_buffer[0] == 0x01) {
        exit_gps_standby();
    }

    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                lora_ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            lora_ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
