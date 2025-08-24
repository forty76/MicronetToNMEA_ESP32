/***************************************************************************
 *                                                                         *
 * Project:  MicronetToNMEA                                                *
 * Purpose:  Decode data from Micronet devices send it on an NMEA network  *
 * Author:   Ronan Demoment                                                *
 *                                                                         *
 ***************************************************************************
 *   Copyright (C) 2021 by Ronan Demoment                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 */

/***************************************************************************/
/*                              Includes                                   */
/***************************************************************************/

#include <Arduino.h>

#include "BoardConfig_ESP32.h"
#include "Configuration.h"
#include "Globals_ESP32.h"
#include "MenuManager.h"
#include "Micronet.h"
#include "MicronetCodec.h"
#include "MicronetMessageFifo.h"
#include "NavCompass.h"
#include "Version.h"

#include <SPI.h>
#include <Wire.h>

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

/***************************************************************************/
/*                             Local types                                 */
/***************************************************************************/

/***************************************************************************/
/*                           Local prototypes                              */
/***************************************************************************/

void RfIsr();

/***************************************************************************/
/*                               Globals                                   */
/***************************************************************************/

bool firstLoop;

/***************************************************************************/
/*                              Functions                                  */
/***************************************************************************/


TaskHandle_t BlinkTaskHandle = NULL;
bool dt = false;


void BlinkTask(void *parameter) {
  for (;;) { // Infinite loop
    if (dt = true)
    {
        //CONSOLE.print("1");
        gRfReceiver.RfIsr();        
        dt=false;
        //Serial.printf("Task1 Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void setup()
{
    // Load configuration from EEPROM
    gConfiguration.LoadFromEeprom();

    // Init USB serial link
    USB_NMEA.begin(USB_BAUDRATE);
    delay(1000);
    CONSOLE.println("Start CONSOLE...");
    CONSOLE.print("SPI pin definition    MOSI ");
    CONSOLE.print(MOSI);
    CONSOLE.print("\t MISO ");
    CONSOLE.print(MISO);
    CONSOLE.print("\t SCK ");
    CONSOLE.print(SCK);
    CONSOLE.print("\t CS ");
    CONSOLE.print(SS);
    CONSOLE.print("\t GDO0_PIN ");
    CONSOLE.println(GDO0_PIN);
    delay(2000);


 

    // Init wired serial link
    WIRED_NMEA.begin(WIRED_BAUDRATE);

    // Let time for serial drivers to set-up
    delay(250);



    CONSOLE.print("Initializing CC1101 ... ");
    // Check connection to CC1101
    if (!gRfReceiver.Init(&gRxMessageFifo, gConfiguration.rfFrequencyOffset_MHz))
    {
        CONSOLE.println("Failed");
        CONSOLE.println("Aborting execution : Verify connection to CC1101 board");
        CONSOLE.println("Halted");

        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }
    CONSOLE.println("OK");


    // Start listening
    gRfReceiver.RestartReception();

    // Attach callback to GDO0 pin
    // According to CC1101 configuration this callback will be executed when CC1101 will have detected Micronet's sync word
#if defined(ARDUINO_TEENSY35) || defined(ARDUINO_TEENSY36)
    attachInterrupt(digitalPinToInterrupt(GDO0_PIN), RfIsr, RISING);
#else
    attachInterrupt(digitalPinToInterrupt(GDO0_PIN), RfIsr, HIGH);
#endif

    // Display serial menu
    gMenuManager.PrintMenu();

    // For the main loop to know when it is executing for the first time
    firstLoop = true;

          xTaskCreatePinnedToCore(
    BlinkTask,         // Task function
    "BlinkTask",       // Task name
    2000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &BlinkTaskHandle,  // Task handle
    1                  // Core 1
  );
}

void loop()
{
    // If this is the first loop, we verify if we are already attached to a Micronet network. if yes,
    // We directly jump to NMEA conversion mode.
    if ((firstLoop) && (gConfiguration.networkId != 0))
    {
        // Menu 4 is NMEA Conversion
        gMenuManager.ActivateMenu(4);
        gMenuManager.PrintMenu();
    }

    // Process console input
    while (CONSOLE.available() > 0)
    {
        gMenuManager.PushChar(CONSOLE.read());
    }

    firstLoop = false;
}

void RfIsr()
{
    dt = true;
    //gRfReceiver.RfIsr();
}

