/*
 * ASTRA Biosensor
 * FAERIE Embedded Code
 */

//----------//
// Includes //
//----------//

#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <vector>
// Our own resources
#include "AstraMotors.h"
#include "AstraCAN.h"
#include "Adafruit_SHT31.h"
#include "TeensyThreads.h"


using namespace std;

//------//
// PINS //
//------//

#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)
#define LASER_PIN 15

//------------------------//
// Classes for components //
//------------------------//

// SHT 31 in SCABBARD
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // Faerie HUM/TEMP Sensor


unsigned CAN_ID = 6;

//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(CAN_ID, 1, false, 50, 0.50F);//Drill


// Last millis value that the motor was sent a duty cycle
unsigned long lastAccel;


//------------//
// Prototypes //
//------------//

void loopHeartbeats();
std::vector<String> parseInput(String input, const char delim);

//-------------//
// Begin Setup //
//-------------//

void setup() {

    //-----------------//
    // Initialize Pins //
    //-----------------//
  
    Serial.begin(115200); // or 115200
    Serial4.begin(9600); // for comms with Arm Socket Teensy/RasPi

    // Teensy built-in LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Faerie drill lasers
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);

    // LED stays on for 2 seconds to show powered on
    delay(2000);
    digitalWrite(LED_PIN, LOW);

    // ------- //
    //   CAN   //
    // ------- //

    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    //--------------------//
    // Initialize Sensors //
    //--------------------//

    if(!sht31.begin(0x44)) { // HUM/Temp
        Serial.println("Couldn't find SHT31!");
        //while(1) delay(1);
    } else {
        Serial.println("SHT Initialized.");
    }


    //-----------//
    // Heartbeat //
    //-----------//

    //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
    threads.addThread(loopHeartbeats);

}



//------------//
// Begin Loop //
//------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//

void loop() {

    // Accelerate the motors
    if(millis()-lastAccel >= 50) {
        lastAccel = millis();
        Motor1.UpdateForAcceleration();

        if(Motor1.getControlMode() == 1)//send the correct duty cycle to the motors
        {
            sendDutyCycle(Can0, CAN_ID, Motor1.getDuty());
        
        } else {
            //pass for RPM control mode
        }
    }

    


    //-------------------//
    // Command Receiving //
    //-------------------//
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////    //\\        ////    //////////      //
    //    //             //  \\    //  //    //        //    //
    //    //             //    \\//    //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //      /////////    //            //    //////////      //
    //                                                       //
    //-------------------------------------------------------//
    //
    // Commands will be received as a comma separated value string
    // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"

    // Allows to use the same code to parse input wherever it comes from
    bool inputAvailable = false;
    int inputMethod;
    if(Serial.available()) {
        inputAvailable = true;
        inputMethod = 0;
    } else if(Serial4.available()) {
        inputAvailable = true;
        inputMethod = 4;
    }

    if (inputAvailable) {

        // Output to be sent to either Serial or Serial4 depending on which was used
        String output = "";


        String input;
        if(inputMethod == 0)  // Input comes via USB
            input = Serial.readStringUntil('\n'); //Take str input from Serial
        
        else if(inputMethod == 4)  // Input comes via UART from Arm
            input = Serial4.readStringUntil('\n'); //Take str input from UART
        
        
        input.trim();
        std::vector<String> args = parseInput(input, ',');
        String command = args[0].toLowerCase();


        // Comes from ROS -> socket raspi ->UART-> socket teensy 4.1 ->UART-> FAERIE
        if(command == "faerie") { 
            //remove first argument, which is "faerie" to tell socket teensy to redirect to faerie
            args.erase(args.begin()); 
            // Our command is not "faerie"
            command = args[0].toLowerCase();
        }

        //--------------------//
        // Command Processing //
        //--------------------//

        //
        // Commands have been documented here:
        // https://docs.google.com/spreadsheets/d/16RYq-beKFbWoqob2tEWtEd6SiWK38EuTyW-QRfZV-Ow/edit#gid=532390044
        //
        
        //---------//
        // general //
        //---------//
        /**/ if(command == "ping") {
            // Serial.println("pong");
            output += "pong\n";
        }
        
        else if(command == "time") {
            // Serial.println(millis());
            output += millis();
            output += '\n';
        }
        
        else if(command == "line") {
            // Serial.println("-------------");
            output += "-------------";
        }

        else if(command == "led") {
            if(args[1] == "on")
                digitalWrite(LED_PIN, HIGH);
            else
                digitalWrite(LED_PIN, LOW);
        }

        //------//
        // ctrl //
        //------//
        else if(command == "ctrl") { // ctrl //
            String subcommand = args[1];

            

            /**/ if(subcommand == "duty") {
                // CW/+ = CLOSE, CCW/- = OPEN
                Motor1.setDuty(args[2].toFloat());
            }

            else if(subcommand == "stop") {
                Motor1.setDuty(0);
            }

            else if(subcommand == "id") {
                identifyDevice(Can0, CAN_ID);
            }

            else if(subcommand == "sht_heater") {
                if(args[2] == "on") {
                    digitalWrite(LED_PIN, HIGH);
                } 
                else {
                    digitalWrite(LED_PIN, LOW);
                }
            }

            else if(subcommand == "laser") {
                if(args[2] == "on") {
                    digitalWrite(LASER_PIN, HIGH);
                }
                else {
                    digitalWrite(LASER_PIN, LOW);
                }
            }

        }
        
        //------//
        // data //
        //------//
        else if(command == "data") { // data //
            String subcommand = args[1];

            /**/ if(subcommand == "sendTemp") {
                float temp = sht31.readTemperature();
                if(!isnan(temp)) {
                    // Serial.println(sht31.readTemperature());
                    output += sht31.readTemperature();
                    output += '\n';
                } else {
                    // Serial.println("Failed to read temperature.");
                    output += "Failed to read temperature.\n";
                }
            }
            
            else if(subcommand == "sendHum") {
                float hum = sht31.readHumidity();
                if(!isnan(hum)) {
                    // Serial.println(sht31.readHumidity());
                    output += sht31.readHumidity();
                    output += '\n';
                } else {
                    // Serial.println("Failed to read humidity.");
                    output += "Failed to read humidity.\n";
                }
            }

            else if(subcommand == "sendHeater") {
                if(sht31.isHeaterEnabled()) {
                    // Serial.println("true");
                    output += "true\n";
                } else {
                    // Serial.println("false");
                    output += "false\n";
                }
            }

            else if(subcommand == "sendSHT") {
                float temp = sht31.readTemperature();
                float hum = sht31.readHumidity();
                // Temperature
                if(!isnan(temp)) { // Temperature
                    // Serial.print("Temp = ");
                    // Serial.print(temp);
                    // Serial.println(" *C");
                    output += "Temp = ";
                    output += temp;
                    output += " *C\n";
                } else {
                    // Serial.println("Failed to read temperature.");
                    output += "Failed to read temperature\n";
                }
                // Humidity
                if(!isnan(hum)) {
                    // Serial.print("Hum = ");
                    // Serial.print(hum);
                    // Serial.println(" %");
                    output += "Hum = ";
                    output += hum;
                    output += " %\n";
                } else {
                    // Serial.println("Failed to read humidity.");
                    output += "Failed to read humidity.\n";
                }
                // Heater state
                // Serial.print("SHT Heater State: ");
                output += "SHT Heater State: ";
                if(sht31.isHeaterEnabled())
                    // Serial.println("ENABLED");
                    output += "ENABLED";
                else
                    // Serial.println("DISABLED");
                    output += "DISABLED\n";
            }
        }

        //--------------//
        // END COMMANDS //
        //--------------//

        if(output.length() > 1) {
            if(inputMethod == 0)
                Serial.print(output);
            else if(inputMethod == 4)
                Serial4.print(output);
        }

    }

}




//-------------------------------------------------------//
//                                                       //
//    ///////////    //\\          //      //////////    //
//    //             //  \\        //    //              //
//    //             //    \\      //    //              //
//    //////         //      \\    //    //              //
//    //             //        \\  //    //              //
//    //             //          \\//    //              //
//    //             //           \//      //////////    //
//                                                       //
//-------------------------------------------------------//


// Parse Arduino String into String vector separated by delim
// Equivalent to Python's .split()
std::vector<String> parseInput(String input, const char delim = ',') {
    //Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813
    
    std::vector<String> args = {};

    while (input.length() > 0) {
        int index = input.indexOf(delim);
        if (index == -1) { // No instance of delim found in input
            args.push_back(input);
            break;
        } else {
            args.push_back(input.substring(0, index));
            input = input.substring(index+1);
        }
    }

    return args;
}


void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
        sendHeartbeat(Can0, CAN_ID);
        threads.delay(12);
        threads.yield();
    }

}