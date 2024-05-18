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
#include <Servo.h>
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
#define SERVO_PWM_PIN 1
#define COMMS_UART Serial4
#define COMMS_UART_NUM 4

//-----------//
// Constants //
//-----------//

const long SERIAL_BAUD     = 115200;
const long COMMS_UART_BAUD = 9600;

const int SERVO_MIN =  500;
const int SERVO_MAX = 2500;

//------------------------//
// Classes for components //
//------------------------//

// Basic 5V PWM servo for SCABBARD
Servo servo;


// SHT 31 in SCABBARD
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // Faerie HUM/TEMP Sensor


unsigned CAN_ID = 6;

//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(CAN_ID, 1, false, 50, 0.50F);//Drill


// Last millis value that the motor was sent a duty cycle
unsigned long lastAccel;


// Last millis value that sht temp and hum data was sent to socket
uint32_t lastDataSend = 0;


//------------//
// Prototypes //
//------------//

void loopHeartbeats();
void parseInput(String input, std::vector<String>& args, const char delim);
void sendSHTData(void);

//-------------//
// Begin Setup //
//-------------//

void setup() {

    //-----------------//
    // Initialize Pins //
    //-----------------//
  
    Serial.begin(SERIAL_BAUD);
    COMMS_UART.begin(COMMS_UART_BAUD); // for comms with Arm Socket Teensy/RasPi

    // Teensy built-in LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Faerie drill lasers
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);

    // SCABBARD Servo
    servo.attach(SERVO_PWM_PIN, SERVO_MIN, SERVO_MAX);

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



    // Send SHT temp and hum data to socket once per second
    if(millis()-lastDataSend >= 1000) {
        lastDataSend = millis();

        sendSHTData();
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
    } else if(COMMS_UART.available()) {
        inputAvailable = true;
        inputMethod = COMMS_UART_NUM;
    }

    if (inputAvailable) {

        // Output to be sent to either Serial or COMMS_UART depending on which was used
        String output = "";

        String input;
        if(inputMethod == 0)  // Input comes via USB
            input = Serial.readStringUntil('\n'); //Take str input from Serial
        
        else if(inputMethod == COMMS_UART_NUM)  // Input comes via UART from Arm
            input = COMMS_UART.readStringUntil('\n'); //Take str input from UART

        
        input.trim();
        std::vector<String> args = {};
        parseInput(input, args, ',');
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
            String subcommand = args[1].toLowerCase();
            

            /**/ if(subcommand == "duty") {
                // CW/+ = CLOSE, CCW/- = OPEN
                Motor1.setDuty(args[2].toFloat());
            }

            else if(subcommand == "servo") {
                servo.write(args[2].toInt());
            }

            else if(subcommand == "stop") {
                Motor1.setDuty(0);
            }

            else if(subcommand == "id") {
                identifyDevice(Can0, CAN_ID);
            }

            else if(subcommand == "shtheater") {
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
            String subcommand = args[1].toLowerCase();


            /**/ if(subcommand == "sendtemp") {
                float temp = sht31.readTemperature();
                if(!isnan(temp)) {
                    // Serial.println(sht31.readTemperature());
                    output += temp;
                    output += '\n';
                } else {
                    // Serial.println("Failed to read temperature.");
                    output += "Failed to read temperature.\n";
                }
            }
            
            else if(subcommand == "sendhum") {
                float hum = sht31.readHumidity();
                if(!isnan(hum)) {
                    // Serial.println(sht31.readHumidity());
                    output += hum;
                    output += '\n';
                } else {
                    // Serial.println("Failed to read humidity.");
                    output += "Failed to read humidity.\n";
                }
            }

            else if(subcommand == "sendheater") {
                if(sht31.isHeaterEnabled()) {
                    // Serial.println("true");
                    output += "true\n";
                } else {
                    // Serial.println("false");
                    output += "false\n";
                }
            }

            else if(subcommand == "sendsht") {
                output += "faeriesht,";

                float temp = sht31.readTemperature();
                float hum = sht31.readHumidity();

                // Verify temperature reading
                if(!isnan(temp))
                    output += temp;
                else
                    output += "FAIL";
                
                output += ',';

                // Verify humidity reading
                if(!isnan(hum))
                    output += hum;
                else
                    output += "FAIL";
                
                output += '\n';
            }
        }

        //--------------//
        // END COMMANDS //
        //--------------//

        if(output.length() > 1) {
            if(inputMethod == 0)
                Serial.print(output);
            else if(inputMethod == COMMS_UART_NUM)
                COMMS_UART.print(output);
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


// Parse `input` into `args` separated by `delim`
// Ex: "ctrl,led,on" => {ctrl,led,on}
// Equivalent to Python's `.split()`
void parseInput(const String input, std::vector<String>& args, const char delim = ',') {
    //Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813/9

    // Index of previously found delim
    int lastIndex = -1;
    // Index of currently found delim
    int index = -1;
    // because lastIndex=index, lastIndex starts at -1, so with lastIndex+1, first search begins at 0

    // if empty input for some reason, don't do anything
    if(input.length() == 0)
        return;

    unsigned count = 0;
    while (count++, count < 200 /*arbitrary limit on number of delims because while(true) is scary*/) {
        lastIndex = index;
        // using lastIndex+1 instead of input = input.substring to reduce memory impact
        index = input.indexOf(delim, lastIndex+1);
        if (index == -1) { // No instance of delim found in input
            // If no delims are found at all, then lastIndex+1 == 0, so whole string is passed.
            // Otherwise, only the last part of input is passed because of lastIndex+1.
            args.push_back(input.substring(lastIndex+1));
            // Exit the loop when there are no more delims
            break;
        } else { // delim found
            // If this is the first delim, lastIndex+1 == 0, so starts from beginning
            // Otherwise, starts from last found delim with lastIndex+1
            args.push_back(input.substring(lastIndex+1, index));
        }
    }

    // output is via vector<String>& args
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

// Poll SHT and send temperature and humidity data to socket over UART
// in format "faeriesht,`{temperature}`,`{humidity}`"
void sendSHTData(void) {

    COMMS_UART.print("faeriesht,");
    
    float temp = sht31.readTemperature();
    float hum = sht31.readHumidity();

    // Verify temperature reading
    if(!isnan(temp))
        COMMS_UART.print(temp);
    else
        COMMS_UART.print("FAIL");
    
    COMMS_UART.print(',');

    // Verify humidity reading
    if(!isnan(hum))
        COMMS_UART.print(hum);
    else
        COMMS_UART.print("FAIL");
    
    COMMS_UART.print('\n');

}