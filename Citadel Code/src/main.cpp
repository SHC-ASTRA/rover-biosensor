/**
 * @file Template.cpp
 * @author your name (you@domain.com)
 * @brief description
 *
 */

//------------//
//  Includes  //
//------------//

#include "AstraMisc.h"
#include "project/TEMPLATE.h"
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <LSS.h>
#include <vector>

//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK
#define LSS_ID (3)
#define LSS_BAUD (LSS_DefaultBaud)
#define LSS_SERIAL (Serial2)
#define LED_PIN 13 // Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

//---------------------//
//  Component classes  //
//---------------------//
LSS myLSS = LSS(LSS_ID);

//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;
unsigned long fanTimer;
unsigned long pumpTimer;
bool fanON = 0; // 1 = long = 2seconds, 0 = short = 0.5s
bool pumpON = 0;

//--------------//
//  Prototypes  //
//--------------//
void activateCapSer(int num, int truFal);
void activatePump(int num, int truFal);
void activateFan(int num, int truFal);

// std::vector<String> parseInput(String prevCommand, const char delim);

//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup()
{
  //--------//
  //  Pins  //
  //--------//

  pinMode(0, OUTPUT); // Pi Tx   (UART) // UART
  pinMode(1, INPUT);  // Pi Rx   (UART) // UART
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);
  digitalWrite(LED_PIN, LOW);

  // Fans
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  // Pumps
  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);

  digitalWrite(31, LOW); // Fan 1
  digitalWrite(32, LOW); // Fan 2
  digitalWrite(33, LOW); // Fan 3

  digitalWrite(39, LOW); // Pump 2
  digitalWrite(40, LOW); // Pump 3
  digitalWrite(41, LOW); // Pump 4

  //------------------//
  //  Communications  //
  //------------------//

  Serial.begin(SERIAL_BAUD);
  LSS::initBus(LSS_SERIAL, LSS_BAUD);
  delay(2000);
  myLSS.setAngularStiffness(0);
  myLSS.setAngularHoldingStiffness(0);
  myLSS.setAngularAcceleration(15);
  myLSS.setAngularDeceleration(15);

  //-----------//
  //  Sensors  //
  //-----------//

  //--------------------//
  //  Misc. Components  //
  //--------------------//
}

//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
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
void loop()
{
  //----------//
  //  Timers  //
  //----------//
#ifdef BLINK
  if (millis() - lastBlink > 1000)
  {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
#endif

  //-------------//
  //  CAN prevCommand  //
  //-------------//

  //------------------//
  //  UART/USB prevCommand  //
  //------------------//
  //
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
  if (Serial.available())
  {
    String prevCommand = Serial.readStringUntil('\n');

    prevCommand.trim();                   // Remove preceding and trailing whitespace
    std::vector<String> parCmd = {};      // Initialize empty vector to hold separated arguments
    parseInput(prevCommand, parCmd, ','); // Separate `prevCommand` by commas and place into parCmd vector
    parCmd[0].toLowerCase();              // Make command case-insensitive
    String command = parCmd[0];           // To make processing code more readable

    //--------//
    //  Misc  //
    //--------//
    /**/ if (command == "ping")
    {
      Serial.println("pong");
    }

    else if (command == "time")
    {
      Serial.println(millis());
    }

    else if (command == "led")
    {
      if (parCmd[1] == "on")
        digitalWrite(LED_BUILTIN, HIGH);
      else if (parCmd[1] == "off")
        digitalWrite(LED_BUILTIN, LOW);
      else if (parCmd[1] == "toggle")
      {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
      }
    }

    //-----------//
    //  Sensors  //
    //-----------//

    //----------//
    //  Motors  //
    //----------//
    // Fan
    if (parCmd[0] == "fans")
    { // Is looking for a command that looks like "fan,0,0,0,time"
      if (command != prevCommand)
      {
        prevCommand = command;
        for (int i = 19; i <= 22; i++)
        {
          digitalWrite(i, parCmd[1].toInt());
        }
        fanON = 1;
        fanTimer = millis() + parCmd[4].toInt();
        Serial.println("Fans Activated");
      }
      // Pump
    }
    else if (parCmd[0] == "fan")
    {
      if (command != prevCommand)
      {
        prevCommand = command;
        digitalWrite(19 + parCmd[1].toInt(), parCmd[2].toInt());
        fanON = 1;
        fanTimer = millis() + parCmd[4].toInt();
        Serial.println("Fans Activated");
      }
    }
    else if (parCmd[0] == "pump")
    { // Is looking for a command that looks like "pump,0,0,0,time"
      if (command != prevCommand)
      {
        prevCommand = command;
        digitalWrite(38 + parCmd[1].toInt(), parCmd[2].toInt());
        pumpON = 1;
        pumpTimer = millis() + parCmd[4].toInt();
        Serial.println("Pumps Activated");
      }
      // Servo
    }
    else if (parCmd[0] == "pumps")
    {
      if (command != prevCommand)
      {
        prevCommand = command;
        for (int i = 38; i < 42; i++)
        {
          digitalWrite(38 + parCmd[1].toInt(), parCmd[2].toInt());
        }
        pumpON = 1;
        pumpTimer = millis() + parCmd[4].toInt();
        Serial.println("Pumps Activated");
      }
    }
    // else if (parCmd[0] == "fan1")
    // { // Is looking for a command that looks like "fan,0,0,0,time"
    //   digitalWrite(31, 1);
    //   fanON = 1;
    //   fanTimer = millis() + parCmd[1].toInt();
    //   Serial.println("Fan1 Activated");
    // }
    // else if (parCmd[0] == "fan2")
    // { // Is looking for a command that looks like "fan,0,0,0,time"
    //   digitalWrite(32, 1);
    //   fanON = 1;
    //   fanTimer = millis() + parCmd[1].toInt();
    //   Serial.println("Fan2 Activated");
    // }
    // else if (parCmd[0] == "fan3")
    // { // Is looking for a command that looks like "fan,0,0,0,time"
    //   digitalWrite(33, 1);
    //   fanON = 1;
    //   fanTimer = millis() + parCmd[1].toInt();
    //   Serial.println("Fan3 Activated");

    //   // Pumps
    // }
    // else if (parCmd[0] == "pump1")
    // { // Is looking for a command that looks like "pump,0,0,0,time"
    //   digitalWrite(39, 1);
    //   pumpON = 1;
    //   pumpTimer = millis() + parCmd[1].toInt();
    //   Serial.println("Pump1 Activated");
    // }
    // else if (parCmd[0] == "pump2")
    // { // Is looking for a command that looks like "pump,0,0,0,time"
    //   digitalWrite(40, 1);
    //   pumpON = 1;
    //   pumpTimer = millis() + parCmd[1].toInt();
    //   Serial.println("Pump2 Activated");
    // }
    // else if (parCmd[0] == "pump3")
    // { // Is looking for a command that looks like "pump,0,0,0,time"
    //   digitalWrite(41, 1);
    //   pumpON = 1;
    //   pumpTimer = millis() + parCmd[1].toInt();
    //   Serial.println("Pump3 Activated");
    // }
    else if (parCmd[0] == "servo")
    {
      if (parCmd[2] == "Relative")
      {
        myLSS.moveRelative(((parCmd[3]).toInt()) * 10);
        Serial.println(myLSS.getPosition());
      }
      else if (parCmd[2] == "FullRetract")
      {
        myLSS.moveRelative(((parCmd[3]).toInt()) * 10);
        Serial.println(myLSS.getPosition());
      }
      else if (parCmd[2] == "FullRetract")
      {
        myLSS.move(-900);
        Serial.println("Full Retractig CITADEL arm");
      }
      else if (parCmd[2] == "Half")
      {
        myLSS.move(0);
        Serial.println("Setting arm to half extend");
      }
      else if (parCmd[2] == "Extend")
      { //
        myLSS.moveRelative(20);
        Serial.println("Extending CITADEL arm");
      }
      else if (parCmd[2] == "Retract")
      { //
        myLSS.moveRelative(-20);
        Serial.println("Retractig CITADEL arm");
      }
      else if (parCmd[2] == "Reset")
      {
        myLSS.reset();
        Serial.println("Servo Reset");
      }
    }
    // else if (parCmd[0] == "servoRelative")
    // { // Is looking for a command that looks like "srvR,x" where is how far you want the servo to move
    //   // activate servo with speed being the token
    //   // myLSS.move((parCmd[1]).toInt());
    //   myLSS.moveRelative(((parCmd[1]).toInt()) * 10);
    //   Serial.println(myLSS.getPosition());
    // }
    // else if (parCmd[0] == "servoFullRetract")
    // { //
    //   myLSS.move(-900);
    //   Serial.println("Full Retractig CITADEL arm");
    // }
    // else if (parCmd[0] == "servoHalf")
    // { //
    //   myLSS.move(0);
    //   Serial.println("Setting arm to half extend");
    // }
    // else if (parCmd[0] == "servoExtend")
    // { //
    //   myLSS.moveRelative(20);
    //   Serial.println("Extending CITADEL arm");
    // }
    // else if (parCmd[0] == "servoRetract")
    // { //
    //   myLSS.moveRelative(-20);
    //   Serial.println("Retractig CITADEL arm");
    // }
    // else if (parCmd[0] == "servoReset")
    // {
    //   myLSS.reset();
    //   Serial.println("Servo Reset");
    // }
    else if (parCmd[0] == "shutdown")
    {
      for(int i=31; i<42; i++)
        digitalWrite(i,0);
      // digitalWrite(31, 0);
      // digitalWrite(32, 0);
      // digitalWrite(33, 0);
      // digitalWrite(39, 0);
      // digitalWrite(40, 0);
      // digitalWrite(41, 0);
      myLSS.reset();
    }
    else if (parCmd[0] == "ping")
    {
      Serial.println("pong");
    }
    else if (parCmd[0] == "time")
    {
      Serial.println(millis());
    }
  }
}

//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//

void activateFan(int num, int truFal)
{
  digitalWrite(31 + num, truFal);
}

void activatePump(int num, int truFal)
{
  digitalWrite(39 + num, truFal);
}

std::vector<String> parseInput(String input, const char delim)
{
  // Parse into array separated by delim
  // Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813
  std::vector<String> args = {};
  // String args[20];
  // int argCount = 0;
  while (input.length() > 0)
  {
    int index = input.indexOf(delim);
    if (index == -1)
    { // No space found
      args.push_back(input);
      break;
    }
    else
    {
      args.push_back(input.substring(0, index));
      input = input.substring(index + 1);
    }
  }

  return args;
}