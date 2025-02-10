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
LSS myLSS_1_1 = LSS(LSS_ID), myLSS_1_2 = LSS(LSS_ID), myLSS_1_3 = LSS(LSS_ID);
hw_timer_t *Timer0_Cfg = NULL, *Timer1_Cfg = NULL;

//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;
unsigned long fanTimer, fansTimer, fanTimer_1, fanTimer_2, fanTimer_3;
unsigned long pumpTimer, pumpsTimer, pumpTimer_1, pumpTimer_2, pumpTimer_3;
bool fanON = 0, fansOn = 0, fanOn_1 = 0, fanOn_2 = 0, fanOn_3 = 0; // 1 = long = 2seconds, 0 = short = 0.5s
bool pumpON = 0, pumpON_1 = 0, pumpON_2 = 0, pumpON_3 = 0;
unsigned long currTime;
unsigned long prevFanTime = 0, prevFanTime_1 = 0, prevFanTime_2 = 0, prevFanTime_3 = 0;

//--------------//
//  Prototypes  //
//--------------//
void activateCapSer(int num, int truFal);
void activatePump(int num, int truFal);
void activateFan(int num, int truFal);
void initialize();

void IRAM_ATTR Timer0_ISR()
{
  if (fansOn && (currTime - prevFanTime >= fansTimer))
  {
    digitalWrite(19, LOW);
    digitalWrite(20, LOW);
    digitalWrite(21, LOW);
    fansOn = 0;
    prevFanTime = currTime;
  }
  else if (fanOn_1 && (currTime - prevFanTime_1 >= fanTimer_1))
  {
    digitalWrite(19, LOW);
    fanOn_1 = 0;
    prevFanTime_1 = currTime;
  }
  else if (fanOn_2 && (currTime - prevFanTime_2 >= fanTimer_2))
  {
    digitalWrite(20, LOW);
    fanOn_2 = 0;
    prevFanTime_2 = currTime;
  }
  else if (fanOn_3 && (currTime - prevFanTime_3 >= fanTimer_3))
  {
    digitalWrite(21, LOW);
    fanOn_3 = 0;
    prevFanTime_3 = currTime;
  }
}

void IRAM_ATTR Timer1_ISR()
{
  digitalWrite(LED_BUILTIN, !ledState);
}
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
  initialize();
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
  // #ifdef BLINK
  //   if (millis() - lastBlink > 1000)
  //   {
  //     lastBlink = millis();
  //     ledState = !ledState;
  //     digitalWrite(LED_BUILTIN, ledState);
  //   }
  // #endif

  currTime = millis();

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
      digitalWrite(LED_BUILTIN, !ledState);
      // if (parCmd[1] == "on")
      //   digitalWrite(LED_BUILTIN, HIGH);
      // else if (parCmd[1] == "off")
      //   digitalWrite(LED_BUILTIN, LOW);
      // else if (parCmd[1] == "toggle")
      // {
      //   ledState = !ledState;
      //   digitalWrite(LED_BUILTIN, ledState);
      // }
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
        fansON = 1;
        fansTimer = parCmd[4].toInt();
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
        // fanOn_1 = 1;
        // fanTimer_1 = parCmd[4].toInt();
        switch (parCmd[1].toInt())
        {
        case 1:
          fanOn_1 = 1;
          fanTimer_1 = parCmd[3].toInt();
          break;
        case 1:
          fanOn_2 = 1;
          fanTimer_2 = parCmd[3].toInt();
          break;
        case 3:
          fanOn_3 = 1;
          fanTimer_3 = parCmd[3].toInt();
          break;
        default:
          break;
        }
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
      switch (parCmd[2].toInt();)
      {

      case 1:
        if (parCmd[2] == "Relative")
        {
          myLSS_1.moveRelative(((parCmd[2]).toInt()) * 10);
          Serial.println(myLSS_1.getPosition());
        }
        else if (parCmd[2] == "FullRetract")
        {
          myLSS_1.moveRelative(((parCmd[2]).toInt()) * 10);
          Serial.println(myLSS_1.getPosition());
        }
        else if (parCmd[2] == "FullRetract")
        {
          myLSS_1.move(-900);
          Serial.println("Full Retractig CITADEL arm");
        }
        else if (parCmd[2] == "Half")
        {
          myLSS_1.move(0);
          Serial.println("Setting arm to half extend");
        }
        else if (parCmd[2] == "Extend")
        { //
          myLSS_1.moveRelative(20);
          Serial.println("Extending CITADEL arm");
        }
        else if (parCmd[2] == "Retract")
        { //
          myLSS_1.moveRelative(-20);
          Serial.println("Retractig CITADEL arm");
        }
        else if (parCmd[2] == "Reset")
        {
          myLSS_1.reset();
          Serial.println("Servo Reset");
        }
        break;

      case 2:
        if (parCmd[2] == "Relative")
        {
          myLSS_2.moveRelative(((parCmd[2]).toInt()) * 10);
          Serial.println(myLSS_1.getPosition());
        }
        else if (parCmd[2] == "FullRetract")
        {
          myLSS_2.moveRelative(((parCmd[2]).toInt()) * 10);
          Serial.println(myLSS_1.getPosition());
        }
        else if (parCmd[2] == "FullRetract")
        {
          myLSS_2.move(-900);
          Serial.println("Full Retractig CITADEL arm");
        }
        else if (parCmd[2] == "Half")
        {
          myLSS_2.move(0);
          Serial.println("Setting arm to half extend");
        }
        else if (parCmd[2] == "Extend")
        { //
          myLSS_2.moveRelative(20);
          Serial.println("Extending CITADEL arm");
        }
        else if (parCmd[2] == "Retract")
        { //
          myLSS_2.moveRelative(-20);
          Serial.println("Retractig CITADEL arm");
        }
        else if (parCmd[2] == "Reset")
        {
          myLSS_2.reset();
          Serial.println("Servo Reset");
        }
        break;

      case 3:
        if (parCmd[2] == "Relative")
        {
          myLSS_3.moveRelative(((parCmd[2]).toInt()) * 10);
          Serial.println(myLSS_1.getPosition());
        }
        else if (parCmd[2] == "FullRetract")
        {
          myLSS_3.moveRelative(((parCmd[2]).toInt()) * 10);
          Serial.println(myLSS_1.getPosition());
        }
        else if (parCmd[2] == "FullRetract")
        {
          myLSS_3.move(-900);
          Serial.println("Full Retractig CITADEL arm");
        }
        else if (parCmd[2] == "Half")
        {
          myLSS_3.move(0);
          Serial.println("Setting arm to half extend");
        }
        else if (parCmd[2] == "Extend")
        { //
          myLSS_3.moveRelative(20);
          Serial.println("Extending CITADEL arm");
        }
        else if (parCmd[2] == "Retract")
        { //
          myLSS_3.moveRelative(-20);
          Serial.println("Retractig CITADEL arm");
        }
        else if (parCmd[2] == "Reset")
        {
          myLSS_3.reset();
          Serial.println("Servo Reset");
        }
        break;

      default:
        break;
      }
    }
    // else if (parCmd[0] == "servoRelative")
    // { // Is looking for a command that looks like "srvR,x" where is how far you want the servo to move
    //   // activate servo with speed being the token
    //   // myLSS_1.move((parCmd[1]).toInt());
    //   myLSS_1.moveRelative(((parCmd[1]).toInt()) * 10);
    //   Serial.println(myLSS_1.getPosition());
    // }
    // else if (parCmd[0] == "servoFullRetract")
    // { //
    //   myLSS_1.move(-900);
    //   Serial.println("Full Retractig CITADEL arm");
    // }
    // else if (parCmd[0] == "servoHalf")
    // { //
    //   myLSS_1.move(0);
    //   Serial.println("Setting arm to half extend");
    // }
    // else if (parCmd[0] == "servoExtend")
    // { //
    //   myLSS_1.moveRelative(20);
    //   Serial.println("Extending CITADEL arm");
    // }
    // else if (parCmd[0] == "servoRetract")
    // { //
    //   myLSS_1.moveRelative(-20);
    //   Serial.println("Retractig CITADEL arm");
    // }
    // else if (parCmd[0] == "servoReset")
    // {
    //   myLSS_1.reset();
    //   Serial.println("Servo Reset");
    // }
    else if (parCmd[0] == "shutdown")
    {
      for (int i = 19; i < 22; i++)
        digitalWrite(i, LOW);
      digitalWrite(25, LOW);
      digitalWrite(20, LOW);
      digitalWrite(22, LOW);
      digitalWrite(23, LOW);
      digitalWrite(17, LOW);
      digitalWrite(11, LOW);
      digitalWrite(12, LOW);
      digitalWrite(21, LOW);
      digitalWrite(24, LOW);
      digitalWrite(18, LOW);
      
      myLSS_1.reset();
      myLSS_2.reset();
      myLSS_3.reset();
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

// void activateFan(int num, int truFal)
// {
//   digitalWrite(31 + num, truFal);
// }

// void activatePump(int num, int truFal)
// {
//   digitalWrite(39 + num, truFal);
// }

void initialize()
{
  //--------//
  //  Pins  //
  //--------//

  pinMode(0, OUTPUT); // Pi Tx   (UART) // UART
  pinMode(1, INPUT);  // Pi Rx   (UART) // UART
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    ;
  digitalWrite(LED_PIN, HIGH);

  delay(2000);
  digitalWrite(LED_PIN, LOW);

  // Fans
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  // Vibrator
  pinMode(25, OUTPUT);

  // Pumps
  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);

  digitalWrite(31, LOW); // Fan 1
  digitalWrite(32, LOW); // Fan 2
  digitalWrite(33, LOW); // Fan 3

  digitalWrite(25, LOW); // Vibrator

  digitalWrite(20, LOW); // Pump 1
  digitalWrite(22, LOW); // Pump 2
  digitalWrite(23, LOW); // Pump 3
  digitalWrite(17, LOW); // Pump 4

  //------------------//
  //  Communications  //
  //------------------//

  Serial.begin(SERIAL_BAUD);
  LSS::initBus(LSS_SERIAL, LSS_BAUD);
  delay(2000);
  myLSS_1.setAngularStiffness(0);
  myLSS_1.setAngularHoldingStiffness(0);
  myLSS_1.setAngularAcceleration(15);
  myLSS_1.setAngularDeceleration(15);

  myLSS_2.setAngularStiffness(0);
  myLSS_2.setAngularHoldingStiffness(0);
  myLSS_2.setAngularAcceleration(15);
  myLSS_2.setAngularDeceleration(15);

  myLSS_3.setAngularStiffness(0);
  myLSS_3.setAngularHoldingStiffness(0);
  myLSS_3.setAngularAcceleration(15);
  myLSS_3.setAngularDeceleration(15);

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 5000, true);
  timerAlarmEnable(Timer0_Cfg);
  Timer1_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, 1000000, true);
  timerAlarmEnable(Timer1_Cfg);

  //-----------//
  //  Sensors  //
  //-----------//

  //--------------------//
  //  Misc. Components  //
  //--------------------//
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