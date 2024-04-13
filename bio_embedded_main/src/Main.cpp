// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <Wire.h>
#include "SHTSensor.h"
// Our own resources
#include "AstraMotors.h"
#include "AstraCAN.h"
#include "TeensyThreads.h"

SHTSensor sht;

using namespace std;


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(2, 1, false, 50, 1.00F);// The Motor



//Prototypes

void loopHeartbeats();

unsigned long clockTimer = millis();


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);
    digitalWrite(LED_PIN, LOW);

    // Initalization for using CAN with the sparkmax
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();


  //--------------------//
  // Initialize Sensors //
  //--------------------//

  Wire.begin();
  Serial.begin(9600);
  delay(1000); // let serial console settle

  if (sht.init()) {
      Serial.print("init(): success\n");
  } else {
      Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);

  //Start heartbeat thread
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
  

  //----------------------------------//
  // Runs something at a set interval //
  // Useful for testing               //
  //----------------------------------//

  if(0){
    if((millis()-clockTimer)>50){
      clockTimer = millis();

    }
  }


  //------------------//
  // Command Receiving //
  //------------------//
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
  // The giant CMD helps with finding this place
  //
  // Commands will be received as a comma separated value string
  // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
  // The program parses the string so that each piece of data can be used individually
  // For examples of parsing data you can use the link below
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Command is equal to a line in the serial
    command.trim();                                 // I don't know why this is here, but it is important
    string delimiter = ",";                         // The key that decides where the command should be split
    size_t pos = 0;                                 // Standard parse variable
    string token;                                   // The current piece of the string being used.
    string token2;                                  // A secondary piece of the string saved.
    string scommand = command.c_str();              // Converts the Arduino String into a C++ string since they are different things
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
    String prevCommand;

    if (token == "ctrl") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 2){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            
            sendDutyCycle(Can0, 1, stof(token));
            
            scommand.erase(0, pos + delimiter.length());
          }

          
        }else{
          //pass if command if control command is same as previous
        }
    }else if (token == "data") {  // Send data out
        
          scommand.erase(0, pos + delimiter.length());
          prevCommand = command;
          pos = scommand.find(delimiter);
          token = scommand.substr(0, pos);


          if(token == "sendData"){ // data,sendGPS

            Serial.print(sht.getTemperature(), 2);
            Serial.print(sht.getHumidity(), 2);

          }else if(token == "sendTemp"){ // data,sendIMU

           Serial.print(sht.getTemperature(), 2);

          }else if(token == "sendHum"){ // data,sendBMP

            Serial.print(sht.getHumidity(), 2);

          }
        
    } else if (token == "ping") {
      Serial.println("pong");
    } else if (token == "time") {
      Serial.println(millis());
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



// Magic that makes the SparkMax work with CAN
void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
      sendHeartbeat(Can0, 1);
      threads.delay(3);
      sendHeartbeat(Can0, 2);
      threads.delay(3);
      sendHeartbeat(Can0, 3);
      threads.delay(3);
      sendHeartbeat(Can0, 4);
      threads.delay(3);
      threads.yield();
    }
}