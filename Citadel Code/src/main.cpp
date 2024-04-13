// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <LSS.h>

#define LSS_BAUD	(LSS_DefaultBaud)
// Choose the proper serial port for your platform
#define LSS_SERIAL	(Serial1)	// ex: Many Arduino boards
//#define LSS_SERIAL	(Serial1)	// ex: Teensy

using namespace std;


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

LSS myLSS_Output = LSS(7);
LSS myLSS_Input = LSS(8);

//Prototypes
void activateCapSer(int num, int truFal);
void activatePump(int num, int truFal);
void activateFan(int num, int truFal);


unsigned long clockTimer = millis();


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//

    pinMode(0, OUTPUT);  // Pi Tx   (UART) // UART
    pinMode(1, INPUT);   // Pi Rx   (UART) // UART

    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);
    digitalWrite(LED_PIN, LOW);

    // Fans
    pinMode(31, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);

    // Pumps
    pinMode(38, OUTPUT);
    pinMode(39, OUTPUT);
    pinMode(40, OUTPUT);
    pinMode(41, OUTPUT);

    digitalWrite(33, LOW); // Fan 1 
    digitalWrite(31, LOW); // Fan 2 
    digitalWrite(32, LOW); // Fan 3 
    digitalWrite(38, LOW); // Pump 1 
    digitalWrite(39, LOW); // Pump 2 
    digitalWrite(40, LOW); // Pump 3 
    digitalWrite(41, LOW); // Pump 4 

  //-----------------------//
  // Initialize Lynx Servo //
  //-----------------------//

  LSS::initBus(LSS_SERIAL, LSS_BAUD);

	// Initialize LSS output to position 0.0
	myLSS_Output.move(0);

	// Wait for it to get there
	delay(2000);

	// Lower output servo stiffness & accelerations
	myLSS_Output.setAngularStiffness(0);
	myLSS_Output.setAngularHoldingStiffness(0);
	myLSS_Output.setAngularAcceleration(15);
	myLSS_Output.setAngularDeceleration(15);

	// Make input servo limp (no active resistance)
	myLSS_Input.limp();

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

    if (token == "fan") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 1){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            
            activateFan(i,stoi(token));
        
            scommand.erase(0, pos + delimiter.length());
          }

          
        }
    }else if(token == "pump") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand)
        {
          scommand.erase(0, pos + delimiter.length());

          prevCommand = command;

          for(int i = 0; i < 3; i+= 1){
            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            
            activatePump(i,stoi(token));
        
            scommand.erase(0, pos + delimiter.length());
          }

          
        }
    }else if (token == "mainServo") {         // Is looking for a command that looks like "ctrl,x" where 0<x<1
      scommand.erase(0, pos + delimiter.length());
      token = scommand.substr(0, pos);
      pos = scommand.find(delimiter);

      //activate servo with speed being the token
      myLSS_Output.move(stoi(token));

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

void activateFan(int num, int truFal){
    if(num == 0){
      digitalWrite(31, truFal);
    }
    if(num == 1){
      digitalWrite(32, truFal); 
    }
    if(num == 2){
      digitalWrite(33, truFal);  
    }
}

void activatePump(int num, int truFal){
    if(num == 0){
      digitalWrite(38, truFal); 
    }
    if(num == 1){
      digitalWrite(39, truFal); 
    }
    if(num == 2){
      digitalWrite(40, truFal); 
    }
    if(num == 3){
      digitalWrite(41, truFal); 
    }
}

// Reminder of how to ask motors
// void turnCCW(){
//   sendDutyCycle(Can0, 2, -0.6);
//   sendDutyCycle(Can0, 4, -0.6);
//   sendDutyCycle(Can0, 1, -0.6);
//   sendDutyCycle(Can0, 3, -0.6);
// }

// Reminder of how to ask motors
// void goForwards(float speed){
//   motorList[0].setDuty(speed);
//   motorList[1].setDuty(speed);
//   motorList[2].setDuty(speed);
//   motorList[3].setDuty(speed);
// }

// Magic that makes the SparkMax work with CAN
// void loopHeartbeats(){
//     Can0.begin();
//     Can0.setBaudRate(1000000);
//     Can0.setMaxMB(16);
//     Can0.enableFIFO();
//     Can0.enableFIFOInterrupt();

//     while(1){
//       sendHeartbeat(Can0, 1);
//       threads.delay(3);
//       sendHeartbeat(Can0, 2);
//       threads.delay(3);
//       sendHeartbeat(Can0, 3);
//       threads.delay(3);
//       sendHeartbeat(Can0, 4);
//       threads.delay(3);
//       threads.yield();
//     }
// }