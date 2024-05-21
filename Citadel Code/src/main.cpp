// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <LSS.h>
#include <vector>

#define LSS_ID        (3)
#define LSS_BAUD    (LSS_DefaultBaud)
#define LSS_SERIAL    (Serial2)

LSS myLSS = LSS(LSS_ID);
using namespace std;

#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

//Prototypes
void activateCapSer(int num, int truFal);
void activatePump(int num, int truFal);
void activateFan(int num, int truFal);

std::vector<String> parseInput(String input, const char delim);


unsigned long fanTimer;
unsigned long pumpTimer;
bool fanON = 0; // 1 = long = 2seconds, 0 = short = 0.5s
bool pumpON = 0;


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

    digitalWrite(37, LOW); // Pump 1 
    digitalWrite(38, LOW); // Pump 2 
    digitalWrite(40, LOW); // Pump 3 
    digitalWrite(41, LOW); // Pump 4 

  //-----------------------//
  // Initialize Lynx Servo //
  //-----------------------//

  LSS::initBus(LSS_SERIAL, LSS_BAUD);

	// Initialize LSS output to position 0.0
	// myLSS.move(0);

	// Wait for it to get there
	delay(2000);

	// Lower output servo stiffness & accelerations
	myLSS.setAngularStiffness(0);
	myLSS.setAngularHoldingStiffness(0);
	myLSS.setAngularAcceleration(15);
	myLSS.setAngularDeceleration(15);

	// Make input servo limp (no active resistance)
	//myLSS_Input.limp();

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

  
  if((pumpON)&&(pumpTimer < millis())){
    for(int i = 0; i <= 2; i++){
      activatePump(i,0);
    } 
    pumpON = 0;
    Serial.println("Stopping Pumps");
  }
  if((fanON)&&(fanTimer < millis())){
    for(int i = 0; i <= 2; i++){
      activateFan(i,0);
    } 
    fanON = 0;
    Serial.println("Stopping Fans");
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
    command.trim();
    String prevCommand;

    vector<String> parCmd = {};
    parCmd = parseInput(command, ',');

    // Fans
    if (parCmd[0] == "fan1") {                          // Is looking for a command that looks like "fan,0,0,0,time" 
      digitalWrite(31, 1);
      fanON = 1;
      fanTimer = millis() + parCmd[1].toInt(); 
      Serial.println("Fan1 Activated");

    } else if (parCmd[0] == "fan2") {                          // Is looking for a command that looks like "fan,0,0,0,time"
      digitalWrite(32, 1);
      fanON = 1;
      fanTimer = millis() + parCmd[1].toInt(); 
      Serial.println("Fan2 Activated");

    } else if (parCmd[0] == "fan3") {                          // Is looking for a command that looks like "fan,0,0,0,time"
      digitalWrite(33, 1);
      fanON = 1;
      fanTimer = millis() + parCmd[1].toInt(); 
      Serial.println("Fan3 Activated");

    // Pumps
    } else if(parCmd[0] == "pump1") {                          // Is looking for a command that looks like "pump,0,0,0,time"
      digitalWrite(39, 1);
      pumpON = 1;
      pumpTimer = millis() + parCmd[1].toInt();
      Serial.println("Pump1 Activated");

    } else if(parCmd[0] == "pump2") {                          // Is looking for a command that looks like "pump,0,0,0,time"
      digitalWrite(40, 1);
      pumpON = 1;
      pumpTimer = millis() + parCmd[1].toInt();
      Serial.println("Pump2 Activated");
      
    } else if(parCmd[0] == "pump3") {                          // Is looking for a command that looks like "pump,0,0,0,time"
      digitalWrite(41, 1);
      pumpON = 1;
      pumpTimer = millis() + parCmd[1].toInt();
      Serial.println("Pump3 Activated");
      
    } else if (parCmd[0] == "servoRelative") {         // Is looking for a command that looks like "srvR,x" where is how far you want the servo to move
      //activate servo with speed being the token
      //myLSS.move((parCmd[1]).toInt());
      myLSS.moveRelative(((parCmd[1]).toInt())*10);
      Serial.println(myLSS.getPosition());
    } else if (parCmd[0] == "servoFullRetract") {         // 
      myLSS.move(-900);
      Serial.println("Full Retractig CITADEL arm");
    } else if (parCmd[0] == "servoHalf") {         // 
      myLSS.move(0);
      Serial.println("Setting arm to half extend");
    } else if (parCmd[0] == "servoExtend") {         // 
      myLSS.moveRelative(20);
      Serial.println("Extending CITADEL arm");
    } else if (parCmd[0] == "servoRetract") {         // 
      myLSS.moveRelative(-20);
      Serial.println("Retractig CITADEL arm");
    } else if (parCmd[0] == "servoReset") {
      myLSS.reset();
      Serial.println("Servo Reset");
    } else if (parCmd[0] == "ping") {
      Serial.println("pong");
    } else if (parCmd[0] == "time") {
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



void activatePump(int num, int truFal){
  digitalWrite(39+num, truFal);
}

std::vector<String> parseInput(String input, const char delim) {
    //Parse into array separated by delim
    //Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813
    std::vector<String> args = {};
    //String args[20];
    //int argCount = 0;
    while (input.length() > 0) {
        int index = input.indexOf(delim);
        if (index == -1) { // No space found
            args.push_back(input);
            break;
        } else {
            args.push_back(input.substring(0, index));
            input = input.substring(index+1);
        }
    }

    return args;
}