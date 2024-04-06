// Version 0.3 of Teensy Board
// Programmers : Emann Rivero, Tristan McGinnis, August Longhurst
// Lastest Editor : Emann Rivero

// **** Formalities and Starting Information ****

// Purpose: Assemble the Teensy Code here using the current pinout and board
// Things to Note: 
// MOSFETS are High/Low (digital), Slave UART to PI and uses UART for the Lynx device and PTZ camera.
// Things to refresh on:
// UART, Serial Connection, and pin setup
// 

// Basic Includes for Platform IO
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <FastLED.h>


//**** Board Information ****

// Current Pinout for Test Board 1
// Check which need to be set high or low on start up and if they need to be changed
// Format: Teensy Pin :: Connection Name :: Connection Type
// Pin 0  ::     Pi Tx     ::  Teensy Child with Parent Pi :: UART connection where Pi is the master to the Teensy
// Pin 1  ::     Pi Rx     ::  Teensy Child with Parent Pi :: UART connection where Pi is the master to the Teensy
// Pin 7  ::    Lynx Tx    ::    Lynx Servo Control        :: UART with Lynx Servo Library
// Pin 8  ::    Lynx Rx    ::    Lynx Servo Control        :: UART with Lynx Servo Library
// Pin 28 ::    PTZ Tx     ::    PTZ Camera Control        :: UART
// Pin 29 ::    PTZ Rx     ::     PTZ Camera Control       :: UART
// Pin 33 :: Fan1 Mos Gate :: N mosfet Control for 1st Fan :: Digital
// Pin 31 :: Fan2 Mos Gate :: N mosfet Control for 2nd Fan :: Digital
// Pin 32 :: Fan3 Mos Gate :: N mosfet Control for 3rd Fan :: Digital
// Pin 38 ::   1 Mos Gate  :: N mosfet Control for 1st Pump:: Digital
// Pin 39 ::   2 Mos Gate  :: N mosfet Control for 2nd Pump:: Digital
// Pin 40 ::   3 Mos Gate  :: N mosfet Control for 3rd Pump:: Digital
// Pin 41 ::   4 Mos Gate  :: N mosfet Control for 4th Pump:: Digital

// Links for Devices if needed
// Lynx Servo : https://www.digikey.com/en/products/detail/robotshop/RB-LYN-990/12155066
// UART COnnections : https://www.pjrc.com/teensy/td_uart.html,


// Variables
String Commands = "";
String Byte_read = "";
bool commandFlag = 0;
int resetCommand = 12;

void setup() {
  // Pinout Setup
  pinMode(0, OUTPUT);  // Pi Tx   (UART) // UART
  pinMode(1, INPUT);   // Pi Rx   (UART) // UART
  pinMode(7, OUTPUT);  // Lynx TX (UART)
  pinMode(8, INPUT);   // Lynx Rx (UART)
  pinMode(28, OUTPUT); // PTZ TX  (UART)
  pinMode(29, INPUT);  // PTZ RX  (UART)
  pinMode(33, OUTPUT); // Fan Mosfet 1
  pinMode(31, OUTPUT); // Fan Mosfet 2
  pinMode(32, OUTPUT); // Fan Mosfet 3
  pinMode(38, OUTPUT); // Pump mosfet Gate 1
  pinMode(39, OUTPUT); // Pump mosfet Gate 2
  pinMode(40, OUTPUT); // Pump mosfet Gate 3
  pinMode(41, OUTPUT); // Pump mosfet Gate 4

  // Setting Default Values (MOSFETS ON/OFF == HIGH/LOW)
  digitalWrite(33, LOW); // Fan 1 
  digitalWrite(31, LOW); // Fan 2 
  digitalWrite(32, LOW); // Fan 3 
  digitalWrite(38, LOW); // Pump 1 
  digitalWrite(39, LOW); // Pump 2 
  digitalWrite(40, LOW); // Pump 3 
  digitalWrite(41, LOW); // Pump 4 
  
  // Software Serial Connection Startup and message
  Serial.begin(9600);
  Serial.println("Default Value Set");
}

// Loop section: Determine a speed, create communication lines, create command section
// Setup ROS2
void loop() {
  // Code UART commands into here to activate certain mosfets/mosfets
  // Task to do: Make command to check whther a mosfet is on or off using flag
  // 
  
  // Check for proper communication
  // Test Code for Serial Testing COmment out
   while(Serial.available()>0) {
    Byte_read = Serial.read(); // Info is read one byte at a time
    Commands += String(Byte_read);
    //echoCommand = Commands;
    //int strLength = echoCommand.length();
    //Serial.println(strLength);
    //echoCommand.remove(strLength-1);
    

    recievedCommands(Commands);
    resetCommand = 12;
  }
  if (commandFlag == 1) // Maybe set a timer that if we do not recieve a command in certain amount of time to reset the command byte string
    {
      Commands = "";
      //Serial.println("Command Reset");
      commandFlag = 0;
      resetCommand = 12;
    }
  if (resetCommand == 0)
  {
    Commands = "";
    Serial.println("Reset Commands Activated");
    resetCommand = 12;
  }
  else
  {
    resetCommand -= 1;
  }
  
  delay(1000); // Forcing 1 second delay:
}

// Command Names to be read:
// Fan 1 ON/OFF
// Fan 2 ON/OFF
// Fan 3 ON/OFF
// Pump 1 ON/OFF
// Pump 2 ON/OFF
// Pump 3 ON/OFF
// Pump 4 ON/OFF
// 
// Things to do:
// Creating a list of commands on drive
// Create MOSFET Commands
// Learn Lynx servo using datasheet
// Lynx Commands : Up and Down is function
// Establish Setup for Uart and create master/slave connection

// New Function: Controls Commands, Command section
void recievedCommands(String commands)
{
  // Note to self: Find a more efficient manner of reading commands
  // New Program Method: fan1,0,0,0 (1 = ON, 0 = OFF)
  if(commands == "fan1,0,0,0")
  {
    
  }

  if(commands == "fan1_on")
  {
    digitalWrite(33, HIGH); // Fan 1 ON
    Serial.println("Fan1ON");
    commandFlag = 1;
  }
  if(commands == "fan1_off")
  {
    digitalWrite(33, LOW); // Fan 1 OFF
    Serial.println("Fan1OFF");
    commandFlag = 1;
  }


  if(commands == "fan2_on")
  {
    digitalWrite(31, HIGH); // Fan 2 ON
    Serial.println("Fan2ON");
    commandFlag = 1;
  }
  if(commands == "fan2_off")
  {
    digitalWrite(31, LOW); // Fan 2 OFF
    Serial.println("Fan2OFF");
    commandFlag = 1;
  }


  if(commands == "fan3_on")
  {
    digitalWrite(32, HIGH); // Fan 3 ON
    Serial.println("Fan3ON");
    commandFlag = 1;
  }
  if(commands == "fan3_off")
  {
    digitalWrite(32, LOW); // Fan 3 OFF
    Serial.println("Fan3OFF");
    commandFlag = 1;
  }

 
 if(commands == "pump1_on")
  {
    digitalWrite(38, HIGH); // Pump 1 ON
    Serial.println("Pump1ON");
    commandFlag = 1;
  }
  if(commands == "pump1_off")
  {
    digitalWrite(38, LOW); // Pump 1 OFF
    Serial.println("Pump1OFF");
    commandFlag = 1;
  }

  if(commands == "pump2_on")
  {
    digitalWrite(39, HIGH); // Pump 2 ON
    Serial.println("Pump2ON");
    commandFlag = 1;
  }
  if(commands == "pump2_off")
  {
    digitalWrite(39, LOW); // Pump 2 OFF
    Serial.println("Pump2OFF");
    commandFlag = 1;
  }


  if(commands == "pump3_on")
  {
    digitalWrite(40, HIGH); // Pump 3 ON
    Serial.println("Pump3ON");
    commandFlag = 1;
  }
  if(commands == "pump3_off")
  {
    digitalWrite(40, LOW); // Pump 3 OFF
    Serial.println("Pump3OFF");
    commandFlag = 1;
  }


  if(commands == "pump4_on")
  {
    digitalWrite(41, HIGH); // Pump 4 ON
    Serial.println("Pump4ON");
    commandFlag = 1;
  }
  if(commands == "pump4_off")
  {
    digitalWrite(41, LOW); // Pump 4 OFF
    Serial.println("Pump4OFF");
    commandFlag = 1;
  }
  
}
