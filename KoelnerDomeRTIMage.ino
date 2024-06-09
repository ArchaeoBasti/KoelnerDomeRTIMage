/* 
RTI-Mage Main Control: Controls lights and camera shutter on RTI-Mage control box to acquire full RTI dataset.
Version 3.1
10/20/2016
Adds mouse commands to Adafruit Bluetooth HID adapter for shutter control with computer

Copyright (C) 2016  Leszek Pawlowicz leszekmp@gmail.com
See the Hackaday.io project page for more info. https://hackaday.io/project/11951-affordable-reflectance-transformation-imaging-dome

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2.0
as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

This software has been modified under the terms of the GNU General Public License Version 2.0
by Sebastian Hageneuer - http://koelner-dome.de/ in order to fit the needs of the provided hardware.
The changes to the software are noted here, down below:
15/09/2021: Added the subroutine Auto_mode_HeatSufficient() in order to provide a more heat sufficuent
method of ligthing up the LEDs without draining either the Mosfets no the LED drivers for a full circle.
Removed unnesessary functions.
07/02/2022: Changed display to Adafruit 177TFT

This software has been modified under the terms of the GNU General Public License Version 2.0
by Uwe Heinemann. 
The changes to the software are noted here, down below:
23.05.2024: MCP4241
Controller: MEGA2650 R3
Taste1_up, Taste1_down: LED
Taste2_up, Taste2_down: DEL
*/

#include <multiCameraIrControl.h> //Sebastian Setz's library for IR shutter control; http://sebastian.setz.name/arduino/my-libraries/multi-camera-ir-control/
#include <SoftwareSerial.h> //Needed for AdaFruit Bluetooth HID module

//Required for OLED Display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#define TFT_CS    10
#define TFT_RST   8
#define TFT_DC    9

#define COLOR1 ST7735_WHITE
#define COLOR2 ST7735_BLACK
#define LIME 0x8E0E

// 128 x 160 Pixel
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

/* This first set of constants should not be changed unless you know exactly what you're doing; system probably won't work
if any of them are changed. They're used to set the Arduino pin numbers for input/output, and also set essential system constants.
*/
const int LED_Pin = A7; //Controls amount of LED on time after shutter is activated. Left knob
const int Photo_Data_Pin=A8; //Controls amount of time LED is off before next picture, for camera to save data. Right knob
const int Beeper_Pin=25; //Direct connection to beeper
const int USB_Camera_Pin=23;  //pin number for voltage to camera USB/IR trigger
const int Servo_Pin=27;  //Controls servo shutter
const int Bluetooth_Tx = 69; //For Bluetooth HID (Pin A15)
const int Bluetooth_Rx = 68; //For Bluetooth HID (Pin A14
const int Action_Pin=A11; //The number of the action button pin (A13)
const int White_Balance_Pin=A10; //The number of the white balance pin (A12)
const int Shutter_Switch_Pin=7; //Switch pin to go from USB shutter control to IR shutter control
const int Auto_Switch_Pin=6; // the number for the switch pin to go from manual (off) to auto (on)
const int Beeper_Control_Pin=3; //Connects to switch that enables/disables beeper
const int Source_Pin=30; //First pin to P-Channel MOSFETs (Columns)
const int Sink_Pin=31; //First pin to CAT4101s (Rows)
const int Bounce_Time=250; //min delay time after button is depressed, to work around debounce issues; set this to no less than 250

//Following are constants for the OLED display
String Software_version="1.61";
String Software_date="26/05/2024";
String Current_setting="700mA"; //This must be set here - the system has no way to determine the actual output current, which is set manually.

/*Following constants can be adjusted to set camera times and other parameters. Maximum value for unsigned int constants is 65535 (bit more than a minute);
maximum for unsigned long constants is 4294967295, or roughly 49 days.
All times are in milliseconds, thousandths of a second; 1000 milliseconds = one second, 1500 milliseconds =1.5 milliseconds, etc.*/
const unsigned int Min_Data_Transfer_Time=500;   // minimum time after LED goes off to save photo data
const unsigned int Max_Data_Transfer_Time=6000; // maximum time after LED goes off to save photo data
const unsigned int Min_LED_Time=100; //minimum time LED is on for picture to be taken
const unsigned int Max_LED_Time=4000; // maximum time LED is on for picture to be taken; set by USB microscope time
const unsigned long White_Balance_Time=3000; //time for one white balance LED before switching to the next one
const unsigned long White_Balance_Max_Time=24000; //maximum total time for white balance before it turns off (to prevent LED stress)
const unsigned int LED_Pause_Time=500; // wait time for Bluetooth system to get ready
const unsigned long Min_LED_On_Time=500; //Minimum on time for LED in manual mode
const unsigned long Max_LED_On_Time=4000; //Maximum on time in manual mode for LED (to prevent overheating); turn back on with white balance button. Higher values not recommended.
const unsigned int Frequency=1000; //Sets default frequency of beeper (in Hz, cycles per second)
const unsigned int Rows=8; //Number of Rows being used (cathode connection to CAT4101s)
const unsigned int Columns=8; //Number of Columns being used (anode connection to MOSFET driver outputs)
const int Shutter_Voltage_Time=100; //Time for 5V at USB to either fire CHDK or remote wired shutter.

// Variables for shutter time and data transfer time
int LED_Time=Max_LED_Time;
int Data_Transfer_Time=Min_Data_Transfer_Time;
int Current_count; //Counter for LED screen
unsigned long LED_On_Time; // Variable time for LED on in manual mode


/* The following section sets up cameras for use with the IR remote LED. To enable a specific camera, just remove the two slashes "/" in front of the camera make parameters (Canon, Nikon, etc.),
and make sure that there are two slashes in front of every other camera make. You will also need to make a similar change in the IR_Shutter function located further down in the program code */
Nikon NikonCamera(USB_Camera_Pin);
String Camera_type="Nikon";
//Canon CanonCamera(USB_Camera_Pin);
//String Camera_type="Canon";
//CanonWLDC100 CanonWLDC100Camera(USB_Camera_Pin);
//String Camera_type="CanonWLDC100";
//Sony SonyCamera(USB_Camera_Pin);
//String Camera_type="Sony";
//Pentax PentaxCamera(USB_Camera_Pin);
//String Camera_type="Pentax";
//Olympus OlympusCamera(USB_Camera_Pin);
//String Camera_type="Olympus";
//Minolta MinoltaCamera(USB_Camera_Pin);
//String Camera_type="Minolta";

/*Sets digital pin number for Software Serial connection, used with Bluetooth HID module. In this case, digital pin 69 (transmit) is the equivalent of A15 on the Mega2560 board, Dgital pin 68 (A14) is set as receive, even though it's not needed.*/
SoftwareSerial BT = SoftwareSerial(Bluetooth_Rx, Bluetooth_Tx); //Pin A15 is digital 69 Tx, Pin A14 is digital 68 Rx. BT is the output to the Bluetooth module.

// Tastendefinitionen (uh)
const int buttonPin_Taste1_up = 4;    // Taster1: up, Einstellung Zeit "LED"
const int buttonPin_Taste1_down = 5;  // Taster1:down
const int buttonPin_Taste2_up = 2;    // Taster2: up, Einstellung Zeit "DEL"
const int buttonPin_Taste2_down = 3;  // Taster2:down

int buttonState_Taste1_up;    // the current reading from the input pin
int buttonState_Taste1_down;  // the current reading from the input pin
int buttonState_Taste2_up;    // the current reading from the input pin
int buttonState_Taste2_down;  // the current reading from the input pin

int lastButtonState = HIGH;
int lastButtonState_Taste1_up = HIGH;    // the previous reading from the input pin
int lastButtonState_Taste1_down = HIGH;  // the previous reading from the input pin
int lastButtonState_Taste2_up = HIGH;    // the previous reading from the input pin
int lastButtonState_Taste2_down = HIGH;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime_Taste1_up = 0;    // the last time the output pin was toggled
unsigned long lastDebounceTime_Taste1_down = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime_Taste2_up = 0;    // the last time the output pin was toggled
unsigned long lastDebounceTime_Taste2_down = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;  // the debounce time; increase if the output flickers

int status_Taste1_up = 0;
int status_Taste1_down = 0;
int status_Taste2_up = 0;
int status_Taste2_down = 0;

// Einstellungen analoge Eingänge A7, A8

const int analogInPin7 = A7;  // AD_Input A7 
int level1 = 0;               // Einschaltzeit LED

const int analogInPin8 = A8;  // AD_Input A8
int level2 = 0;               // Pausenzeit LED

const int write_protection = 46;  // /Write MCP4241
const int slave_Select_Pin = 53;  // /Chip Select MCP4241
int volatile value_on = 0;        // Einschaltzeit LED
int value_off = 0;                // Pausenzeit LED
byte miso_data = 0;


//--------------------------------------------------------------------------------------------------

void setup() { //INPUT_PULLUP means high is "off", and eliminates need for pulldown resistor

  Serial.begin(9600);

  pinMode(White_Balance_Pin,INPUT_PULLUP); //Left button; starts white balance
  pinMode(Action_Pin, INPUT_PULLUP); //Right button; starts sequence, also switches rows for white balance
  pinMode(Auto_Switch_Pin, INPUT_PULLUP); //Toggle switch to choose Auto to Manual
  pinMode(Shutter_Switch_Pin,INPUT_PULLUP); //Toggle switch to select CHDK or IR/Bluetooth output
  pinMode(Beeper_Control_Pin,INPUT_PULLUP); //Toggle switch to turn beeper on/off
  pinMode(USB_Camera_Pin,OUTPUT); //Cable output to control CHDK, IR/remote/Bluetooth HID
  pinMode(Beeper_Pin,OUTPUT); //Audio output control
    for (int x=0; x < Columns; x++) {
    pinMode(Source_Pin+x*2,OUTPUT);
    digitalWrite(Source_Pin+x*2,LOW);} //Set P MOSFET pins output mode, low
    for (int y=0; y < Rows; y++) {
    pinMode(Sink_Pin+y*2, OUTPUT);
    digitalWrite(Sink_Pin+y*2,LOW);} //Set CAT4101 current sink pins to output mode, low
  pinMode(13,OUTPUT); //On-board LED
  digitalWrite(13,LOW); // Turn off on-board LED

  // Konfiguration Tasten (uh)
  pinMode(buttonPin_Taste1_up, INPUT_PULLUP);    //Tasten für manuelle Bedienung
  pinMode(buttonPin_Taste1_down, INPUT_PULLUP);  // interner Pull-UP ein
  pinMode(buttonPin_Taste2_up, INPUT_PULLUP);
  pinMode(buttonPin_Taste2_down, INPUT_PULLUP);

  // Konfiguration fuer MCP4241 (uh)
  pinMode(write_protection, OUTPUT); //WR
  pinMode(slave_Select_Pin, OUTPUT); //CS
  pinMode(51, OUTPUT);  // SI MCP4241
  pinMode(52, OUTPUT);  // SCK MCP4241
  pinMode(50, INPUT);   // SDO MCP4241

  // initialize SPI (uh)
  SPI.begin();

  // Register TCON write
  // SDO = 0xFF, 0xFF
  digitalWrite(slave_Select_Pin, LOW);
  SPI.transfer(B01000000); //TCON write (CMD: 0x04)
  SPI.transfer(B11111111); //Anschlüsse Pot0, Pot1 Konfiguration,  
                           // DS22059B-page 34
  digitalWrite(slave_Select_Pin, HIGH);

  //NV Wiper 0 read
  spi_read(B00101100);
  value_on = miso_data; //Speicherwert NV Wiper 0 aus EEPROM holen
  Serial.print("value_on: ");
  Serial.println(miso_data);
  
  digitalWrite(slave_Select_Pin, LOW);
  SPI.transfer(B00000000); //Volatile Wiper 0 write
  SPI.transfer(miso_data);
  digitalWrite(slave_Select_Pin, HIGH);
  
  //NV Wiper 1 read
  spi_read(B00111100);
  value_off = miso_data;  //Speicherwert NV Wiper 1 aus EEPROM holen

  digitalWrite(slave_Select_Pin, LOW);
  SPI.transfer(B00010000); //Volatile Wiper 1 write
  SPI.transfer(miso_data);
  digitalWrite(slave_Select_Pin, HIGH);  

  digitalWrite(write_protection, LOW); //Write-Protect einschalten


  tft.initR(INITR_GREENTAB);
  Display_Intro_Text();
  Build_Standby_Screen();
}

//---------------------------------------------------------------------------------------------

void loop() {

Display_Standby_Screen();
if (digitalRead(Action_Pin)==LOW && digitalRead(Auto_Switch_Pin)==LOW) {//switch in manual mode
  Manual_mode();}
if (digitalRead(Action_Pin)==LOW && digitalRead(Auto_Switch_Pin)==HIGH) {//switch in auto mode
  Auto_mode_HeatSufficient();}
if (digitalRead(White_Balance_Pin)==LOW) {//do white balance sequence / focus test
   delay(Bounce_Time);
   White_Balance();}

Mcp4241();

}


//---------------------------------------------------------------------------------------------------
void Auto_mode() {  //Loop for running through LEDs automatically
  Current_count=0; // Keeps count of LED # to pass on to OLED display routine

  for(int y=0; y < Rows; y++) { //Start of rows loop
    digitalWrite(Sink_Pin+y*2,HIGH); //Turn row on
    for (int x=0; x < Columns; x++) { //Start of loop to cycle through columns
      Current_count++;
      Display_Auto_Screen();
      digitalWrite(Source_Pin+x*2, HIGH); //Turn on LED
      if (digitalRead(Shutter_Switch_Pin)==HIGH) { //First part fires IR remote and Bluetooth signal
        BT_Shutter();
        IR_Shutter();} //Fire either IR/Bluetooth shutter ...
      else {USB_Shutter();} //... or USB shutter
      delay(LED_Time); //LED is on for this amount of time
      digitalWrite(Source_Pin+x*2,LOW); //Turns off  LED
      delay(Data_Transfer_Time); //Delay for data transfer in camera
    }//End inner loop
    digitalWrite(Sink_Pin+y*2,LOW); //Turn off row cathode pin before moving on to next row
  }
}
//---------------------------------------------------------------------------------------------------
void Auto_mode_HeatSufficient() {  //Loop for running through LEDs automatically, but heat sufficient
  Current_count=0; // Keeps count of LED # to pass on to OLED display routine

  for (int y=0; y < Rows*2; y+=2) { //Start of rows loop
    int z=0;
    for (int x=0; x < Columns*2; x+=2) { //Start of loop to cycle through columns
      digitalWrite(Source_Pin+x,HIGH); //Turn row on
      if (Sink_Pin+x+y <= 2*(Rows-1)+Sink_Pin) {
        digitalWrite(Sink_Pin+x+y, HIGH); //Turn on LED
      } else {
        digitalWrite(Sink_Pin+z, HIGH); //Turn on LED
      }
      Current_count++;
      Display_Auto_Screen();

      if (digitalRead(Shutter_Switch_Pin)==HIGH) { //First part fires IR remote and Bluetooth signal
        BT_Shutter();
        IR_Shutter();} //Fire either IR/Bluetooth shutter ...
      else {USB_Shutter();} //... or USB shutter

      delay(LED_Time); //LED is on for this amount of time
      digitalWrite(Source_Pin+x,LOW); //Turns off  LED
      if (Sink_Pin+x+y <= 2*(Rows-1)+Sink_Pin) {
        digitalWrite(Sink_Pin+x+y,LOW); //Turn off row cathode pin before moving on to next row
      } else {
        digitalWrite(Sink_Pin+z,LOW); //Turn off row cathode pin before moving on to next row
        z+=2;
      }
      delay(Data_Transfer_Time); //Delay for data transfer in camera
    } //End inner loop
  }
}
//---------------------------------------------------------------------------------------------------
void Manual_mode() {
  unsigned long Previous_Millis; //Time markers for shutting off LED if it hasn't been advanced for a while
  unsigned long Current_Millis; //Sets start time for monitoring total LED on time

  int LED_Status;
  Current_count=0;

  for(int y=0; y < Rows; y++) { //Start of loop to cycle through rows
    digitalWrite(Sink_Pin+y*2,HIGH); //Turn row on
    for (int x=0; x < Columns; x++) { //Start of loop to cycle through columns
      Set_Manual_Time();
      Current_count++;
      digitalWrite(Source_Pin+x*2, HIGH); //Turn on bright LED
      LED_Status=1;
      Display_Manual_Screen(LED_Status);
      if (digitalRead(Shutter_Switch_Pin)==HIGH) {
        /*IR/Bluetooth works in manual mode with switch in either USB or IR mode*/
        BT_Shutter();
        IR_Shutter();} //Fire either IR/Bluetooth shutter ...
      else {USB_Shutter();} //... or USB shutter
      Previous_Millis=millis();
      while (digitalRead(Action_Pin)==HIGH) {
        Current_Millis=millis();
        if ((Current_Millis - Previous_Millis) > LED_On_Time) { //Check duration; if over limit, turn light off to prevent burn-out
          digitalWrite(Source_Pin+x*2,LOW);
          Set_Manual_Time();
          LED_Status=0;
          Display_Manual_Screen(LED_Status);
        }
        if (digitalRead(White_Balance_Pin)==LOW) {
          digitalWrite(Source_Pin+x*2,HIGH);
          Set_Manual_Time();
          LED_Status=1;
          Display_Manual_Screen(LED_Status);
          delay(Bounce_Time);
          Previous_Millis=Current_Millis;
        }
      } //Sit and wait until button is pushed again, with light on
      digitalWrite(Source_Pin+x*2,LOW); //Turn off column LED
    }//End inner loop
    digitalWrite(Sink_Pin+y*2,LOW); //Turn off current row cathode pin before moving on to next row
  }
  delay(Bounce_Time);
}
//----------------------------------------------------------------------------------------------------

void White_Balance() { //Turn on LEDs at top to set white balance and/or illuminate inside of dome; cycle to reduce wear on one LED

unsigned long Start_Millis=millis(); //Start/End used to turn off white balance after a set period of time, to prevent LED burn & wire overheating
unsigned long End_Millis=millis();
unsigned long Previous_Millis=millis(); //Previous/Current used to time LED on duration
unsigned long Current_Millis=millis();

delay(Bounce_Time); //Avoid button bounce
int Sink_Pin_Current = Sink_Pin; //Defines which row pin is on
digitalWrite(Sink_Pin,HIGH); //Turn on first row cathode


Start_Loop:
for (int x=0; x < Columns; x++) { //Start of loop to cycle through Columns
End_Millis=millis(); //Current time
if ((End_Millis-Start_Millis) > White_Balance_Max_Time) {goto Exit_Loop;} //Exits white balance after White_Balance_Max_Time exceeded
    digitalWrite(Source_Pin+x*2, HIGH); //Turn on bright LED
    while ((Current_Millis - Previous_Millis) < White_Balance_Time) { //Check to see how much time has passed
    Current_Millis=millis();
      if (digitalRead(White_Balance_Pin)==LOW) {goto Exit_Loop;} //Escape from loop if white balance button is pushed.
      if (digitalRead(Action_Pin)==LOW) { //Switch to next row if action button is pushed
        digitalWrite(Sink_Pin_Current,LOW); //Turn off current row
        delay(Bounce_Time); //Pause for bounce issues
        Sink_Pin_Current=Sink_Pin_Current + 2; //Increment to next row down
        if (Sink_Pin_Current > (Sink_Pin + Rows*2 - 2)) {Sink_Pin_Current=Sink_Pin;} //If already at the bottom row, moves up to top row
        digitalWrite(Sink_Pin_Current,HIGH); //Turn on next row
        Start_Millis=millis(); //Reset time markers after switch to new row
        End_Millis=millis();
      }
    }

digitalWrite(Source_Pin+x*2,LOW); //Turn off LED
Previous_Millis=Current_Millis; //Reset 0 time to current time
}
//x=0; //When for loop is done, reset x to 0 to start from beginning
goto Start_Loop;


Exit_Loop:
for (int y=0; y < Rows; y++) { //Start of loop to cycle through Rows
digitalWrite(Sink_Pin+y*2,LOW); //Make sure all row voltages are off
}
for (int x=0; x < Columns; x++) { //Start of loop to cycle through Columns
digitalWrite(Source_Pin+x*2,LOW); //Make sure all column voltages are off
}
}

//------------------------------------------------------------------------------------------------------

void Set_Data_Transfer_Time() { //Sets data transfer time delay from potentiometer
int Data_Transfer_Sensor;

Data_Transfer_Sensor=  analogRead(Photo_Data_Pin);
Data_Transfer_Time= map(Data_Transfer_Sensor,0,1023,Min_Data_Transfer_Time,Max_Data_Transfer_Time);
}

//-----------------------------------------------------------------------------------------------------
void Set_LED_Time() { //Sets LED on time from potentiometer
int LED_Sensor;

LED_Sensor=  analogRead(LED_Pin);
LED_Time= map(LED_Sensor,0,1023,Min_LED_Time,Max_LED_Time);
}
//------------------------------------------------------------------------------------------------------
void Set_Manual_Time() { //Sets manual LED time from potentiometer
int LED_Sensor;

LED_Sensor=  analogRead(LED_Pin);
LED_On_Time= map(LED_Sensor,0,1023,Min_LED_On_Time,Max_LED_On_Time);//Limits for LED on time in manual mode
}
//------------------------------------------------------------------------------------------------------

void USB_Shutter() { //Controls shutter for CHDK-enabled cameras directly through USB cable connection
Set_Data_Transfer_Time();
Set_LED_Time();

digitalWrite(USB_Camera_Pin,HIGH); //Turn on 5V to USB Shutter cable
delay(Shutter_Voltage_Time); //5V Time to fire camera
digitalWrite(USB_Camera_Pin,LOW); //Turns off 5V to USB Shutter cable

}

//------------------------------------------------------------------------------------------------------

void IR_Shutter() { //Controls shutter for IR-controlled cameras through IR LED attached to USB cable
Set_Data_Transfer_Time();
Set_LED_Time();

//Section for IR remotes
//Remove double-slash ("//") from front of camera make being used to activate; place double-slash in front of unneeded cameras to de-activate. Make sure you've also done this in the initialization section at top.
NikonCamera.shotNow();
//CanonCamera.shotNow();
//CanonWLDC100Camera.shotNow();
//SonyCamera.shotNow();
//PentaxCamera.shotNow();
//OlympusCamera.shotNow();;
//MinoltaCamera.shotNow();

}
//________________________________________________________________________________________________
void BT_Shutter() { //Sets commands for Adafruit Bluetooth HID module to fire shutter using computer
          //Fires shutter with Alt-f, s command
          //BT.write(0xE2); //Alt key
          //BT.write(0x66); //f key
          //delay(25);
          //BT.write(0x73); //s key

          //Fires shutter with left mouse button
          delay(LED_Pause_Time);
          mouseCommand(0x1,0,0); //Left button down
          delay(25);
          mouseCommand(0,0,0); //Release button
}

//------------------------------------------------------------------------------------------------------
void mouseCommand(uint8_t buttons, uint8_t x, uint8_t y) { //Sends mouse commands from Adafruit HID Bluetooth module
  BT.write(0xFD);
  BT.write((byte)0x00);
  BT.write((byte)0x03);
  BT.write(buttons);
  BT.write(x);
  BT.write(y);
  BT.write((byte)0x00);
  BT.write((byte)0x00);
  BT.write((byte)0x00);

}

//---------------------------------------------------------------------------------------------------------------------------------------
void Display_Intro_Text() {  //Shows intro OLED screen

  String Col_string=String(Columns);
  String Row_string=String(Rows);
  String LED_total=String(Columns*Rows);

  tft.fillScreen(ST7735_WHITE);
  tft.setRotation(3);

  int startX = 60;
  int startY = 30;
  tft.setTextColor(ST7735_BLACK);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextSize(1);
  tft.setCursor(startX,startY);
  tft.println("Kolner");
  tft.fillCircle(startX+20, startY-15, 1, ST7735_BLACK);
  tft.fillCircle(startX+25, startY-15, 1, ST7735_BLACK);
  tft.setCursor(startX,startY + 25);
  tft.setFont(&FreeSansBold18pt7b);
  tft.println("ome");
  tft.setRotation(2);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(95-startY,startX-4);
  tft.println("D");
  tft.setRotation(3);
  tft.fillTriangle(startX-14, startY, startX-18, startY-25, startX-22, startY, ST7735_BLACK);
  tft.fillTriangle(startX-11, startY-2, startX-13, startY-14, startX-15, startY-2, ST7735_BLACK);
  tft.fillTriangle(startX-4, startY+10, startX-8, startY-25, startX-12, startY, ST7735_BLACK);

  tft.setFont();
  tft.setTextSize(1);
  tft.setCursor(10,startY+40);
  tft.println("RTIMage by L. Pawlowicz");
  tft.setCursor(10,startY+50);
  tft.println("adapted by S. Hageneuer");
  tft.setCursor(10,startY+80);
  tft.println("Ver. " + Software_version + ",  " + Software_date);
  delay(3000);
  tft.fillScreen(ST7735_WHITE);
}
//---------------------------------------------------------------------------------------------------------------------------------

void Build_Standby_Screen() {
  tft.fillScreen(COLOR1);
  tft.drawRect(0, 0, 160, 128, COLOR2);
  tft.drawLine(0, 30, 160, 30, COLOR2);
  tft.drawLine(0, 78, 160, 78, COLOR2);
  tft.setFont();
  tft.setTextColor(COLOR2);

  tft.setTextSize(2);
  tft.setCursor(40,8);
  tft.print("RTIMage");

  tft.setTextSize(1);
  tft.setCursor(10,40);
  tft.print("MODE:");
  if (digitalRead(Auto_Switch_Pin)==HIGH) {
    tft.setCursor(10,50);
    tft.print("SHUTTER:");
    if (digitalRead(Shutter_Switch_Pin)==HIGH) {
      tft.setCursor(10,60);
      tft.print("CAMERA:");
    }
  }
  tft.setTextSize(2);
  tft.setCursor(10,85);
  tft.print("LED:");
  tft.setCursor(10,105);
  tft.print("DEL:");
}
//---------------------------------------------------------------------------------------------------------------------------------

void Display_Standby_Screen() {  //Shows standby OLED screen

  Set_Data_Transfer_Time();
  Set_LED_Time();
  Set_Manual_Time();
  String Shutter_string=String((float) LED_Time/1000, 2);
  String Data_Transfer_string=String((float) Data_Transfer_Time/1000, 2);
  String LED_On_Time_string = String((float) LED_On_Time/1000,2);

  tft.setTextColor(COLOR2, COLOR1);

  tft.setTextSize(1);
  if (digitalRead(Auto_Switch_Pin)==LOW) {
    tft.setCursor(60,40);
    tft.print("Manual");
  } else {
    tft.setCursor(60,40);
    tft.print("Auto");
    if (digitalRead(Shutter_Switch_Pin)==HIGH) {
      tft.setCursor(60,50);
      tft.print("IR/Bluetooth");
      tft.setCursor(60,60);
      tft.println(Camera_type);
    } else {
      tft.setCursor(60,50);
      tft.print("USB");
    }
  }

  tft.setTextColor(LIME, COLOR1);
  tft.setTextSize(2);
  if (digitalRead(Auto_Switch_Pin)==LOW) {
    tft.setCursor(60,85);
    tft.print(millis()/1000);
    tft.setCursor(60,105);
    tft.print("N/A");
  } else {
    tft.setCursor(60,85);
    tft.print(Shutter_string + "s");
    tft.setCursor(60,105);
    tft.print(Data_Transfer_string + "s");}

    //delay(999);
    delay(100);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------
void Display_Auto_Screen() {  //Shows OLED screen in Automatic Mode

  String Current_count_string=String(Current_count);
  String Total_LEDs_string=String(Rows*Columns);
  Set_Data_Transfer_Time();
  Set_LED_Time();

  String Shutter_string=String((float) LED_Time/1000, 2);
  String Data_Transfer_string=String((float) Data_Transfer_Time/1000, 2);

  tft.fillScreen(COLOR1);
  tft.drawRect(0, 0, 160, 10, COLOR2);

  tft.setTextColor(COLOR2, COLOR1);
  tft.setTextSize(4);
  tft.setCursor(20,30);
  int StatusBalken;
  StatusBalken = 158 * Current_count / Rows * Columns;
  tft.fillRect(1, 1, StatusBalken, 8, LIME);
  tft.print(Current_count_string + String("/" + Total_LEDs_string));

  tft.setTextColor(COLOR2, COLOR1);
  tft.setTextSize(2);
  tft.setCursor(10,85);
  tft.print("LED:");
  tft.setCursor(10,105);
  tft.print("DEL:");

  tft.setCursor(60,85);
  tft.print(Shutter_string + "s");
  tft.setCursor(60,105);
  tft.print(Data_Transfer_string + "s");

  delay(999);
}

//---------------------------------------------------------------------------------------------------------------
void Display_Manual_Screen(int LED_Status) {  //Shows OLED screen in manual mode

  String Current_count_string=String(Current_count);
  String Total_LEDs_string=String(Rows*Columns);
  String LED_On_Time_string = String((float) LED_On_Time/1000,2);

  tft.fillScreen(COLOR1);
  tft.drawRect(0, 0, 160, 10, COLOR2);

  tft.setTextColor(COLOR2, COLOR1);
  tft.setTextSize(4);
  tft.setCursor(20,30);
  int StatusBalken;
  StatusBalken = 158 * Current_count / Rows * Columns;
  tft.fillRect(1, 1, StatusBalken, 8, LIME);
  tft.print(Current_count_string + String("/" + Total_LEDs_string));

  tft.setTextColor(COLOR2, COLOR1);
  tft.setTextSize(2);
  tft.setCursor(10,85);
  tft.print("LED:");
  tft.setCursor(10,105);
  tft.print("TIME:");

  tft.setCursor(60,85);
  if (LED_Status ==1) {
    tft.print("On");
  } else {
    tft.print("Off");
  }
  tft.setCursor(60,105);
  tft.print(LED_On_Time_string + "s");

  delay(999);
}

void printValues(int level, int aPin) {
  delay(5);
  int pot = 0;
  if (aPin == 15) {
    pot = 1;
  }
  Serial.print("level Pot");
  Serial.print(pot);
  Serial.print(": ");
  Serial.print(level);
  Serial.print(" Spannung an A");
  Serial.print(pot);
  Serial.print(": ");
  double sl = analogRead(aPin);
  sl = sl * 5 / 1024;
  //sl = sl * 5 / 512;
  Serial.print(sl);
  Serial.println(" Volt");
}


  void spi_read(byte read_command) {
  digitalWrite(slave_Select_Pin, LOW);
  SPI.transfer(read_command);
  miso_data = SPI.transfer(0);
  digitalWrite(slave_Select_Pin, HIGH);
  }

 void Mcp4241() {

  // Einlesen der Tasten
  int reading_Taste1_up = digitalRead(buttonPin_Taste1_up);
  int reading_Taste1_down = digitalRead(buttonPin_Taste1_down);
  int reading_Taste2_up = digitalRead(buttonPin_Taste2_up);
  int reading_Taste2_down = digitalRead(buttonPin_Taste2_down);
  

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // Switch Taste1_up auf Betätigung testen
  // If the switch changed, due to noise or pressing:
  if (reading_Taste1_up != lastButtonState_Taste1_up) {
    // reset the debouncing timer
    lastDebounceTime_Taste1_up = millis();
  }

  // Switch Taste1_down auf Betätigung testen
  if (reading_Taste1_down != lastButtonState_Taste1_down) {
    // reset the debouncing timer
    lastDebounceTime_Taste1_down = millis();
  }

  // Switch Taste2_up auf Betätigung testen
  if (reading_Taste2_up != lastButtonState_Taste2_up) {
    // reset the debouncing timer
    lastDebounceTime_Taste2_up = millis();
  }

  // Switch Taste2_down auf Betätigung testen
  if (reading_Taste2_down != lastButtonState_Taste2_down) {
    // reset the debouncing timer
    lastDebounceTime_Taste2_down = millis();
  }

  // Switch Taste1_up entprellen und bedienen
  // Folgende Funktion:

  if ((millis() - lastDebounceTime_Taste1_up) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading_Taste1_up != buttonState_Taste1_up) {
      buttonState_Taste1_up = reading_Taste1_up;
      

      if ((buttonState_Taste1_up == LOW)) {
        if (status_Taste1_up == 0) {
          Serial.println("Taste1_up");
          status_Taste1_up = 1;
                    
          digitalWrite(slave_Select_Pin, LOW);
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00000000); //Volativer Wiper 0 write (CMD: 0x00)
          SPI.transfer(value_on);
          digitalWrite(write_protection, LOW);
          digitalWrite(slave_Select_Pin, HIGH);
          printValues(value_on, analogInPin7);
          
        }
        else if (status_Taste1_up == 1)
        {
          value_on++;
          Serial.print(value_on);

          digitalWrite(slave_Select_Pin, LOW);
          SPI.transfer(B00000100); //Volatile Wiper 0 incrementieren (CMD: 0X00)
         
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00100000); //NV Wiper 0 write
          SPI.transfer(value_on);
          digitalWrite(write_protection, LOW);

          digitalWrite(slave_Select_Pin, HIGH);
          printValues(value_on, analogInPin7);

          }

        if (value_on == 127) { value_on = 0; }

        
      }
    }
  }
 

  // Switch Taste1_down entprellen und bedienen
  // Folgende Funktion:

  if ((millis() - lastDebounceTime_Taste1_down) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading_Taste1_down != buttonState_Taste1_down) {
      buttonState_Taste1_down = reading_Taste1_down;

      if ((buttonState_Taste1_down == LOW)) {
        if ((status_Taste1_down == 0) && (value_on < 127 )) {

          Serial.println("Taste1_down");
          status_Taste1_up = 0;
          status_Taste1_down = 1;
          value_on--;
                    
          digitalWrite(slave_Select_Pin, LOW);
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00000000); //Volativer Wiper 0 write (CMD: 0x00)
          SPI.transfer(value_on);
          digitalWrite(write_protection, LOW);
          digitalWrite(slave_Select_Pin, HIGH);
          printValues(level1, analogInPin7);
          
        }
     
        else if (status_Taste1_down == 1)
        {
          value_on--;

          digitalWrite(slave_Select_Pin, LOW);
          SPI.transfer(B00001000); //Volatile Wiper 0 decrementieren (CMD: 0X00)
         
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00100000); //NV Wiper 0 write
          SPI.transfer(value_on);
          digitalWrite(write_protection, LOW);

          digitalWrite(slave_Select_Pin, HIGH);
          printValues(value_on, analogInPin7);
        }
     
        if (value_on == 127) { value_on = 0; }  // Bedingungen bitte überprüfen
      }
    }
  
  

    // Switch Taste2_up entprellen und bedienen
    // Folgende Funktion:

    if ((millis() - lastDebounceTime_Taste2_up) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading_Taste2_up != buttonState_Taste2_up) {
        buttonState_Taste2_up = reading_Taste2_up;

        if ((buttonState_Taste2_up == LOW)) {
        if (status_Taste2_up == 0) {
          Serial.println("Taste2_up");
          status_Taste2_up = 1;

          digitalWrite(slave_Select_Pin, LOW);
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00010000); //Volativer Wiper 1 write (CMD: 0x00)
          SPI.transfer(value_off);
          digitalWrite(write_protection, LOW);
          digitalWrite(slave_Select_Pin, HIGH);
          printValues(value_off, analogInPin8);
          
        }
        else if (status_Taste2_up == 1)
        {
          value_off++;
          Serial.print(value_off);

          digitalWrite(slave_Select_Pin, LOW);
          SPI.transfer(B00010100); //Volatile Wiper 1 incrementieren (CMD: 0X00)

          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00110000); //NV Wiper 1 write
          SPI.transfer(value_off);
          digitalWrite(write_protection, LOW);

          digitalWrite(slave_Select_Pin, HIGH);
          printValues(value_off, analogInPin8);

        }
        
        if (value_off == 127) { value_off = 0; }

        
      }
     }
    }

    // Switch Taste2_down entprellen und bedienen
    // Folgende Funktion:

    if ((millis() - lastDebounceTime_Taste2_down) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading_Taste2_down != buttonState_Taste2_down) {
        buttonState_Taste2_down = reading_Taste2_down;
      
      if ((buttonState_Taste2_down == LOW)) {
       if ((status_Taste2_down == 0) && (value_on < 127 )) {

          Serial.println("Taste2_down");
          status_Taste2_up = 0;
          status_Taste2_down = 1;
          value_off--;
                    
          digitalWrite(slave_Select_Pin, LOW);
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00010000); //Volativer Wiper 1 write (CMD: 0x00)
          SPI.transfer(value_off);
          digitalWrite(write_protection, LOW);
          digitalWrite(slave_Select_Pin, HIGH);
          printValues(level1, analogInPin8);
          
        } 
        else if (status_Taste2_down == 1)
        {
          value_off--;

          digitalWrite(slave_Select_Pin, LOW);
          SPI.transfer(B00011000); //Volatile Wiper 1 decrementieren (CMD: 0X00)
         
          digitalWrite(write_protection, HIGH);
          SPI.transfer(B00110000); //NV Wiper 1 write
          SPI.transfer(value_off);
          digitalWrite(write_protection, LOW);

          digitalWrite(slave_Select_Pin, HIGH);
          printValues(value_off, analogInPin8);          
        }

        if (value_off == 127) { value_off = 0; } // Bedingungen bitte überprüfen
        
       
      }
     }
    }
  }
   
   
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState_Taste1_up = reading_Taste1_up;
  lastButtonState_Taste1_down = reading_Taste1_down;
  lastButtonState_Taste2_up = reading_Taste2_up;
  lastButtonState_Taste2_down = reading_Taste2_down;
  
}
