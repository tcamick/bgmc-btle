

#include <Servo.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/


#ifdef __AVR__
#endif
  #include <avr/power.h>

#define PIN 6


// Min and max values for input mapping to pwm signal
int min_map = 0;
int min_ms = 1000;
int max_map = 1000;

// Min and max values for pwm signa;
int max_ms = 2000;

// PWM signal for stopped motor
int STOPPED = 1500;
int bgmc_signal = STOPPED;

// Starting program with motor in stopped position
int value = max_map/2;

int incoming_byte = 0; //incoming serial data

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int pot=0;

Servo bgmc;



void setup() {

  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  BTLEserial.setDeviceName("camick"); /* 7 characters max! */

  BTLEserial.begin();
  
  bgmc.attach(3); // attached to pin 9 I just do this with 1 Servo
 
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {

  bgmc.writeMicroseconds(bgmc_signal);
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    
    if(BTLEserial.available()) {
      char c = BTLEserial.read();
      value=bgmc_parse_input(c);
      bgmc_signal =  bgmc_map_signal(value);
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }

}

char bgmc_display_values(int bgmc_signal){
    if(bgmc_signal < STOPPED){
        Serial.println("Rotation: Counter Clockwise");
    }
    else if(bgmc_signal > STOPPED){
        Serial.println("Rotation: Clockwise");
    }
    else{
        Serial.println("Rotation: Stopped");
    }
    Serial.println("PWM Signal: " + String(bgmc_signal));
}
    
int bgmc_parse_input(char incoming_byte){
  if( incoming_byte == '1'){
     value-=10;
     if(value < min_map){
       value = min_map;
     }
   }
  if( incoming_byte == '2'){
    value+=10;
    if(value > max_map){
      value = max_map;
    }
  }
       
  return value;
}

int bgmc_map_signal(int value){
  return bgmc_signal = map(value, min_map, max_map, min_ms, max_ms);
}

