/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/SerialEvent
*/
const int analogInPin = A1; 
int val_filter, temp_data;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  // print the string when a newline arrives:
  temp_data = analogRead(analogInPin);
  if (stringComplete) {
    Serial.println(temp_data);
    //Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}



//#include "math.h"
//
//#define CALIB_AMOUNT 20
//// These constants won't change. They're used to give names to the pins used:
//const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
//
//int val[3];
//bool fl = true;
//int val_filter, temp_data;
//bool receiveData = false; 
//byte index, i = 0;
//String inputString = "";
//int k = 1000;
//int outputValue = 0;        // value output to the PWM (analog out)
//
//void setup() {
//  // initialize serial communications at 9600 bps:
//  Serial.begin(115200);
//  inputString.reserve(200);
//}
//
//void loop() {
//  // change the analog out value:
//  //temp_data = analogRead(analogInPin);
//  //if (++index > 2) index = 0; // change index from 0 to 2 (0, 1, 2, 0, 1, 2…)
//  //val[index] = temp_data; // записываем значение с датчика в массив
//  //val_filter = middle_of_3(val[0], val[1], val[2]);
//  //outputValue = val_filter;
//  
//  //Serial.println(outputValue);
//  //delay(10);
//}
//
//void serialEvent() {
//  while (Serial.available()) {
//    // get the new byte:
//      temp_data = analogRead(analogInPin);
//      
//      Serial.println(temp_data);
//  }
//
//}
//
//float middle_of_3(float a, float b, float c) {
//  int middle;
//  if ((a <= b) && (a <= c)) {
//    middle = (b <= c) ? b : c;
//  }
//  else {
//    if ((b <= a) && (b <= c)) {
//      middle = (a <= c) ? a : c;
//    }
//    else {
//      middle = (a <= b) ? a : b;
//    }
//  }
//  return middle;
//}
