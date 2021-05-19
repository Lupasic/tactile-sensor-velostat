#include "math.h"

#define CALIB_AMOUNT 20
// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to

int val[3];
bool fl = true;
int val_filter, temp_data;
bool receiveData = false; 
byte index, i = 0;
String inputString = "";
int k = 1000;
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  inputString.reserve(200);
}

void loop() {
  // change the analog out value:
  temp_data = analogRead(analogInPin);
  //if (++index > 2) index = 0; // change index from 0 to 2 (0, 1, 2, 0, 1, 2…)
  //val[index] = temp_data; // записываем значение с датчика в массив
  //val_filter = middle_of_3(val[0], val[1], val[2]);
  outputValue = temp_data;
  
  Serial.println(outputValue);
  delay(2);
}

float middle_of_3(float a, float b, float c) {
  int middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}
