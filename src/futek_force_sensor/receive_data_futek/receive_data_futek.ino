#include "math.h"

#define CALIB_AMOUNT 20
// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int val[3];
bool fl=true;
int val_filter, temp_data;
byte index, i=0;
int outputValue = 0;        // value output to the PWM (analog out)
const float error = 5.0;
float avg_calib = 0;
int correction_coeff = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // change the analog out value:
  temp_data=analogRead(analogInPin);
  
  if (fl==true)
  {
//          Serial.print("avg_calib = ");
//  Serial.println(avg_calib);
    avg_calib +=temp_data;
    i++;
//    Serial.println(i);
    if (i==CALIB_AMOUNT && abs(temp_data-avg_calib/i)<error)
      {
        correction_coeff = temp_data;
        fl = false;
      }
    if (i==CALIB_AMOUNT && abs(temp_data-avg_calib/i)>=error)
      {
        avg_calib = 0;
        i = 0;
      }
  }
  if (++index > 2) index = 0; // переключаем индекс с 0 до 2 (0, 1, 2, 0, 1, 2…)
  val[index] = temp_data; // записываем значение с датчика в массив
  // фильтровать медианным фильтром из 3ёх ПОСЛЕДНИХ измерений
  //val_filter = middle_of_3(val[0], val[1], val[2]) - correction_coeff;
  val_filter = middle_of_3(val[0], val[1], val[2]);
  outputValue = map(val_filter, 0, 1023, 0, 1023);
  
  // print the results to the Serial Monitor:
  Serial.println(outputValue);

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
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
