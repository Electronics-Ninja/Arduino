#include <Servo.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
 
Servo l_servo;  // Left Servo
Servo r_servo;  // Right Servo

Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
 
int pos = 0;    // variable to store the servo position 
int L = 0;
int R = 0;
  
int l_pos = 0;
int r_pos = 0;

int l_step = 90;
int r_step = 90;
 
void setup() 
{ 
  l_servo.attach(9);  // attaches the servo on pin 9 to the servo object 
  r_servo.attach(8);  
  matrix.begin(0x73);  // pass in the address
} 
 
void loop() 
{ 
  matrix.clear();
  l_servo.write(0);
  r_servo.write(0);
  L = 0;
  R = 0;
  
  while (L <= l_step) {
    l_servo.write(L);
    if (L % 10 == 0) {
      incrementMatrix(L/10);
    }
    L = L + 1;
    r_servo.write(R);
    R = R + 1;
    delay(15);
  }
  delay(300);
  
  while (L >= 0) {
    l_servo.write(L);
    if (L % 10 == 0) {
      incrementMatrix(L/10);
    }
    L = L - 1;
    r_servo.write(R);
    R = R - 1;
    delay(15);
  }
  delay(300);
}
  
void incrementMatrix(int L) {
  matrix.clear();
  matrix.setCursor(1,0);
  matrix.print(L);
  matrix.writeDisplay();
  delay(10);
}
//  for(pos = 0; pos <= 90; pos += 1) // goes from 0 degrees to 180 degrees 
//  {                                  // in steps of 1 degree 
//    l_servo.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
// } 
// for(pos = 0; pos <= 90; pos += 1) // goes from 0 degrees to 180 degrees 
//  {                                  // in steps of 1 degree 
//    r_servo.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  } 
//  for(pos = 90; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
//  {                                
//    l_servo.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  } 
//  for(pos = 90; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
//  {                                
//    r_servo.write(pos);              // tell servo to go to position in variable 'pos' 
//    delay(15);                       // waits 15ms for the servo to reach the position 
//  } 
