#include "mbed.h"
//b0 is the rightmost and b4 is the leftmost
// Sensors
AnalogIn S0(A0);
AnalogIn S1(A1);
AnalogIn S2(A2);
AnalogIn S3(A3);
AnalogIn S4(A4);
// Motors
PwmOut PWM1(D11); // Left motor speed
PwmOut PWM2(D9); // Right motor speed
#define THRESHOLD 0.65f
int isBlack(float v) {
   return (v < THRESHOLD);
}
 
DigitalOut DIR1(D12);
DigitalOut DIR2(PA_11_ALT0);
 
int main() {
   PWM1.period(0.01f);
   PWM2.period(0.01f);
 
   DIR1 = 0; // Left motor forward
   DIR2 = 0; // Right motor forward
   while (true) {
       float a0 = S0.read();
       float a1 = S1.read();
       float a2 = S2.read();
       float a3 = S3.read();
       float a4 = S4.read();
       int b0 = isBlack(a0);
       int b1 = isBlack(a1);
       int b2 = isBlack(a2);
       int b3 = isBlack(a3);
       int b4 = isBlack(a4);
       float leftSpeed = 0.0f;
       float rightSpeed = 0.0f; 
       // Case 1:straight
       if ( b0||b4||b2){
           leftSpeed = 0.15f;
           rightSpeed = 0.15f;
       }
       // Case 2: right
       else if (b1){
           leftSpeed = 0.25f;
           rightSpeed = 0.05f;
       }
       //case 3 : left
       else if (b3) {
           leftSpeed = 0.05f;
           rightSpeed = 0.25f;
       }
       else {
           // stop
           leftSpeed = 0.0f;
           rightSpeed = 0.0f;
       }
       PWM1.write(leftSpeed);
       PWM2.write(rightSpeed);
       printf("b0:%d b1:%d b2:%d b3:%d b4:%d\n", b0, b1, b2, b3, b4);
       ThisThread::sleep_for(10ms);
   }
}