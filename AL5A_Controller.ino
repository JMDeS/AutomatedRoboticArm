#include <SoftwareSerial.h>
#include <math.h>
#include "Arduino.h"
#include <Servo.h>


//Arm servo pins
#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define WRISTV 3
#define WRISTH 4
#define GRIP 5
#define SPEED 6

//Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo Gripper;
Servo WristR;

//#define DIGITAL_RANGE

#define offset 20


const float A = 3.75;
const float B = 4.25;

#define deg_to_pwm 2000.00/180.0
//Radians to Degrees constant
const float rtod = 57.295779;

SoftwareSerial serial(10,11);
int servos[7];

float Speed = 10.0;
int sps = 3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  serial.begin(9600);
}


//Arm Current Pos
float X = 4;
float Y = 4;
int Z = 90;
int G = 90;
float WA = 0;
int WR = 90;

//Arm temp pos
float tmpx =4;
float tmpy = 4;
int tmpz = 90;
int tmpg = 90;
int tmpwr = 80;
float tmpwa = 0;


int Arm(float x, float y, float z, int g, float wa, int wr) //Here's all the Inverse Kinematics to control the arm
{
  x/=rtod;
  y/=rtod;
  
  Serial.println();
  Serial.println("Arm called");
  float M = sqrt((y*y)+(x*x));

  if(M <= 0)
    return 1;
  float A1 = atan2(y,x)*rtod;
  Serial.print("A1: ");Serial.println(A1);

  if(x <= 0)
    return 2;
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M))*rtod;
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B))*rtod;
  float Shoulder = A1 + A2;
  Serial.print("Shoulder: ");
  Serial.println((int)Shoulder);
  Serial.print("Elbow: ");
  Serial.println((int)Elbow);
  if((int)Elbow <= 0 || (int)Shoulder <= 0)
    return 3;
  float Wris = abs(wa - Elbow - Shoulder) - 90;

  execute( ELBOW ,    90 );
  execute( SHOULDER , Shoulder    );
  execute(  WRISTH ,   wr         );
  execute(  WRISTV ,   Wris ); //180 - Wris 
  execute(  BASE   ,   A1          );
#ifndef FSRG
  execute( GRIP , g );
#endif
  Y = tmpy;
  X = tmpx;
  Z = tmpz;
  WA = tmpwa;
#ifndef FSRG
  G = tmpg;
#endif

  return 0; 
}

void Arm(float x, float y, float z)
{
  Arm(x,y,z,90,90,90);
}


void execute(int servo, int deg){
  Serial.print("executing: servo:");Serial.print(servo);Serial.print(" deg: ");Serial.println(deg);
  int pwm = 500 + deg*deg_to_pwm;
//  serial.println("#"+servo+" P"+pwm+" T500");
  serial.print("#"); serial.print(servo); 
  serial.print(" P");
      serial.print(pwm);
      serial.println(" T2000");
      delay(30);
}
void loop() 
{
//  serial.println("#0 P1000 T1000");
  Arm(90,90,90);
  Serial.println(90,90,90);
  delay(500);
  Arm(0,90,90);
  Serial.println(0,90,90);
  delay(500);
  Arm(90,0,90);
  Serial.println(90,0,90);
  delay(500);
  Arm(90,90,45);
  Serial.println(90,90,45);
  delay(500);
  Arm(90,90,0);
  Serial.println(90,90,0);
  delay(1000);
  Arm(90,90,135);
  Serial.println(90,90,135);
  delay(1000);
//  Arm(160,90,45);
//  delay(1000);
 
}

