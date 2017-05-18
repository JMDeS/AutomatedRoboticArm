#include <SoftwareSerial.h>
#include <Bridge.h>
#include <math.h>
#include "Arduino.h"
#include <Servo.h>
#include <Bridge.h>
#include <Temboo.h>
#include "TembooAccount.h"
 // contains Temboo account information, as described below

//Arm servo pins
#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define WRISTV 3
#define WRISTH 4
#define GRIP 5
#define SPEED 6

//#define DIGITAL_RANGE

#define offset 20



int numRuns = 1;   // Execution count, so this doesn't run forever
int maxRuns = 10;   

const float A = 3.75;
const float B = 4.25;

#define deg_to_pwm 2000.00/180.0
//Radians to Degrees constant
const float rtod = 57.295779;

SoftwareSerial serial(10,11);
int servos[7];

float Speed = 10.0;
int sps = 3;


//  void execute(){
//    Serial.print("BASE : "); Serial.println(servos[BASE]);serial.println(servos[BASE]);
//    Serial.print("SHOULDER : ");Serial.println(servos[SHOULDER]);serial.print(servos[SHOULDER]);
//    Serial.print("ELBOW : ");Serial.println(servos[ELBOW]);serial.print(servos[ELBOW]);
//    Serial.print("WRISTV : ");Serial.println(servos[WRISTV]);serial.print(servos[WRISTV]);
//    Serial.print("WRISTH : ");Serial.println(servos[WRISTH]);serial.print(servos[WRISTH]);
//    Serial.print("GRIP : ");Serial.println(servos[GRIP]);serial.print(servos[GRIP]);
//    Serial.println();
//    
//    for(int i=0;i<6;i++){
////     Serial.print("#"); Serial.print(i); Serial.print(" P");
////     Serial.print(servos[i]); Serial.print(" S"); Serial.println(servos[6]);
//      serial.print("#"); serial.print(i); serial.print(" P");
//      serial.print(servos[i]);
//      serial.print(" S");
//      serial.println(servos[6]);
////      delay(30);
//    }
//  }


//Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo Gripper;
Servo WristR;

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

char section;



int Arm(float x, float y, float z, int g, float wa, int wr) //Here's all the Inverse Kinematics to control the arm
{
  float M = sqrt((y*y)+(x*x));
  
  if(M <= 0)
    return 1;
  float A1 = atan(y/x);
  if(x <= 0)
    return 1;
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;
  if((int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  float Wris = abs(wa - Elbow - Shoulder) - 90;
#ifdef DIGITAL_RANGE
  Elb.writeMicroseconds(map(180 - Elbow, 0, 180, 900, 2100  ));
  Shldr.writeMicroseconds(map(Shoulder, 0, 180, 900, 2100));
#else
  servos[ELBOW] = ((180 - Elbow)*deg_to_pwm)*0.9;
  servos[SHOULDER] = Shoulder*deg_to_pwm*0.9;
#endif
  servos[WRISTH] = wr*deg_to_pwm;
  servos[WRISTV] = ((180 - Wris)*deg_to_pwm);
  servos[BASE] = z*deg_to_pwm;
#ifndef FSRG
  servos[GRIP] = g*deg_to_pwm;
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

         
#define actionBackUp 119        // w -> pulls up and back
#define actionForwardDown 115   // s -> moves forward and down
#define actionLower 97          // a -> moves down in place
#define actionRaise 100         // d -> moves up in place
#define actionRotCW 101         // e 
#define actionRotCCW 113        // q
#define actionGripperOpen 114   // r
#define actionGripperClose 116  // t
#define actionWristUp 122       // z
#define actionWristDown 120     // x
#define actionWristRotCW 103    // g
#define actionWristRotCCW 102   // f


const float posDeltaX = 0.25;
const float posDeltaY = 0.25;
const float posDeltaZ = 2.5;
const float posDeltaWa = 2.5;
const int posDeltaG = 30;
const int posDeltaWr = 2;
long lastReferenceTime;
unsigned char action;

void inputAction() {
     if(Serial.available() > 0)
    {
      // Read character
      action = Serial.read();
      if(action > 0)
      {
        // Set action
        switch(action)
        {
          case actionBackUp:
          if(tmpx != 0)
            tmpx -= posDeltaX;
          break;
          
          case actionForwardDown:
          tmpx += posDeltaX;
          break;

          case actionLower:
          tmpy -= posDeltaY;
          break;

          case actionRaise:
          tmpy += posDeltaY;
          break;

          case actionRotCW:
          tmpz += posDeltaZ;
          break;
          
          case actionRotCCW:
          if(tmpz != 0)
            tmpz -= posDeltaZ;
//          else
//            tmpz += posDeltaZ
          break;
          
          case actionGripperOpen:
          if(tmpg != 0)
            tmpg -= posDeltaG;
          break;
          
          case actionGripperClose:
          tmpg += posDeltaG;
          break;
          
          case actionWristUp:
          tmpwa += posDeltaWa;
          break;
          
          case actionWristDown:
//          if(tmpwa != 0)
            tmpwa -= posDeltaWa;
          break;
          
          case actionWristRotCW:
          tmpwr += posDeltaWr;
          break;
          
          case actionWristRotCCW:
          if(tmpwr != 0)
            tmpwr -= posDeltaWr;
          break;
        }
        
        // Display position
        Serial.print("tmpx = "); Serial.print(tmpx, DEC); Serial.print("\ttmpy = "); Serial.print(tmpy, DEC); Serial.print("\ttmpz = "); Serial.print(tmpz, DEC); Serial.print("\ttmpg = "); Serial.print(tmpg, DEC); Serial.print("\ttmpwa = "); Serial.print(tmpwa, DEC); Serial.print("\ttmpwr = "); Serial.println(tmpwr, DEC);
        
        // Move arm
        Arm(tmpx, tmpy, tmpz, tmpg, tmpwa, tmpwr);
        
        // Pause for 100 ms between actions
        lastReferenceTime = millis();
        while(millis() <= (lastReferenceTime + 30)){};
      }
    }
}


void calculate(int x, int y) {
  int xFactor = (int)x/75;
  int yFactor = (int)y/30;
  Serial.println();
  Serial.print("xfactor: ");Serial.println(xFactor);
  Serial.print("yfactor: ");Serial.println(yFactor);
 if ( yFactor == 0){
  if ( xFactor == 0 )
    chooseSection('Y');
  if ( xFactor == 1 )
    chooseSection('Z');
  if ( xFactor == 2 )
    chooseSection('a');
  if ( xFactor == 3 )
    chooseSection('b');
  if ( xFactor == 4 )
    chooseSection('c');
  if ( xFactor == 5 )
    chooseSection('d');
  if ( xFactor == 6 )
      chooseSection('e');
  if ( xFactor == 7 )
      chooseSection('f');
 }
  if ( yFactor == 1){
    if ( yFactor == 0 )
      chooseSection('Q');
    if ( xFactor == 1 )
      chooseSection('R');
    if ( xFactor == 2 )
      chooseSection('S');
    if ( xFactor == 3 )
      chooseSection('T');
    if ( xFactor == 4 )
      chooseSection('U');
    if ( xFactor == 5 )
      chooseSection('V');
    if ( xFactor == 6 )
      chooseSection('W');
    if ( xFactor == 7 )
      chooseSection('X');
  }
  if ( yFactor == 2 ) {
    if ( xFactor == 0 )
      chooseSection('I');
    if ( xFactor == 1 )
      chooseSection('J');
    if ( xFactor == 2 )
      chooseSection('K');
    if ( xFactor == 3 )
      chooseSection('L');
    if ( xFactor == 4 )
      chooseSection('M');
    if ( xFactor == 5 )
      chooseSection('N');
    if ( xFactor == 6 )
      chooseSection('O');
    if ( xFactor == 7 )
      chooseSection('P');
  }
  if ( yFactor == 3 ) {
    if ( xFactor == 0 )
      chooseSection('A');
    if ( xFactor == 1 )
      chooseSection('B');
    if ( xFactor == 2 )
      chooseSection('C');
    if ( xFactor == 3 )
      chooseSection('D');
    if ( xFactor == 4 )
      chooseSection('E');
    if ( xFactor == 5 )
      chooseSection('F');
    if ( xFactor == 6 )
      chooseSection('G');
    if ( xFactor == 7 )
      chooseSection('H');
  }
  
  delay(2000);
  Serial.println("---arm moved---");
  delay(2000);
  
}

boolean grabbed = false;

void moveToS(){
  execute(BASE,1500);delay(1500);execute(SHOULDER,1700);execute(ELBOW,1950);execute(3,950);delay(1500);execute(GRIP,800);
}
void moveToV(){
  execute(BASE,1100);delay(1000);execute(SHOULDER,1700);execute(ELBOW,1750);execute(3,800);delay(500);execute(GRIP,800);
  }
void moveToH(){
  execute(BASE,700);execute(SHOULDER,2000);execute(ELBOW,2000);execute(3,850);
}
void moveToY(){
  execute(BASE,1580);execute(SHOULDER,1175);execute(ELBOW,1050);execute(3,850);
}
void moveToHome(){
  execute(GRIP,800);
  delay(500);
  execute(2,1500);
  delay(500);
  execute(BASE,2000);
  delay(5000);
  execute(GRIP,2000);
}
void reach(int x, int y){
  if(x==0&&y==0);
  else if(60<x && x<150 && 30<y && y<200)
  {
    Serial.println();
    Serial.print("  x:");Serial.println(x);
    Serial.print("  y:");Serial.println(y);
    Serial.println("  moveToS");
    moveToS();
    delay(5000);
    moveToHome();
    delay(2000);
  }else if( x > 40 && x < 500 && y>30 &y<200){
    moveToV();
    delay(4000);
    moveToHome();
    delay(2000);
  }else if(false){ 
    moveToY();
  delay(2000);
    moveToHome();
    delay(2000);
    }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  serial.begin(9600);
   delay(4000);
  Bridge.begin();
 
  reset();
}

void reset(){
  servos[BASE] = 2500;
  servos[SHOULDER] = 1500;
  servos[ELBOW] = 1000; // 
  servos[WRISTV] = 1000; //R:500-1500
  servos[WRISTH] = 1000; // rotation (1000 is @ horizontal)
  servos[GRIP] = 1500; 
  servos[SPEED] = 200;
  execute(2,1300);delay(1000);
  execute(0,2000);
  delay(1000);
  execute(1,1300);delay(1000);
  
  execute(3,1500);delay(1000);
  execute(4,1500);delay(1000);
  execute(5,1500);


  delay(1000);
}

void execute(int servo, int pwm){
  servos[servo]=pwm;
  Serial.print(servo);Serial.print("  ");Serial.print(servos[servo]);Serial.print("  ");Serial.println(pwm); 
//  serial.println("#"+servo+" P"+pwm+" T500");
  serial.print("#"); serial.print(servo); 
  serial.print(" P");
      serial.print(servos[servo]);
      serial.println(" T4000");
}
void execute(int base, int shoulder,int elbow, int wrist){
  servos[BASE]=base;
  servos[SHOULDER]=shoulder;
  servos[ELBOW]=elbow;
  servos[3]=wrist;
  for(int servo = 0 ; servo < 4 ; servo++){
  Serial.print(servo);Serial.print("  ");Serial.print(servos[servo]);Serial.print("  ");Serial.println(servos[servo]); 
//  serial.println("#"+servo+" P"+servos[servo]+" T500");
  serial.print("#"); serial.print(servo); 
  serial.print(" P");
      serial.print(servos[servo]);
      serial.println(" T4000");
  }
  
}

 char joint[32] = {0};
 int servo = 0;
 int PWM = 0;
 
void loop()
{ 
    String receiptHandle;
    int awsx;
    int awsy;
  if (numRuns <= 400) {
    Serial.println("Running ReceiveMessage - Run #" + String(numRuns++));
    
    TembooChoreo ReceiveMessageChoreo;

    // Invoke the Temboo client
    ReceiveMessageChoreo.begin();

    // Set Temboo account credentials
    ReceiveMessageChoreo.setAccountName("gate1");
    ReceiveMessageChoreo.setAppKeyName("myFirstApp");
    ReceiveMessageChoreo.setAppKey("vRr6LbmtjiLzhmJmiDsURF0Qpce9ornk");
    
    // Set profile to use for execution
//    ReceiveMessageChoreo.setProfile("AmazonAWSAccount");
    
    // Set Choreo inputs
    ReceiveMessageChoreo.addInput("AWSAccountId", "391901324314");
    ReceiveMessageChoreo.addInput("AWSSecretKeyId", "Z6EQ8u1E74FDWCkw4sZuBk17uEgVyv0Iljzlc/vR");
    ReceiveMessageChoreo.addInput("AWSAccessKeyId", "AKIAJSKLRWGKXF4CA6HQ");
    ReceiveMessageChoreo.addInput("QueueName", "maintenanceArduino");
    
    // Identify the Choreo to run
    ReceiveMessageChoreo.setChoreo("/Library/Amazon/SQS/ReceiveMessage");
    
    // Run the Choreo; when results are available, print them to serial
    ReceiveMessageChoreo.run();

    String coordinates;
  
    int count = 0;
    while(ReceiveMessageChoreo.available()) {
      count++;
      String c = ReceiveMessageChoreo.readStringUntil('\n');
      Serial.print("c");Serial.println(c);
      if(count == 4){
        receiptHandle = c;
      }
      String xString;
      String yString;
      int comma;
        if(count == 6){
        Serial.print("count == 6");
              Serial.print(c);
              coordinates = c;
              int length = coordinates.length();
              
              comma = coordinates.indexOf(',');

              if(length == 8){
                xString = coordinates.substring(1,4); 
                yString = coordinates.substring(5,8);
              }else if(length == 7){
                if(comma == 4){
                 xString = coordinates.substring(1,4);
                 yString = coordinates.substring(5,7);
                }if(comma ==3){
                   xString = coordinates.substring(1,3);
                  yString = coordinates.substring(4,7);
                }
              }else{
                 xString = coordinates.substring(1,3);
                 yString = coordinates.substring(4,6);
              }

              awsx = xString.toInt();
              awsy = yString.toInt();

              Serial.print(awsx);
              Serial.print(awsy);
              break;
        }
    }

    ReceiveMessageChoreo.close();
    reach(awsx,awsy);

    Serial.println("x:");Serial.println(awsx);
    Serial.println("y:");Serial.println(awsy);
    
    delay(10000); // wait 30 seconds between ReceiveMessage calls
  
    Serial.println();
}
}
   

  

// 0 to 600 camera side to side
const byte numChars = 32;

 char receivedChars[numChars]; // an array to store the received data
int xin;
int yin;
void testServo() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;

 bool newData = false;
 // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
     rc = Serial.read();
    
     if (rc != endMarker) {
         receivedChars[ndx] = rc;
         ndx++;
         if (ndx >= numChars) {
         ndx = numChars - 1;
        }
      }
     else {
     receivedChars[ndx] = '\0'; // terminate the string
     ndx = 0;
     parseData();
     execute(servo, PWM);
//     reach(xin,yin);
     newData = true;
     }
  }
}
void parseData(){
  char * strtokIndx;
  strtokIndx = strtok(receivedChars,".");
  servo = atoi(strtokIndx);
  strtokIndx = strtok(NULL, "/n");
  PWM=atoi(strtokIndx);
}
//void parseData(){
//  char * strtokIndx;
//  strtokIndx = strtok(receivedChars,".");
//  yin = atoi(strtokIndx);
//  strtokIndx = strtok(NULL, "/n");
//  yin=atoi(strtokIndx);
//}


