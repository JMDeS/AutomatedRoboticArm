#include <math.h>
#include "Arduino.h"
#include <SoftwareSerial.h>
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

#define deg_to_pwm 2000.00/180.0
SoftwareSerial serial(10,11);
int servos[7];

float xin,yin,zin;

const float a1 = 65;// height of base (m)
const float a2 = 94;// distance b/w shoulder and elbow joints (cm)
const float a3 = 184;// distance b/w elbow and wrist joints (cm)
const float a4 = 111;// distance b/w wrist joint and tip of grip (cm)

//Radians to Degrees constant
const float rtod = 57.295779;


//Joint angles
float t1;//=0/rtod; //base angle -90 to 90 deg
float t2;//=45/rtod; //shoulder angle 0 to 180 deg
float t3;//=45/rtod; //elbow angle 0 to 180 deg
float t4;//=45/rtod; //wrist angle 0 to 180 deg
float t234;//=(t2+t3+t4); // t1 + t2 + t3
float t5;//=90/rtod;
float t6 =90/rtod;//=0;
float c3, s3;

float d1,d2,d3,d4,d5,d6,d234;




/*  Hand Frame
 *  [ nx ox ax px ]
 *  [ ny oy ay py ]
 *  [ nz oz az pz ]
 *  [ 0  0  0  1  ]
*/
float px,py,pz;
float nx,ny,nz;
float ax,ay,az;
float ox,oy,oz;

void gripOpen(){
  servos[GRIP]=1250;
  Serial.print("GRIP @ ");Serial.print("OPEN");Serial.print("  PWM:");Serial.println(servos[GRIP]);
}
void gripClose(){
  servos[GRIP]=2000;
  Serial.print("GRIP @ ");Serial.print("CLOSED");Serial.print("  PWM:");Serial.println(servos[GRIP]);
}
void reach(float x,float y,float z)
{
  px=x;py=y;pz=z;
  calculateAngles();
  execute();
}

float dx=100,dy=100,dz=100;
float X(){ return dx;}
float Y(){ return dy;}
float Z(){ return dz;}

//float PX(){ return cos(t1)*(cos(t234)*a4+cos(t2+t3)*a3+cos(t2)*a2); }
//float PY(){ return sin(t1)*(cos(t234)*a4+cos(t2+t3)*a3+cos(t2)*a2); }
//float PZ(){ return sin(t234)*a4+sin(t2+t3)*a3+sin(t2)*a2; }
float PX(){return px;}
float PY() {return py;}
float PZ(){return pz;}

float NX(){ return cos(t1)*(cos(t234)*cos(t5)*cos(t6) - sin(t234)*sin(t6))-sin(t1)*sin(t5)*sin(t6); }
float NY(){ return sin(t1)*(cos(t234)*cos(t5)*cos(t6)-sin(t234)*sin(t6))+cos(t1)*sin(t5)*cos(t6); }
float NZ(){ return sin(t234)*cos(t5)*cos(t6)+cos(t234)*sin(t6); }

float OX(){ return cos(t1)*(-cos(t234)*cos(t5)*cos(t6)-sin(t234)*cos(t6)) + sin(t1)*sin(t5)*sin(t6); }
float OY(){ return sin(t1)*(-cos(t234)*cos(t5)*cos(t6)-sin(t234)*cos(t6)) - cos(t1)*sin(t5)*sin(t6); }
float OZ(){ return -sin(t234)*cos(t5)*cos(t6) + cos(t234)*cos(t6); }

float AX(){ return cos(t1)*(cos(t234)*sin(t5)) + sin(t1)*cos(t5); }
float AY(){ return sin(t1)*(cos(t234)*sin(t5)) - cos(t1)*cos(t5); }
float AZ(){ return sin(t234)*sin(t5); }

float T1(){ return atan2(PY(),PX()); }
float T234(){ return atan2( AZ() , cos(t1)*AX()+sin(t1)*AY() ); }
float C3(){ return ( pow( PX()*cos(t1)+PY()*sin(t1)-cos(t234)*a4 , 2 ) + pow( PZ()-sin(t234)*a4 , 2 ) - a2*a2 - a3*a3 ) / (2*a2*a3); }
float S3(){ return sqrt(1-C3()*C3()); }
float T3(){ return atan2(S3(),C3()); }
float T2(){ return atan2( (cos(t3)*a3+a2)*(PZ()-sin(t234)*a4) - (sin(t3)*a3*(PX()*cos(t1)+PY()*sin(t1)-cos(t234)*a4) ) , ( (cos(t3)*a3+a2)*(PX()*cos(t1)+PY()*sin(t1)-cos(t234)*a4)+ sin(t3)*a3*(PZ()-sin(t234)*a4) ) ); }
float T4(){ return t234-t2-t3; }
float T5(){ return atan2( (cos(t234)*(cos(t1)*AX()+sin(t1)*AY())+sin(t234)*AZ()) , (sin(t1)*AX()-cos(t1)*AY()) ); }
float T6(){ return atan2( (-sin(t234)*(cos(t1)*NX()+sin(t1)*NY())+cos(234)*NZ()) , (-sin(t234)*(cos(t1)*OX()+sin(t1)*OY())+cos(234)*OZ()) ); }


void updateReferenceFrame()
{ //called whenever an angle is changed
//  Serial.println("referenceFrameUpdated");
//  px=PX();py=PY();pz=PZ();
  ax=AX(); ay=AY(); az=AZ();
  ox=OX(); oy=OY();oz=OZ();
  nx=NX(); ny=NY(); nz=NZ();
  
//  Serial.print(nx);Serial.print(" ");Serial.print(ny);Serial.print(" ");Serial.println(nz);
//  Serial.print(ox);Serial.print(" ");Serial.print(oy);Serial.print(" ");Serial.println(oz);
//  Serial.print(ax);Serial.print(" ");Serial.print(ay);Serial.print(" ");Serial.println(az); 
}

void adjustAngle(float &a){
  while ( a > 180 )
    a-=180;
  while( a < 0 )
    a+=180;
  return;
}
float offset = 500;
void calculateAngles()
{    
  Serial.print("x:");Serial.print(px);Serial.print(" y:");Serial.print(py);Serial.print(" z:");Serial.println(pz);
    t1 = atan2(py,px);
      servos[BASE]=t1*rtod*deg_to_pwm+offset;Serial.print("BASE @ ");Serial.print(t1*rtod);Serial.print(" PWM:");Serial.println(servos[BASE]);
        t234 = T234(); //Serial.print("T234=");Serial.println(t234*rtod);
        c3 = C3(); //Serial.print("c3=");Serial.println(c3);
        s3=S3(); //Serial.print("s3=");Serial.println(s3);
    t3=T3();
      servos[ELBOW]=t3*rtod*deg_to_pwm+offset;
      Serial.print("ELBOW @ ");Serial.print(t3*rtod);Serial.print(" PWM:");Serial.println(servos[ELBOW]);
    t2=T2();
      servos[SHOULDER]=(t2*rtod+180)*deg_to_pwm+offset;
      Serial.print("SHOULDER @ ");Serial.print(t2*rtod+180);Serial.print(" PWM:");Serial.println(servos[SHOULDER]);
    t4=T4();
      servos[WRISTV]=t4*rtod*deg_to_pwm+offset;
      Serial.print("WRIST @ ");Serial.print(t4*rtod);Serial.print(" PWM:");Serial.println(servos[WRISTV]);
    t5=T5();
      servos[WRISTH]=t5*rtod*deg_to_pwm+offset;
      Serial.print("WRISTH @ ");Serial.print(t5*rtod);Serial.print(" PWM:");Serial.println(servos[WRISTH]);
    
 
    Serial.println();
  
}

 void reset(){
  
  d1=90/rtod; //base angle -90 to 90 deg
  d2=45/rtod; //shoulder angle 0 to 180 deg
  d3=45/rtod; //elbow angle 0 to 180 deg
  d4=45/rtod; //wrist angle 0 to 180 deg
  d234=(t2+t3+t4); // t1 + t2 + t3
  d5=90/rtod;
  d6=90;
  gripOpen();

  for(int i=0 ; i< sizeof(servos)/2 ; i++)
    servos[i] = 1500;
  execute();
  updateReferenceFrame();
}
void execute(){
  for(int i=0 ; i <sizeof(servos)/2; i++)
  {
    serial.print("#");
    serial.print(i);
    serial.print(" P");
    serial.print(servos[i]);
    serial.println(" T1000");
    delay(30);
  }
}

void setup()
{
//  Serial.begin(115200);
Serial.begin(9600);
serial.begin(9600);
  Serial.println("setup");
  delay(1000);
  reset();
  calculateAngles();
  Serial.println();
  Serial.println("Enter xyz:");
  
}
const byte numChars = 32;
 char receivedChars[numChars]; // an array to store the received data
void getCoords() {
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
     newData = true;
     }
  }
}
void parseData(){
  char * strtokIndx;
  strtokIndx = strtok(receivedChars,",");
  xin =atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  yin=atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  zin=atof(strtokIndx);
}


void loop()
{
//  calculateAngles(50,50,50);
  
   if (Serial.available() > 0) {
    getCoords();
    reach(xin,yin,zin); 
  }

}
