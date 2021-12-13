#include <mcp_can.h>
#include <SPI.h>
#include <RMDx8Arduino.h>   
//#include <SoftwareSerial.h> 




/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 9600
long unsigned int MOTOR_ADDRESS = 0x143; //0x140 + ID(1~32)
long unsigned int MOTOR_ADDRESS2 = 0x144;
int SPI_CS_PIN = 5;
int analogInPin = 35; 
int pos1, angpos1, torque1;
int pos2, angpos2, torque2;
int posref = 0, posref2 = 0, dif = 0,dif2=0;
int sensorValue = 0, potpos=0;
int torqueminm1=0, torquemaxm1=0, torqueminm2=0, torquemaxm2=0;
const int finaleffector1=36;
int finaleffector;
long unsigned int rxId;
//SoftwareSerial MyBlue(2, 3); // RX | TX 

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN
RMDx8Arduino rmd1(CAN, MOTOR_ADDRESS);
RMDx8Arduino rmd2(CAN, MOTOR_ADDRESS2);
void setup() {
  SERIAL.begin(BAUDRATE);
//  MyBlue.begin(9600);
  delay(1000);
  rmd1.canSetup();
  rmd2.canSetup();
  rmd1.writePID(40,20,15,8,15,10);
  rmd2.writePID(40,20,15,8,15,10);
  rmd1.readPID();
  rmd2.readPID();
  rmd1.writeCurrent(0);
  rmd2.writeCurrent(0);
  SERIAL.print("posKp ");
  SERIAL.print(rmd1.posKp); //100
  SERIAL.print("posKi");
  SERIAL.print(rmd1.posKi); //100
  delay(3000);
}

void loop() {
  sensorValue = analogRead(analogInPin);
  finaleffector=analogRead(finaleffector1);
  float value1=map(finaleffector,630,700,10,0);
  float value2=value1/10;
  //Motor 1 control
  rmd1.readPosition();
  pos1 = rmd1.present_position;
  int error1 = pos1 - posref;

  //Limit position
  if (pos1 >= 15300) {
    pos1 = 15300;
  }
  if (pos1 <= 8600) {
    pos1 = 8600;
  }
  
  //Mapping and degree convert
  angpos1 = map(pos1, 15300, 8600, 0, 60);
  posref = pos1;
  rmd1.readPosition();
  //Direction
  if (error1>30 || error1 < -30){
    dif=0;
  }
  else {
    dif=1;
  }
  torqueminm1=map(sensorValue,0,1023,-20,-30);
  torquemaxm1=map(sensorValue,0,1023,-40,-65);
    if (dif==1){
      torque1 = map(angpos1, 0, 50, torqueminm1, torquemaxm1);
    }
    else {
      torque1=map(angpos1, 0, 50, -30, -15);
    }
    rmd1.writeCurrent(torque1);
  
  //Motor 2 control
  rmd2.readPosition();
  pos2 = rmd2.present_position;
  int error2 = pos2 - posref2;
  if (pos2 <= 8050) {
    pos2 = 8050;
  }
  if (pos2 >= 10400) {
    pos2 = 10400;
  }
  angpos2 = map(pos2, 8050, 10400, 0, 30);

   //Direction

  if (error2>20 || error2 < -20){
    dif2=0;
  }
  else {
    dif2=1;
  }
  torqueminm2=map(sensorValue,0,1023,10,30);
  torquemaxm2=map(sensorValue,0,1023,20,35);
  
    if (dif2==1){
      torque2 = map(angpos2, 0, 20, torqueminm2, torquemaxm2);
      if (angpos1>=35){
        torque2=35;
      }
    }
    else {
      torque2=map(angpos2, 0, 20,20,10);
    }
    rmd2.writeCurrent(torque2);

  posref2 = pos2;
  rmd2.readPosition();
//  MyBlue.println(value2);
 SERIAL.println(value2);
//  SERIAL.print(sensorValue);
//  SERIAL.print("    ");
//  SERIAL.print("temp1:    ");
//  SERIAL.print(rmd1.temperature);
//  SERIAL.print("    ");
//  SERIAL.print("torque1:    ");
//  SERIAL.print(torque1);
//  SERIAL.print("    ");
//  SERIAL.print("pos1:    ");
//  SERIAL.print(angpos1);
//  SERIAL.print("    ");
//  SERIAL.print("mov1:    ");
//  SERIAL.print(dif);
//  SERIAL.print("    ");
//  SERIAL.print("temp2:    ");  
//  SERIAL.print(rmd2.temperature);
//  SERIAL.print("    ");
//  SERIAL.print("torqu2:    ");
//  SERIAL.print(torque2);  
//  SERIAL.print("    ");
//  SERIAL.print("mov2:    ");
//  SERIAL.println(dif2);
}
