#include "variant.h"
#include <due_can.h>
#include <Metro.h>
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
CAN_FRAME msg;

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8


int SOC; // from bms
int volt; // from bms
//int Batvoltraw;
int amps; //from isashunt
int MotorT;
int inverT;
int Batttemp; // from bms
int kW; // calculate from volt and amps
int mph;
int celldelta; // from bms
int rpm;
int AuxBattVolt;
int Batvoltraw;

Metro dashupdatetimer = Metro(500);
Metro gpstimer = Metro(100);

/// gps stuff
// Choose two Arduino pins to use for software serial
//int RXPin = 2;
//int TXPin = 3;

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
//SoftwareSerial gpsSerial(RXPin, TXPin);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(400);
  Serial1.begin(9600);  // nextion display
  delay(5000);
  Serial3.begin(9600);  // Start the software serial port at the GPS's default baud
  delay(400);

  Serial.println("Leyland Dash");

  Can0.begin(CAN_BPS_500K);

  int filter;
  //standard
  for (int filter = 3; filter < 7; filter++) {
  Can0.setRXFilter(filter, 0, 0, false);
  Can1.setRXFilter(filter, 0, 0, false);
  }  
}


void canSniff1() { //edit for Leaf canbus messages
  
  Can0.read(msg);

  if (msg.id == 0x55A) // from leaf inverter
  {
    inverT = msg.data.bytes[2];//INVERTER TEMP
   // Serial.println(inverT);
    MotorT = msg.data.bytes[1];//MOTOR TEMP

  }

  if (msg.id == 0x1DA)  // from leaf inverter
  {
    rpm = (( msg.data.bytes[4] << 8) | msg.data.bytes[5]);
    //Serial.println(rpm);
  }


  if (msg.id == 0x355)
  {
    SOC = (( msg.data.bytes[1] << 8) | msg.data.bytes[0]);
    //Serial.println("SIMPBMS");
  }
  
  if (msg.id == 0x1DA)//battery voltage from Leaf intverter
    volt = ((msg.data.bytes[0] << 2) | (msg.data.bytes[1] >> 6));//MEASURED VOLTAGE FROM LEAF INVERTER
  
  /*(
  if (msg.id == 0x356)//battery voltage from SIMP BMS
  {

    Batvoltraw = (( msg.data.bytes[1] << 8) | msg.data.bytes[0]);
    volt = Batvoltraw / 10;
    //Serial.println("SIMPBMS2");
  }
  */

}
void dashupdate()
{
  if (dashupdatetimer.check()) {
    Serial1.write(0x22);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("soc.val=");
    Serial1.print(SOC);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("soc1.val=");
    Serial1.print(SOC);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("amps.val=");
    Serial1.print(amps);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("motorT.val=");
    Serial1.print(MotorT);// Get motor temps
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("BattT.val=");
    Serial1.print(Batttemp); //get battery temp
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("InverT.val=");
    Serial1.print(inverT);// get inverter temp
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("volt.val=");
    Serial1.print(volt); //get battery pack voltage
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("kW.val=");
    Serial1.print(kW); //get kW value
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("mph.val=");
    Serial1.print(mph);//get mph
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    //Serial1.print("celldelta.val = ");
    // Serial1.print(); // to do
    //  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    // Serial1.write(0xff);
    // Serial1.write(0xff);
    Serial1.print("rpm.val=");
    Serial1.print(rpm);
    // Serial1.print("\xFF\xFF\xFF");
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
}

void gpsupdate()// This sketch displays information every time a new sentence is correctly encoded.
{
  while (Serial3.available() > 0)

    if (gps.encode(Serial3.read()))
    {
      mph = gps.speed.mph();
      Serial.println(mph);
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    //while (true);
  }

}
void loop() {
  // put your main code here, to run repeatedly:
  //Can0.events();

 if (Can0.available() > 0) {
    canSniff1();

  }
  

 // if (gpstimer.check())
//  {
 //   gpsupdate();
//  }
  // sending data to nextion display
  dashupdate();



}
