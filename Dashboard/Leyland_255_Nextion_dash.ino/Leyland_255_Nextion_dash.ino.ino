#include "variant.h"
#include <due_can.h>
#include <Metro.h>
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
CAN_FRAME msg;
String message;

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN 8


int SOC;          // from bms
int32_t volt;     // from isashunt
int32_t rawvolt;  // from isashunt
int32_t mamps;    //from isashunt
int amps;
int MotorT;
int inverT;
int Batttemp;  // from bms
int32_t kW;    // calculate from volt and amps
int mph;
int celldelta;  // from bms
int rpm;
int AuxBattVolt;
int Batvoltraw;   // from bms
uint32_t Batmax;  // from bms
uint32_t Batmin;  // from bms



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


void canSniff1() {  //edit for Leaf canbus messages

  Can0.read(msg);

  if (msg.id == 0x55A)  // from leaf inverter
  {
    inverT = (msg.data.bytes[2] - 32) / 1.8;  //INVERTER TEMP
    // Serial.println(inverT);
    MotorT = (msg.data.bytes[1] - 32) / 1.8;  //MOTOR TEMP
  }

  if (msg.id == 0x1DA)  // from leaf inverter
  {
    //rawvolt = ((int16_t)msg.data.bytes[0] << 2) | (msg.data.bytes[1] >> 6);//((uint16_t)((msg.data.bytes[0] << 2) | msg.data.bytes[1] >> 6));//MEASURED VOLTAGE FROM LEAF INVERTER
    //volt = rawvolt / 2;
    rpm = ((int16_t)((msg.data.bytes[4] << 8) | msg.data.bytes[5]));
    rpm = rpm / 2;
    mph = rpm * 0.008;
    
  }


  if (msg.id == 0x355) {
    SOC = ((msg.data.bytes[1] << 8) | msg.data.bytes[0]);
    //Serial.println("SIMPBMS");
  }


  if (msg.id == 0x522)  //battery voltage from isa shunt
  {

    int8_t* bytes = (int8_t*)msg.data.bytes;  // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    rawvolt = ((msg.data.bytes[5] << 24) | (msg.data.bytes[4] << 16) | (msg.data.bytes[3] << 8) | (msg.data.bytes[2]));
    volt = rawvolt / 1000;
    Batmax = (volt * 1000) / 96;
    Batmin = (volt * 1000) / 96;
    celldelta = Batmax - Batmin;
  }

  if (msg.id == 0x373)  // highest cell voltage from SIMP BMS
  {

    //Batmax = (( msg.data.bytes[3] << 8) | msg.data.bytes[2]);
    //Batmin = (( msg.data.bytes[1] << 8) | msg.data.bytes[0]);
  }

  if (msg.id == 0x521)  // amps from isa shunt
  {
    //uint8_t* bytes = (uint8_t*)msg.data.bytes;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    // mamps = ((msg.data.bytes[2] << 24) | (msg.data.bytes[3] << 16) | (msg.data.bytes[4] << 8) | (msg.data.bytes[5]));
    int8_t* bytes = (int8_t*)msg.data.bytes;  // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    mamps = ((msg.data.bytes[5] << 24) | (msg.data.bytes[4] << 16) | (msg.data.bytes[3] << 8) | (msg.data.bytes[2]));
    mamps = mamps * -1;
    amps = (float)mamps / 100;
    kW = (volt * amps) / 1000;
  }

  if (msg.id == 0x6F6)  // amps from zombie
  {
    //uint8_t* bytes = (uint8_t*)msg.data.bytes;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    // amps = msg.data.bytes[0];
  }


  if (msg.id == 0x3C5)  // figure out can ID from Zombie for Aux Batt volt
  {

    AuxBattVolt = msg.data.bytes[5];  // figure this out
  }
}
void dashupdate() {
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
    Serial1.print(MotorT);  // Get motor temps
    Serial1.write(0xff);    // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("BattT.val=");
    Serial1.print(Batttemp);  //get battery temp
    Serial1.write(0xff);      // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("InverT.val=");
    Serial1.print(inverT);  // get inverter temp
    Serial1.write(0xff);    // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("volt.val=");
    Serial1.print(volt);  //get battery pack voltage
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("kW.val=");
    Serial1.print(kW);    //get kW value
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("mph.val=");
    Serial1.print(mph);   //get mph
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("delta.val=");
    Serial1.print(celldelta);  //get max cell voltage
    Serial1.write(0xff);       // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("rpm.val=");
    Serial1.print(rpm);
    // Serial1.print("\xFF\xFF\xFF");
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("highest.val=");
    Serial1.print(Batmax);  //get max cell voltage
    Serial1.write(0xff);    // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("lowest.val=");
    Serial1.print(Batmin);  //get max cell voltage
    Serial1.write(0xff);    // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("volt12v.val=");
    Serial1.print(AuxBattVolt);  //get max cell voltage
    Serial1.write(0xff);         // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
}

void gpsupdate()  // This sketch displays information every time a new sentence is correctly encoded.
{
  while (Serial3.available() > 0)

    if (gps.encode(Serial3.read())) {
      mph = gps.speed.mph();
      Serial.println(mph);
    }
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
    //while (true);
  }
}

void dashreturn()  //
{
  kW = volt * amps;
  Batmax = (volt / 96) * 1000;
  Batmin = (volt / 96) * 1000;
  celldelta = Batmax - Batmin;
}

void buttonread() {
  message = Serial1.read();

  if (message[1] == 65) {
    CAN_FRAME msg1;
    msg1.id = 0x111;
    msg1.length = 8;
    msg1.data.bytes[0] = 1;
    msg1.data.bytes[1] = 0;
    msg1.data.bytes[2] = 0;
    msg1.data.bytes[3] = 0;
    msg1.data.bytes[4] = 0;
    msg1.data.bytes[5] = 0;
    msg1.data.bytes[6] = 0x00;
    msg1.data.bytes[7] = 0;
    Can0.sendFrame(msg1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //Can0.events();

  if (Can0.available() > 0) {
    canSniff1();
  }

  if (Serial1.available()) {  //Is data coming through the serial from the Nextion?
    buttonread();
  }


  // if (gpstimer.check())
  //  {
  //   gpsupdate();
  //  }
  // sending data to nextion display
  dashupdate();
  //dashreturn();
}
