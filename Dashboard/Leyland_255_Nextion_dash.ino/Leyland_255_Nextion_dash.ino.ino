#include <due_can.h>
#include <Metro.h>
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
CAN_FRAME msg;


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

/// gps stuff
// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
//SoftwareSerial gpsSerial(RXPin, TXPin);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(400);
  Serial2.begin(9600);  // nextion display
  delay(400);
  Serial3.begin(9600);  // Start the software serial port at the GPS's default baud
    delay(400);

  Serial.println("Leyland Dash");
 
  Can0.begin(CAN_BPS_500K);
 
}


void canSniff1() { //edit for Leaf canbus messages
  
  Can0.read(msg);

  if (msg.id == 0x55A) // from leaf inverter
  {
    inverT = msg.data.bytes[2];//INVERTER TEMP
    MotorT = msg.data.bytes[1];//MOTOR TEMP

  }

  if (msg.id == 0x1DA)  // from leaf inverter
  {
    rpm = (( msg.data.bytes[4] << 8) | msg.data.bytes[5]);
  }


  if (msg.id == 0x355)
  {
    SOC = (( msg.data.bytes[1] << 8) | msg.data.bytes[0]);
    //Serial.println("SIMPBMS");
  }
  if (msg.id == 0x356)//battery voltage from SIMP BMS
  {

    Batvoltraw = (( msg.data.bytes[1] << 8) | msg.data.bytes[0]);
    volt = Batvoltraw / 10;
    //Serial.println("SIMPBMS2");
  }

}
void dashupdate() //Run every 500ms or so depending on refreash desired.
{

  Serial2.write(0x22);
  Serial2.print("soc.val=");
  Serial2.print(SOC);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("soc1.val=");
  Serial2.print(SOC);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("amps.val=");
  Serial2.print(amps);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("motorT.val=");
  Serial2.print(MotorT);// Get motor temps
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("BattT.val=");
  Serial2.print(Batttemp); //get battery temp
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("InverT.val=");
  Serial2.print(inverT);// get inverter temp
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("volt.val=");
  Serial2.print(volt); //get battery pack voltage
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("kW.val=");
  Serial2.print(kW); //get kW value
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("mph.val=");
  Serial2.print(mph);//get mph
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("celldelta.val=");
  // Serial2.print(); // to do
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("rpm.val=");
  Serial2.print(rpm); //get rpm
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
}



void loop() {
  // put your main code here, to run repeatedly:
  //Can0.events();
  // sending data to nextion display
  if (Can0.available() > 0) {
    canSniff1();
  }

  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      mph = gps.speed.mph();
      Serial.println(mph);

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while (true);
  }


}
