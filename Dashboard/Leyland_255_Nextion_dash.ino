#include <FlexCAN_T4.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#define NUM_TX_MAILBOXES 6
#define NUM_RX_MAILBOXES 6
CAN_message_t msg;


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

/// gps stuff
// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(400);
  Serial2.begin(9600); // is this ok?
  delay(400);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++) {
    Can0.setMB(i, RX, STD);
  }
  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++) {
    Can0.setMB(i, TX, STD);
  }
  //  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  //  Can0.setMBFilterProcessing(MB0, 0x3FF, 0xFF);
  //Can0.setMBFilterProcessing(MB1,0x400, 0xFF);
  //Can0.setMBFilterProcessing(MB2,0x0B,0xFF);
  // Can0.enhanceFilter(MB0);
  //Can0.enhanceFilter(MB1);
  Can0.onReceive(MB0, canSniff1);
  //Can0.onReceive(MB1,canSniff2);
  //Can0.onReceive(MB2,canSniff);
  Can0.mailboxStatus();
}


void canSniff1(const CAN_message_t &msg) { //edit for Leaf canbus messages

  if (msg.id == 0x55A) // from leaf inverter
  {
    inverT = msg.buf[2];//INVERTER TEMP
    MotorT = msg.buf[1];//MOTOR TEMP

  }

  if (msg.id == 0x1DA)  // from leaf inverter
  {
    rpm = (( msg.buf[4] << 8) | msg.buf[5]);
  }


  if (msg.id == 0x355)
  {
    SOC = (( msg.buf[1] << 8) | msg.buf[0]);
    //Serial.println("SIMPBMS");
  }
  if (msg.id == 0x356)//battery voltage from SIMP BMS
  {

    Batvoltraw = (( msg.buf[1] << 8) | msg.buf[0]);
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
  Can0.events();
  // sending data to nextion display


  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      mph = gps.speed.mph();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while (true);
  }


}
