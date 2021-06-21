#ifndef BLE_Cad
#define BLE_Cad
#include <SoftwareSerial.h>
HardwareSerial BLE_UART(HM10_RX, HM10_TX);

String content = "";
char character;
int ctrx = 1;
int t = 0;

//Receive variables
bool speedConnected = false;
String speedBatteryString = "";
String speedRevolutionsString = "";
String speedTimestampString = "";
uint32_t speedBattery = 0;
uint32_t speedRevolutions = 0;
uint32_t speedRevolutions_prev = 0;
uint32_t speedTimestamp = 0;
uint32_t speedTimestamp_prev = 0;
float bikeSpeed = 0;

bool crankConnected = false;
String crankBatteryString = "";
String crankRPMString = "";
uint32_t crankBattery = 0;
uint32_t crankRPM = 0;


void decodeData() {
  // PARSE SPEED SENSOR DATA
  if (content.substring(2, 3) == "1") {
    speedConnected = true;
  }
  else {
    speedConnected = false;
  }

  speedBatteryString = content.substring(4, 6);
  speedRevolutionsString = content.substring(11, 20);
  speedTimestampString = content.substring(20, 25);
  speedBattery = speedBatteryString.toInt();
  speedRevolutions = speedRevolutionsString.toInt();
  speedTimestamp = speedTimestampString.toInt();

  // PARSE CADENCE SENSOR DATA
  if (content.substring(6, 7) == "1") {
    crankConnected = true;
  }
  else {
    crankConnected = false;
  }

  crankBatteryString = content.substring(8, 10);
  crankRPMString = content.substring(26, 28);
  crankBattery = crankBatteryString.toInt();
  //Serial1.println(crankBattery);
  if(crankConnected){
    crankRPM = crankRPMString.toInt();
  } else {
    crankRPM = 0;
  }

}

void calculateBikeSpeed(void) {
  if ((speedRevolutions - speedRevolutions_prev) > 0) {

    if (speedTimestamp < speedTimestamp_prev){
      bikeSpeed = 3.6*float(speedRevolutions - speedRevolutions_prev) * WHEEL_CMF / float((speedTimestamp +(65535.0 - speedTimestamp_prev))/1024.0);
    } else {
      bikeSpeed = 3.6*float(speedRevolutions - speedRevolutions_prev) * WHEEL_CMF / float((speedTimestamp - speedTimestamp_prev)/1024.0);
    }
   
    speedRevolutions_prev = speedRevolutions;
    speedTimestamp_prev = speedTimestamp;

  }

}

void setupBLE(void) {
  BLE_UART.begin(115200);  //Start HM-10 serial
  BLE_UART.setTimeout(2);  //Small timeout needed!
  delay(100);
}

void pollBLE(void) {

  while (BLE_UART.available()) {
    character = BLE_UART.read();
    content.concat(character);
    if (character == 10) {
      decodeData();
      calculateBikeSpeed();
      content = "";
      if (crankRPM >= CAD_LIM) {
        Pedal = true;
      } else {
        Pedal = false;
      }
    }

  }

}


#endif
