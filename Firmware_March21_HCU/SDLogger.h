#ifndef SD_LOGGER_FILE
#define SD_LOGGER_FILE
//HardwareSerial Serial(PA9,PA10);

#include <SPI.h>
#include <Arduino.h>
//#include <SD.h>
#include <SdFat.h>
#include <PinDefs.h>

SdFat sd;
SdFile dataFile;

String filename = "datalog";
String ext = ".txt";
int filectr = 0;
uint32_t ctr = 0;

void setupSD(void) {

  digitalWrite(RLED, HIGH);
  if (!sd.begin(SD_CS, SD_SCK_MHZ(72))) {
    while (1);
  }

  String file = filename + String(filectr) + ext;
  int str_len = file.length() + 3;
  char fname[str_len];
  file.toCharArray(fname, str_len);
  while (sd.exists(fname)) {
    filectr++;
    file = filename + String(filectr) + ext;
    file.toCharArray(fname, str_len);
  }

  dataFile.open(fname, O_RDWR | O_CREAT | O_APPEND);
  digitalWrite(RLED, LOW);
}

void saveToCard(void) {
  ctr++;
  dataString = "";
  dataString += (String(millis()) + ",");
  dataString += (String(wheelSpeed) + ",");
  dataString += (String(UART.data.tempFET) + ",");
  dataString += (String(UART.data.tempMotor) + ",");
  dataString += String(crankRPM) + ",";
  dataString += String(incline) + ",";
  dataString += (String(SetPower) + ",");
  dataString += (String(PowerMeasure) + ",");
  dataString += (String(CurrentOut) + ",");
  dataFile.println(dataString);
  //Serial1.println(filectr);
  if (ctr % 500 == 0) {
    dataFile.flush();
  }
}
#endif
