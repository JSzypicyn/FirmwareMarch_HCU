#include <IMU_ACC_GYR.h>
#include <ESC.h>
#include <PAS.h>
#include <BLE_Cad.h>
#include <Arduino.h>
#include <ModeSwitch.h>
#include <SDLogger.h>

MedianFilter<float> speedMedian(3);
int ledUpdateTime = 0;
const int blinkTime = 500;
bool m3_doonce = true;
bool startupBoost = false;
void setup() {
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
  Serial1.begin(115200);
  delay(500);

  setupESC();

  setupIMU();
  setupPAS();
  //IMUcal();
  setupBLE();
  setupModeSwitch();
  setupSD();
  digitalWrite(GLED, HIGH);
  delay(1000);
  initDone = true;

}


void loop() {


  Serial1.print(UART.data.inpVoltage);
  Serial1.print("\t");
    Serial1.print(UART.data.tempFET );
  Serial1.print("\t");
    Serial1.print(UART.data.tempMotor);
  Serial1.print("\t");
    Serial1.print(crankRPM);
  Serial1.print("\t");
  Serial1.println(UART.data.rpm);
  incline = XYZtoInc();
  //delay(15);
  checkStandstill();

  pollBLE();
  wheelSpeed = speedMedian.AddValue(getSpeed() / 3.6);
  if ((UART.data.tempFET > 80.0) || (UART.data.tempMotor > 70.0) ) {
    setPowerPID(0, true);
  } else {

    if ((wheelSpeed == 0 && !Pedal && digitalRead(BOOST_HCU) ) || startupBoost ) {
      Serial1.println("BOOST");
      setPowerPID(450);
      if (wheelSpeed > 6.0 || !digitalRead(BOOST_HCU)) {
        startupBoost = false;
      } else {
        startupBoost = true;
      }
    } else {
      switch (mode) {
        case 0:
          OFF_MODE();
          Pedal = false;
          standstillStart_init = false;
          standstillStart = true;
          m3_doonce = true;
          digitalWrite(GLED, LOW);
          if (millis() > ledUpdateTime + blinkTime) {
            digitalToggle(RLED);
            ledUpdateTime = millis();
          }

          break;
        case 1:

          HILL_MODE();
          m3_doonce = true;
          digitalWrite(RLED, LOW);
          if (millis() > ledUpdateTime + blinkTime) {
            digitalToggle(GLED);
            ledUpdateTime = millis();
          }
          break;
        case 2:
          HILL_POWER_MODE();
          m3_doonce = true;
          digitalWrite(GLED, HIGH);
          digitalWrite(RLED, LOW);
          break;

        case 3:
          HILL_SUPER_MODE();
          if (m3_doonce) {
            digitalWrite(GLED, HIGH);
            digitalWrite(RLED, LOW);
            m3_doonce = false;
          }
          if (millis() > ledUpdateTime + blinkTime / 3) {
            digitalToggle(GLED);
            digitalToggle(RLED);
            ledUpdateTime = millis();
          }
          break;

      }
    }
  }





  saveToCard();


}
