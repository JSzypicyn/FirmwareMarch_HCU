#ifndef PAS_FILE
#define PAS_FILE
#include <PinDefs.h>
#include <BLE_Cad.h>
#include "MedianFilterLib.h"


FIR<float, FIR_FILTER_LEN_MA_SPEED> firCad;
HardwareTimer *Time = new HardwareTimer(TIM1);
bool DetectCad = false;
bool DetectSpeed = false;
int MicrosCad = 0;
int MicrosCadprev = 0;
int MicrosSpeed = 0;
int MicrosSpeedprev = 0;
int missedCounts = 0;
int lastSpeedUpdate = 0;
int totalT = 0;

bool Riding = false;
int8_t pedCount = 0;
//bool notMoving = true;

void checkStandstill(void) {
//  if (speedConnected) {
//    if (bikeSpeed == 0) {
//      standstillStart = true;
//      standstillStart_init = false;
//    }
//    
//  } else {
    if (crankRPM == 0) {
      standstillStart = true;
      standstillStart_init = false;
    }
//  }

}

void ISRSpeed(void) {
  //flag
  if (missedCounts > 10) {
    missedCounts = 0;
    totalT = 0;
  }
  DetectSpeed = true;
  //capture time
  MicrosSpeed = micros();
  //MicrosSpeed = micros();
  missedCounts++;
  if (MicrosSpeed > MicrosSpeedprev) {
    totalT += MicrosSpeed - MicrosSpeedprev;
  } else {
    totalT += OVERFLOW_VAL - MicrosSpeedprev + MicrosSpeed;
  }
  MicrosSpeedprev = MicrosSpeed;
  lastSpeedUpdate = millis();


}

void ISRCad(void) {
  //flag
  DetectCad = true;
  pedCount++;

  //capture time
  MicrosCad = micros();//Time->getCount(MICROSEC_FORMAT);

}

void calculateSpeed(void) {

  if (missedCounts >= 2) {
    wheelSpeed = WHEEL_DIST * missedCounts / (7 * 1e-6 * totalT);
    totalT = 0;
    missedCounts = 0;
  }
}

void setupPAS(void) {
  pinMode(PASCad, INPUT);
  pinMode(HALLSpeed, INPUT_PULLUP);
  Time->pause();
  Time->setCount(0);
  Time->setPrescaleFactor(720000);
  Time->setMode(4, TIMER_OUTPUT_COMPARE);
  Time->refresh();
  Time->resume();
  attachInterrupt(PASCad, ISRCad, RISING);
  attachInterrupt(HALLSpeed, ISRSpeed, FALLING);
  firCad.setFilterCoeffs(aMA);
}

void pedalTimeoutCheck() {
  int currTime = micros();
  if (Pedal) {
    if (currTime >= MicrosCadprev) {
      if (currTime - MicrosCadprev > CAD_THRESHOLD_US) {
        Pedal = false;
        cadence = 0;
        pedCount = 0;
      }
    } else {
      if (OVERFLOW_VAL - MicrosCadprev + currTime > CAD_THRESHOLD_US) {
        Pedal = false;
        cadence = 0;
        pedCount = 0;
      }
    }
  }
}


float speedCorrectionFactor(void) {

  if (wheelSpeed < 4.7) {
    return 1.0;
  } else {
    if (wheelSpeed > 7.0) {
      return 0.0;
    } else {
      return (1.0 - (wheelSpeed - 4.7) / 2.3);
    }
  }
}

#endif
