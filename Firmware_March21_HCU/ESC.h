#ifndef ESC_FILE
#define ESC_FILE

#include <VescUart.h>
#include <PinDefs.h>
#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>
#include <PID_v1.h>
#include <BLE_Cad.h>

inline float my_max(float a, float b) {
  return ((a) > (b) ? (a) : (b));
}

HardwareSerial Serial2(UART_RX, UART_TX);
VescUart UART;

double SetPower, PowerMeasure, CurrentOut;

/*TO DO: TUNE VARIOS PID MODES WITH FREEWHEEL*/
double Kp = 0.04, Ki = 0.4, Kd = 0.0001; // Startup PID
double Kp_run = 0.01, Ki_run = 0.1, Kd_run = 0.00001; // Normal Operation PID
double Kp_stop = 0.3, Ki_stop = 1, Kd_stop = 0.0001; // Stopping PID
double Kp_slow = 0.0008, Ki_slow = 0.015, Kd_slow = 0.0001; //slow startup

bool dooncePID = false;
bool activePID = false;
bool PPM_off = true;
int samp_ctr = 0;
int slowPIDtime = 0;
float startPower = 0.05;
int fix_ctr = 0;
bool fix_mode = false;
PID myPID(&PowerMeasure, &CurrentOut, &SetPower, Kp, Ki, Kd, DIRECT);

void setupESC(void) {
  // UART to talk with ESC
  Serial2.begin(115200);
  while (!Serial2) {}
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial2);
  SetPower = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(MIN_CURRENT, MAX_CURRENT);
  myPID.SetSampleTime(14);

}

float getPower(void) {
  float power;
  if ( UART.getVescValues()) {
    power = UART.data.avgMotorCurrent * UART.data.inpVoltage * abs(UART.data.dutyCycleNow);
  }
  else {
    power = 0;
  }

  return power;
}

float getSpeed(void) {
  //UART.getVescValues();
  return (3.6 * WHEEL_CMF * UART.data.rpm / (7 * GEARBOX_REDUCTION * 60.0));
}

float totPowEst(float vel, float inc) {
  return ((vel / 3.6) * 9.81 * 100 * (0.02 + sin(inc)));
}

float calculateTorque(float pwr, int rpm_mot) {
  if (rpm_mot == 0) {
    return 0;
  } else {
    return pwr * 60.0 * GEARBOX_REDUCTION / (2 * M_PI * float(rpm_mot));
  }
}


void setPowerPID(float power_target, bool off_mode = false) {
  if (digitalRead(BOOST_HCU)) {
    power_target += 150;
  }

  // GET CURRENT POWER OF THE MOTOR

  float motor_power = getPower();
  //Serial1.println(UART.data.avgMotorCurrent * UART.data.inpVoltage * abs(UART.data.dutyCycleNow));

  float torque = calculateTorque(motor_power, UART.data.rpm);

  if (torque > 0.9 * TORQUE_LIMIT) {
    power_target = 0.9 * TORQUE_LIMIT * UART.data.rpm * 2 * M_PI / (60 * GEARBOX_REDUCTION);
  }

  if ( (UART.data.rpm > 8000.0) || (power_target < MIN_POWER)) {
    fix_mode = false;
  }

  if (((power_target - motor_power) > 60.0) && (UART.data.rpm < 8000.0)) {
    fix_mode = true;
  }

  if (fix_mode) {
    SetPower = 10;
    PowerMeasure = 10;

  } else {
    SetPower = power_target;
    PowerMeasure = motor_power;
  }


  if (SetPower >= MIN_POWER) {
    if (!activePID) {
      slowPIDtime = millis() + 1000;
      dooncePID = true;
    }
    activePID = true;
    if (!standstillStart_init && !standstillStart) {
      myPID.SetTunings(Kp_run, Ki_run, Kd_run);
    } else {
      if (millis() < slowPIDtime) {
        myPID.SetTunings(Kp, Ki, Kd);
      } else {
        myPID.SetTunings(Kp_run, Ki_run, Kd_run);
      }

    }
  } else {
    myPID.SetTunings(Kp_stop, Ki_stop, Kd_stop);
  }


  if ((torque < TORQUE_LIMIT) && (abs(UART.data.dutyCycleNow) < 0.95)) {
    //Serial1.print("HERE ");
    if (((SetPower >= MIN_POWER) || (CurrentOut > 1.0)) && activePID) {
      myPID.Compute();

    } else {
      //CurrentOut = 0;
      activePID = false;
      samp_ctr = 0;
    }
  }

  if ((torque > TORQUE_LIMIT) || (abs(UART.data.dutyCycleNow) < 0.95)) {
    CurrentOut = 0.99 * CurrentOut;
  }

  // IF THE TARGET IS BELOW MIN POWER LIMIT
  if ((SetPower < MIN_POWER) && (CurrentOut < 1)) {

    // DELIVER NOTHING
    CurrentOut = 0;
    activePID = false;
    samp_ctr = 0;
  }

  if (CurrentOut < 0) {
    CurrentOut = 0;
    samp_ctr = 0;
  }

  if (CurrentOut == 0 && UART.data.rpm == 0) {
    if (off_mode) {
      CurrentOut = 0;
    } else {
      CurrentOut = 0.04;
    }
  }

  if (fix_mode) {
    if (fix_ctr < 5) {
      UART.setCurrent(0);
      fix_ctr++;
    }
    if (fix_ctr >= 5 && fix_ctr < 45) {
      UART.setCurrent(4.5);
      fix_ctr++;
    }
    if (fix_ctr >= 45) {
      fix_ctr = 0;
    }

  } else {

    UART.setCurrent(CurrentOut);
  }

}

#endif
