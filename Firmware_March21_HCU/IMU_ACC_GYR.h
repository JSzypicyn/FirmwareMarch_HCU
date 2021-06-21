#ifndef IMU_ACC_GYR_file
#define IMU_ACC_GYR_file

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <FlashStorage_STM32.h>
#include <Wire.h>
#include <PinDefs.h>
#include <Math.h>
#include <FIR.h>

static inline int8_t sgn(float val) {
  if (val < 0) return -1;
  if (val==0) return 1;
  return 1;
}

FIR<float, FIR_FILTER_LEN_LPF_ACC> firAcc;
FIR<float, FIR_FILTER_LEN_MA_SPEED> firSpeed;
FIR<float, FIR_FILTER_LEN_ACC> xacc_fir;
FIR<float, FIR_FILTER_LEN_ACC> yacc_fir;
FIR<float, FIR_FILTER_LEN_ACC> zacc_fir;
FIR<float, FIR_FILTER_LEN_EXTRA> extra_fir;
float currAngle = 0;
int sampleIMUtime;

Adafruit_MPU6050 mma;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

float newPitch = 0;
float previous_mean_speed = 0;
int8_t IMU_DS_ctr = 0;
float acc_est = 0;



const int WRITTEN_SIGNATURE = 0xBEEFDEED;

typedef struct
{
  float xacc;
  float zacc;
  float yacc;
  float xgyr;
  float ygyr;
  float zgyr;
  float refAngle1;
  float refAngle2;
  float refAngle3;
  float magG;
} MMAOUT;

float gyroAngleX = 0;
float gyroAngleY = 0;
float gyroAngleZ = 0;
float gyroAngleZ2 = 0;

MMAOUT zeroPoint;
float filtAngle = 0;

void getIMUData(void) {
  //mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
}

void readMeanIMU(void) {
  zeroPoint.xacc = 0;
  zeroPoint.yacc = 0;
  zeroPoint.zacc = 0;
  zeroPoint.xgyr = 0;
  zeroPoint.ygyr = 0;
  zeroPoint.zgyr = 0;
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, HIGH);
  for (int ctr = 0; ctr <= 256; ctr++) {
    getIMUData();
    if (ctr % 100 == 0) {
      digitalToggle(GLED);
      digitalToggle(RLED);
    }
  }
  for (int ctr = 0; ctr <= NUM_CAL_SAMPS; ctr++) {

    getIMUData();

    zeroPoint.xacc += accel.acceleration.x;
    zeroPoint.yacc += accel.acceleration.y;
    zeroPoint.zacc += accel.acceleration.z;
    zeroPoint.xgyr += gyro.gyro.x;
    zeroPoint.ygyr += gyro.gyro.y;
    zeroPoint.zgyr += gyro.gyro.z;

    if (ctr % 100 == 0) {
      digitalToggle(GLED);
      digitalToggle(RLED);
    }
  }
  zeroPoint.xacc = zeroPoint.xacc / 1024.0;
  zeroPoint.yacc = zeroPoint.yacc / 1024.0;
  zeroPoint.zacc = zeroPoint.zacc / 1024.0;
  zeroPoint.xgyr = zeroPoint.xgyr / 1024.0;
  zeroPoint.ygyr = zeroPoint.ygyr / 1024.0;
  zeroPoint.zgyr = zeroPoint.zgyr / 1024.0;
  zeroPoint.refAngle1 = 180 * atan (zeroPoint.xacc / sqrt(zeroPoint.yacc * zeroPoint.yacc + zeroPoint.zacc * zeroPoint.zacc)) / M_PI;
  zeroPoint.refAngle2 =  180 * atan (zeroPoint.yacc / sqrt(zeroPoint.xacc * zeroPoint.xacc + zeroPoint.zacc * zeroPoint.zacc)) / M_PI;
  zeroPoint.refAngle3 = 180 * atan (zeroPoint.zacc / sqrt(zeroPoint.yacc * zeroPoint.yacc + zeroPoint.xacc * zeroPoint.xacc)) / M_PI;
  zeroPoint.magG = sqrt(zeroPoint.yacc * zeroPoint.yacc + zeroPoint.zacc * zeroPoint.zacc + zeroPoint.xacc * zeroPoint.xacc);
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
}

// Setup IMU and Comms
void setupIMU(void) {
  // IMU is powered through a digital pin otherwise when a board undergoes soft reset, IMU remains powered and gets stuck.
  pinMode(IMU_PWR, OUTPUT);
  digitalWrite(IMU_PWR, LOW);
  delay(500);
  digitalWrite(IMU_PWR, HIGH);

  // Setup I2C Port
  Wire.setSDA(I2C_DataPin);
  Wire.setSCL(I2C_ClockPin);
  Wire.begin();
  Wire.setClock(400000L);
  // Connect
  mma.begin();
  // Keep trying to connect
  while (! mma.begin()) {
    Serial1.println("tuck");
    digitalWrite(RLED, HIGH);

  }
  digitalWrite(RLED, LOW);
  // Change the detection range in PinDefs.h
  //zeroPoint = IMUcal();
  mma.setAccelerometerRange(A_RANGE);
  mma.setGyroRange(G_RANGE);
  mma.setFilterBandwidth(BW_FILTER);

  mpu_temp = mma.getTemperatureSensor();
  mpu_accel = mma.getAccelerometerSensor();
  mpu_gyro = mma.getGyroSensor();

  readMeanIMU();
  firAcc.setFilterCoeffs(aLPF);
  firSpeed.setFilterCoeffs(aMA);
  xacc_fir.setFilterCoeffs(accMA);
  yacc_fir.setFilterCoeffs(accMA);
  zacc_fir.setFilterCoeffs(accMA);
  extra_fir.setFilterCoeffs(extraMA);
}

void IMUcal(void) {
  int signature;
  EEPROM.get(CAL_ADDRESS, signature);
  // If the EEPROM is empty then no WRITTEN_SIGNATURE
  if (signature == WRITTEN_SIGNATURE)
  {
    EEPROM.get(CAL_ADDRESS + sizeof(signature), zeroPoint);
  }
  else
  {
    EEPROM.put(CAL_ADDRESS, WRITTEN_SIGNATURE);
    readMeanIMU();
    // ...and finally save everything into emulated-EEPROM
    EEPROM.put(CAL_ADDRESS + sizeof(signature), zeroPoint);
  }
}


/* MAIN INCLINE CALCULATION FUNCTION */

float XYZtoInc() {
  float refAngle;
  // float magV;
  float filt_angle;

  /* GET TIME STEP */
  int time_temp = micros();
  int td;
  if (time_temp > sampleIMUtime) {
    td = time_temp - sampleIMUtime;
  } else {
    td = (OVERFLOW_32BIT - sampleIMUtime) + time_temp;
  }
  /* GET IMU DATA */
  getIMUData();

  /* ESTIMATE ACCELERATION OF THE BIKE */

  float current_mean_speed = firSpeed.processReading(wheelSpeed);
  acc_est = (current_mean_speed - previous_mean_speed) * 1 * 1e6 / float(td);
  //Serial1.println(acc_est);
  if (acc_est > 1) acc_est = 1;
  if (acc_est < -1) acc_est = -1;
  previous_mean_speed = current_mean_speed;
  //acc_est =0;

  /* Integrate Gyro Reading */
  gyroAngleZ = newPitch - (gyro.gyro.z - zeroPoint.zgyr) * float(td) * 1e-6;

  /* DOWNSAMPLING COUNTER */
  IMU_DS_ctr++;

  /* CALCULATE TOTAL G FORCE */
  // magV = sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z);

  /* ESTIMATE ANGLE FROM ACC-METER
      accounting for reference angle and physcial acceleration
      then filter
  */
  refAngle =  zeroPoint.refAngle2;
  float prev_currAngle = currAngle;

  if (abs(acc_est) > 0.6) {
    currAngle =  180 * atan ((accel.acceleration.y + acc_est * cos(refAngle * M_PI / 180.0) ) / sqrt((accel.acceleration.x - acc_est * sin(refAngle * M_PI / 180.0)) * (accel.acceleration.y - acc_est * sin(refAngle * M_PI / 180.0)) + accel.acceleration.z * accel.acceleration.z)) / M_PI;
  } else {
    currAngle =  180 * atan (accel.acceleration.y / sqrt((accel.acceleration.x) * (accel.acceleration.x) + accel.acceleration.z * accel.acceleration.z)) / M_PI;
  }
  if (isnan(currAngle)) {
    currAngle = prev_currAngle;
  }
  float x_temp = xacc_fir.processReading(accel.acceleration.x);
  float y_temp = yacc_fir.processReading(accel.acceleration.y);
  float z_temp = zacc_fir.processReading(accel.acceleration.z);
  float currAngle2 = 180 * atan (y_temp/ sqrt(x_temp * x_temp + z_temp * z_temp)) / M_PI;
  currAngle = extra_fir.processReading(currAngle2);
  
  //float
  filt_angle = LPF_ACC_GAIN * firAcc.processReading(-(currAngle - refAngle));

  /* STORE TIMESTAMP */
  sampleIMUtime = time_temp;

  /* FUSE SENSOR READINGS BY DOWNSAMPLING ACC-METER READOUT */
  if (current_mean_speed > 0) {
    if (IMU_DS_ctr == MOD_CTR) {

      newPitch = 0.96 * gyroAngleZ + 0.04 * (-(currAngle - refAngle));
      IMU_DS_ctr = 0;

    } else {

      newPitch =  0.99 * gyroAngleZ + 0.01 * filt_angle;

    }
  } else {
    if (IMU_DS_ctr == MOD_CTR) {

      newPitch = 0.9 * gyroAngleZ + 0.1 * (-(currAngle - refAngle));
      IMU_DS_ctr = 0;

    } else {

      newPitch =  0.99 * gyroAngleZ + 0.01 * filt_angle;

    }
  }
  return -newPitch;

}

#endif
