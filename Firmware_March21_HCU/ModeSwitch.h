#ifndef MODE_FILE
#define MODE_FILE
#include <Math.h>
MedianFilter<float> cadenceMedian(5);
int8_t mode = 0;
int lastModeUpdate = 0;
int lastCountUpdate = 0;

inline float my_min(float a, float b) {
  return ((a) > (b) ? (b) : (a));
}

unsigned long endTime;


/* INTERRUPT HANDLER FOR MODE SELECTION BUTTON */
void changeMode() {
  if(millis()>(lastModeUpdate+500)){
    mode  = (mode+1)%NUM_MODES;
    lastModeUpdate = millis();
  }
}

void countMode() {
  if(millis()>(lastCountUpdate+10)){
    mode = -1;
  }
    mode++;
    lastCountUpdate = millis();
}

/* INTERRUPT SETUP FOR MODE BUTTONS */
void setupModeSwitch(void) {
  pinMode(MODE_BUTTON, INPUT);
  pinMode(MODE_HCU, INPUT);
  pinMode(BOOST_HCU, INPUT);
  attachInterrupt(MODE_BUTTON, changeMode, FALLING);
  attachInterrupt(MODE_HCU, countMode, RISING);
  //attachInterrupt(BOOST_HCU, boostOn, CHANGE);
  //changeMode();
}

void calculateCadence(void) {
  if (MicrosCad > MicrosCadprev) {
    cadence = 60 / (12 * 1e-6 * (MicrosCad - MicrosCadprev));
  } else {
    cadence = 60 / (12 * 1e-6 * (OVERFLOW_VAL - MicrosCadprev + MicrosCad));
  }

  cadence = cadenceMedian.AddValue(cadence);
  cadence = firCad.processReading(cadence);
}


/*
   OFF MODE
   TURNS OFF MOTOR BY SETTING POWER TO 0
*/

void OFF_MODE(void) {
  if (initDone) {
    setPowerPID(0,true);
  }
}

/*
   HILL MODE
   1.5 seconds of boost
   followed by gradient dependent power addition
   provided cadence>0 and speed <25kmh
   max power limitation of 350W
*/

void HILL_MODE(void) {

  float pwr_set;
  // IF USER IS PEDALLING
 
  if (Pedal) {

    // IF ABOVE MIN SPEED TO ENSURE NO RANDOM STARTUP
      // IS HE WAS PREVIOUSLY STATIONARY AND THE STRATUP RPOCEDURE WASNT INITIATED YET
      if (standstillStart && !standstillStart_init) {
        // INITIATE THE STARTUP
        standstillStart_init = true;
        // CALCULTE THE END TIME FOR STARTUP
        unsigned long currTime = millis();
        endTime = currTime + STANDSTILL_DURATION;

      }

    // IF PERFORMING STARUP PROCEDURE
    if (standstillStart_init) {
      // USER IS ALREADY MOVING
      standstillStart = false;
      // HAS STARUP TIME EXPIRED?
      if (millis() > endTime) {
        // INDICATE END OF STARTUP
        standstillStart_init = false;

      } else {

        // KEEP DELIVERING STARTUP POWER
        pwr_set = my_min((millis()-(endTime-STANDSTILL_DURATION))*(2*HILLS_START_POWER/STANDSTILL_DURATION),HILLS_START_POWER);
      }
    }

    // IS USER MOVING AND STARTUP FINISHED?
    if (!standstillStart_init && !standstillStart) {

      //if ((incline > INCLINE_HARD_LIMIT) && (crankRPM>CAD_LIM)) {
      if (incline > INCLINE_HARD_LIMIT) {
        pwr_set = HILLS_BASE_POWER + HILLS_SCALE_FACTOR * (incline - INCLINE_OFFSET) + CADENCE_SCALE_FACTOR * (crankRPM - CADENCE_OFFSET);
        //pwr_set = my_min(pwr_set*speedCorrectionFactor(), HILLS_POWER_LIMIT);
        pwr_set = my_min(pwr_set, HILLS_POWER_LIMIT);

        if (incline < INCLINE_THRESHOLD) {
          float grad = (HILLS_BASE_POWER + HILLS_SCALE_FACTOR * (INCLINE_THRESHOLD - INCLINE_OFFSET) + CADENCE_SCALE_FACTOR * (crankRPM - CADENCE_OFFSET)) / (INCLINE_THRESHOLD - INCLINE_HARD_LIMIT) ;
          pwr_set = (HILLS_BASE_POWER + HILLS_SCALE_FACTOR * (INCLINE_THRESHOLD - INCLINE_OFFSET) + CADENCE_SCALE_FACTOR * (crankRPM - CADENCE_OFFSET)) - grad * (INCLINE_THRESHOLD - incline);
          //pwr_set = pwr_set*speedCorrectionFactor();
        }

        if (pwr_set < 0) pwr_set = 0;
      } else {
        pwr_set = 0;
      }
    }
    // IF USER IS NOT PEDALLING
  } else {
    // DELIVER 0 W
    pwr_set = 0;
    // INDICATE END OF STARTUP HERE AS WELL IN CASE IT IS INTERRUPTED
    standstillStart_init = false;
  }
  setPowerPID(pwr_set);
}

void HILL_POWER_MODE(void) {

  float pwr_set;
  // IF USER IS PEDALLING
 
  if (Pedal) {

    // IF ABOVE MIN SPEED TO ENSURE NO RANDOM STARTUP
      // IS HE WAS PREVIOUSLY STATIONARY AND THE STRATUP RPOCEDURE WASNT INITIATED YET
      if (standstillStart && !standstillStart_init) {
        // INITIATE THE STARTUP
        
        standstillStart_init = true;
        // CALCULTE THE END TIME FOR STARTUP
        unsigned long currTime = millis();
        endTime = currTime + STANDSTILL_DURATION;

      }
    // IF PERFORMING STARUP PROCEDURE
    if (standstillStart_init) {

      // USER IS ALREADY MOVING
      standstillStart = false;
      // HAS STARUP TIME EXPIRED?
      if (millis() > endTime) {
        // INDICATE END OF STARTUP
        standstillStart_init = false;

      } else {

        // KEEP DELIVERING STARTUP POWER
        pwr_set = my_min((millis()-(endTime-STANDSTILL_DURATION))*(2*HILLS_POWER_START_POWER/STANDSTILL_DURATION),HILLS_POWER_START_POWER);
      }
    }

    // IS USER MOVING AND STARTUP FINISHED?
    if (!standstillStart_init && !standstillStart) {
     
      //if ((incline > INCLINE_HARD_LIMIT) && (crankRPM>CAD_LIM)) {
      if (incline > INCLINE_HARD_LIMIT) {
        pwr_set = HILLS_POWER_BASE_POWER + HILLS_POWER_SCALE_FACTOR * (incline - INCLINE_OFFSET) + CADENCE_POWER_SCALE_FACTOR * (crankRPM - CADENCE_POWER_OFFSET);
        //pwr_set = my_min(pwr_set*speedCorrectionFactor(), HILLS_POWER_LIMIT);
        pwr_set = my_min(pwr_set, HILLS_POWER_POWER_LIMIT);

        if (incline < INCLINE_THRESHOLD) {
          float grad = (HILLS_POWER_BASE_POWER + HILLS_POWER_SCALE_FACTOR * (INCLINE_THRESHOLD - INCLINE_OFFSET) + CADENCE_POWER_SCALE_FACTOR * (crankRPM - CADENCE_POWER_OFFSET)) / (INCLINE_THRESHOLD - INCLINE_HARD_LIMIT) ;
          pwr_set = (HILLS_POWER_BASE_POWER + HILLS_POWER_SCALE_FACTOR * (INCLINE_THRESHOLD - INCLINE_OFFSET) + CADENCE_POWER_SCALE_FACTOR * (crankRPM - CADENCE_POWER_OFFSET)) - grad * (INCLINE_THRESHOLD - incline);
          //pwr_set = pwr_set*speedCorrectionFactor();
        }

        if (pwr_set < 0) pwr_set = 0;
      } else {
        pwr_set = 0;
      }
    }
    // IF USER IS NOT PEDALLING
  } else {
    // DELIVER 0 W
    pwr_set = 0;
    // INDICATE END OF STARTUP HERE AS WELL IN CASE IT IS INTERRUPTED
    standstillStart_init = false;
  }
  setPowerPID(pwr_set);
}


void HILL_SUPER_MODE(void) {

  float pwr_set;
  // IF USER IS PEDALLING
 
  if (Pedal) {

    // IF ABOVE MIN SPEED TO ENSURE NO RANDOM STARTUP
      // IS HE WAS PREVIOUSLY STATIONARY AND THE STRATUP RPOCEDURE WASNT INITIATED YET
      if (standstillStart && !standstillStart_init) {
        // INITIATE THE STARTUP
        
        standstillStart_init = true;
        // CALCULTE THE END TIME FOR STARTUP
        unsigned long currTime = millis();
        endTime = currTime + STANDSTILL_DURATION;

      }
    // IF PERFORMING STARUP PROCEDURE
    if (standstillStart_init) {

      // USER IS ALREADY MOVING
      standstillStart = false;
      // HAS STARUP TIME EXPIRED?
      if (millis() > endTime) {
        // INDICATE END OF STARTUP
        standstillStart_init = false;

      } else {

        // KEEP DELIVERING STARTUP POWER
        pwr_set = my_min((millis()-(endTime-STANDSTILL_DURATION))*(2*HILLS_SUPER_START_POWER/STANDSTILL_DURATION),HILLS_SUPER_START_POWER);
      }
    }

    // IS USER MOVING AND STARTUP FINISHED?
    if (!standstillStart_init && !standstillStart) {
     
      //if ((incline > INCLINE_HARD_LIMIT) && (crankRPM>CAD_LIM)) {
      if (incline > INCLINE_HARD_LIMIT) {
        pwr_set = HILLS_SUPER_BASE_POWER + HILLS_SUPER_SCALE_FACTOR * (incline - INCLINE_OFFSET) + CADENCE_SUPER_SCALE_FACTOR * (crankRPM - CADENCE_SUPER_OFFSET);
        //pwr_set = my_min(pwr_set*speedCorrectionFactor(), HILLS_POWER_LIMIT);
        pwr_set = my_min(pwr_set, HILLS_SUPER_POWER_LIMIT);

        if (incline < INCLINE_THRESHOLD) {
          float grad = (HILLS_SUPER_BASE_POWER + HILLS_SUPER_SCALE_FACTOR * (INCLINE_THRESHOLD - INCLINE_OFFSET) + CADENCE_SUPER_SCALE_FACTOR * (crankRPM - CADENCE_SUPER_OFFSET)) / (INCLINE_THRESHOLD - INCLINE_HARD_LIMIT) ;
          pwr_set = (HILLS_SUPER_BASE_POWER + HILLS_SUPER_SCALE_FACTOR * (INCLINE_THRESHOLD - INCLINE_OFFSET) + CADENCE_SUPER_SCALE_FACTOR * (crankRPM - CADENCE_SUPER_OFFSET)) - grad * (INCLINE_THRESHOLD - incline);
          //pwr_set = pwr_set*speedCorrectionFactor();
        }

        if (pwr_set < 0) pwr_set = 0;
      } else {
        pwr_set = 0;
      }
    }
    // IF USER IS NOT PEDALLING
  } else {
    // DELIVER 0 W
    pwr_set = 0;
    // INDICATE END OF STARTUP HERE AS WELL IN CASE IT IS INTERRUPTED
    standstillStart_init = false;
  }
  setPowerPID(pwr_set);
}


void HILL_MODEL_MODE(void) {

  float pwr_set;
  // IF USER IS PEDALLING
 
  if (Pedal) {

    // IF ABOVE MIN SPEED TO ENSURE NO RANDOM STARTUP
      // IS HE WAS PREVIOUSLY STATIONARY AND THE STRATUP RPOCEDURE WASNT INITIATED YET
      if (standstillStart && !standstillStart_init) {
        // INITIATE THE STARTUP
        
        standstillStart_init = true;
        // CALCULTE THE END TIME FOR STARTUP
        unsigned long currTime = millis();
        endTime = currTime + STANDSTILL_DURATION;

      }
    // IF PERFORMING STARUP PROCEDURE
    if (standstillStart_init) {

      // USER IS ALREADY MOVING
      standstillStart = false;
      // HAS STARUP TIME EXPIRED?
      if (millis() > endTime) {
        // INDICATE END OF STARTUP
        standstillStart_init = false;

      } else {

        // KEEP DELIVERING STARTUP POWER
        pwr_set = my_min((millis()-(endTime-STANDSTILL_DURATION))*(2*HILLS_MODEL_START_POWER/STANDSTILL_DURATION),HILLS_MODEL_START_POWER);
      }
    }

    // IS USER MOVING AND STARTUP FINISHED?
    if (!standstillStart_init && !standstillStart) {
     
      //if ((incline > INCLINE_HARD_LIMIT) && (crankRPM>CAD_LIM)) {
      if (incline > INCLINE_MODEL_HARD_LIMIT) {
        float k1 = VELOCITY*MASS*CRR*GRAVITY;
        float k2 = k1/CRR;
        pwr_set = (k1+k2*sin(incline*M_PI/180.0))*float(crankRPM)/(CADENCE_MODEL_OFFSET*EFFICIENCY);
        //pwr_set = HILLS_SUPER_BASE_POWER + HILLS_SUPER_SCALE_FACTOR * (incline - INCLINE_OFFSET) + CADENCE_SUPER_SCALE_FACTOR * (crankRPM - CADENCE_SUPER_OFFSET);
        //pwr_set = my_min(pwr_set*speedCorrectionFactor(), HILLS_POWER_LIMIT);
        pwr_set = my_min(pwr_set, HILLS_MODEL_POWER_LIMIT);

        if (incline < INCLINE_MODEL_THRESHOLD) {
          float grad = (k1+k2*sin(INCLINE_MODEL_THRESHOLD*M_PI/180.0))*float(crankRPM)/(CADENCE_MODEL_OFFSET*EFFICIENCY);
          grad = grad/(INCLINE_MODEL_THRESHOLD-INCLINE_MODEL_HARD_LIMIT);
           pwr_set = ((k1+k2*sin(INCLINE_MODEL_THRESHOLD*M_PI/180.0))*float(crankRPM)/(CADENCE_MODEL_OFFSET*EFFICIENCY)) - grad * (INCLINE_MODEL_THRESHOLD - incline);
          //pwr_set = pwr_set*speedCorrectionFactor();
        }

        if (pwr_set < 0) pwr_set = 0;
      } else {
        pwr_set = 0;
      }
    }
    // IF USER IS NOT PEDALLING
  } else {
    // DELIVER 0 W
    pwr_set = 0;
    // INDICATE END OF STARTUP HERE AS WELL IN CASE IT IS INTERRUPTED
    standstillStart_init = false;
  }
  setPowerPID(pwr_set);
}

/*
    CADENCE BASED MODE ONLY
    COPY OF SWYTCH ROUGHLY
    DELIVERS STARUP POWER
    THEN DELIVERS 60W PLUS CADENCE DEPENDEND EXTRA
*/
//void SWYTCH_MODE2(void) {
//
//  float pwr_set;
//
//  // WHEN PEDAL MOVEMENT IS REGISTERED
//  if (pedCount == 3) Pedal = true;
//
//  // IF USER IS PEDALLING
//  if (Pedal) {
//
//    // IF ABOVE MIN SPEED TO ENSURE NO RANDOM STARTUP
//    if (previous_mean_speed > MIN_SPEED_MS) {
//      // IS HE WAS PREVIOUSLY STATIONARY AND THE STRATUP RPOCEDURE WASNT INITIATED YET
//      if (standstillStart && !standstillStart_init) {
//        // INITIATE THE STARTUP
//        standstillStart_init = true;
//        // CALCULTE THE END TIME FOR STARTUP
//        unsigned long currTime = millis();
//        endTime = currTime + STANDSTILL_DURATION;
//      }
//    }
//
//    // GET MOST RECENT CADENCE VALUE
//    if (DetectCad) calculateCadence();
//    // RESET CADENCE FLAG - THIS CAN BE OPTIMISED
//    DetectCad = false;
//    MicrosCadprev = MicrosCad;
//
//    // IF PERFORMING STARUP PROCEDURE
//    if (standstillStart_init) {
//
//      // USER IS ALREADY MOVING
//      standstillStart = false;
//      dataString += String(1);
//      // HAS STARUP TIME EXPIRED?
//      if (millis() > endTime) {
//
//        // INDICATE END OF STARTUP
//        standstillStart_init = false;
//
//      } else {
//
//        // KEEP DELIVERING STARTUP POWER
//        pwr_set = SWYTCH_START_POWER;
//
//      }
//    }
//
//    // IS USER MOVING AND STARTUP FINISHED?
//    if (!standstillStart_init && !standstillStart) {
//      
//      // ESTIMATE POWER TO BE DELIVERED CAPPED BY THE UPPER LIMIT
//      pwr_set = my_min(SWYTCH_POWER_LIMIT, SWYTCH_BASE_POWER + (SWYTCH_POWER_LIMIT - SWYTCH_BASE_POWER) * cadence / SWYTCH_MAX_CADENCE);
//
//    }
//
//    // IF USER IS NOT PEDALLING
//  } else {
//    // DELIVER 0 W
//    pwr_set = 0;
//    // INDICATE END OF STARTUP HERE AS WELL IN CASE IT IS INTERRUPTED
//    standstillStart_init = false;
//  }
//
//  
//  setPowerPID(pwr_set);
//
//}

#endif
