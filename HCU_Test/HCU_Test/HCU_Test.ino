#define BUTTON_UP 18
#define BUTTON_DOWN 19
#define BOOST 10
#define MODE_OUT 13
#define BOOST_OUT 12
#define LED1 14
#define LED2 15
#define LED3 16
#define LED4 17
#define VBATPIN A7
#define BOOST_LED_TIME 300
#define PWM_T 20.0
#define DC 0.001
#define BOOST_TIMEOUT 5000
#define REDBLINK 200

uint8_t mode = 0;
int toggletime = 0;
int boosttoggletime = 0;
bool toggle_high = true;
bool mode_changed = false;
float measuredvbat;
bool boost_toggle = false;
bool prev_boostpin = true;
int firstPress = 0;
int unpressed = 0;
bool pressed = false;
bool cooldown = false;
int redblinktime = 0;
bool redon = true;
/*TODO: BLINK LEDs WHEN VOTLAGE IS LOW*/
void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BOOST, INPUT_PULLUP);
  attachInterrupt(BUTTON_UP, next_mode, FALLING);
  attachInterrupt(BUTTON_DOWN, prev_mode, FALLING);
  //attachInterrupt(BOOST, recordTime, FALLING);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  pinMode(MODE_OUT, OUTPUT);
  pinMode(BOOST_OUT, OUTPUT);
  Serial.begin(115200);

}

void loop() {
  measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  if (measuredvbat < 3.1) {
    digitalWrite(LED3, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    while (1) {
      digitalWrite(LED4, HIGH);
      delay(500);
      digitalWrite(LED4, LOW);
      delay(500);
    }
  } else {
    bool boostpin = digitalRead(BOOST);

    if ((!boostpin && boostpin != prev_boostpin) && (millis() > firstPress + unpressed * 4)) {
      firstPress = millis();
      unpressed = 0;
      pressed = true;
    }
    if ((!boostpin && boostpin != prev_boostpin) && (millis() < firstPress + unpressed * 4)) {
      cooldown = true;
    }

    if ((boostpin && boostpin != prev_boostpin) && pressed) {
      unpressed = millis() - firstPress;
      pressed = false;
    }
    if ((boostpin && boostpin != prev_boostpin) && !pressed) {
      cooldown = false;
    }

    prev_boostpin = boostpin;
    if (millis() < firstPress + unpressed * 4) {
      boostpin = true;
    }

    if (!boostpin) {
      if (millis() > firstPress + BOOST_TIMEOUT) {
        boostpin = true;
        //prev_boostpin = true;
      }
    }
    Serial.println(cooldown);
    //    if (cooldown) {
    //      if (millis() > redblinktime + REDBLINK) {
    //        digitalWrite(LED4, LOW);
    //        redblinktime = millis();
    //      }
    //    }


    if (boostpin) {
      digitalWrite(BOOST_OUT, LOW);
      int t = millis();
      if (toggle_high) {
        if (t > (toggletime + PWM_T * (1.0 - DC))) {
          switch (mode) {
            case 0:
              if (!cooldown) {
                digitalWrite(LED4, HIGH);
              }
              digitalWrite(LED1, LOW);
              digitalWrite(LED2, LOW);
              digitalWrite(LED3, LOW);
              break;
            case 1:
              if (!cooldown) {
                digitalWrite(LED4, HIGH);
              }
              digitalWrite(LED3, HIGH);
              digitalWrite(LED1, LOW);
              digitalWrite(LED2, LOW);
              break;
            case 2:
              if (!cooldown) {
                digitalWrite(LED4, HIGH);
              }
              digitalWrite(LED3, HIGH);
              digitalWrite(LED2, HIGH);
              digitalWrite(LED1, LOW);
              break;
            case 3:
              if (!cooldown) {
                digitalWrite(LED4, HIGH);
              }
              digitalWrite(LED3, HIGH);
              digitalWrite(LED2, HIGH);
              digitalWrite(LED1, HIGH);
              break;

          }

          toggle_high = false;
          toggletime = t;
        }
      }

      if (!toggle_high) {
        if (t > (toggletime + PWM_T * DC)) {
          digitalWrite(LED1, LOW);
          digitalWrite(LED2, LOW);
          digitalWrite(LED3, LOW);
          if (cooldown) {
            if (millis() > redblinktime + REDBLINK) {
              redblinktime = millis();

              if (redon) {
                digitalWrite(LED4, LOW);
                redon = false;
              } else {
                digitalWrite(LED4, HIGH);
                redon = true;
              }

            }
          } else {
            digitalWrite(LED4, LOW);
          }
          toggle_high = true;
          toggletime = t;
        }
      }


    } else {

      digitalWrite(BOOST_OUT, HIGH);
      if (millis() > boosttoggletime + BOOST_LED_TIME) {
        boosttoggletime = millis();
        if (boost_toggle) {
          digitalWrite(LED1, LOW);
          digitalWrite(LED2, LOW);
          digitalWrite(LED3, LOW);
          boost_toggle = false;
        } else {
          digitalWrite(LED1, HIGH);
          digitalWrite(LED2, HIGH);
          digitalWrite(LED3, HIGH);
          boost_toggle = true;
        }
      }


    }

  }


  int t = millis();

  if (mode_changed) {
    for (int i = 0; i <= mode; ++i) {
      if (i != 0) {
        delay(2);
      }
      digitalWrite(MODE_OUT, HIGH);
      delay(2);
      digitalWrite(MODE_OUT, LOW);

    }
    mode_changed = false;
  }






}

void next_mode(void) {
  if (mode < 3) {
    mode++;
    mode_changed = true;
  }
}
void prev_mode(void) {
  if (mode > 0) {
    mode --;
    mode_changed = true;
  }
}
