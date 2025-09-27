#define CH1 8           // RIGHT MOTOR
#define CH2 13          // LEFT MOTOR
#define CH3 7           // WEAPON INPUT
#define CH5 4           // INVERT SWITCH
#define CH6 2           // 3-POS SWITCH

#define MOT_A_LPWM 5    // Drive Motor A
#define MOT_A_RPWM 3
#define MOT_B_LPWM 6    // Drive Motor B
#define MOT_B_RPWM 9
#define MOT_W_LPWM 10   // Weapon Motor
#define MOT_W_RPWM 11

#define currentW A1     // Current Sensor for weapon
#define WeaponSW A2     // Weapon enable switch 
#define enable A3       // Drive enable pin
#define enableW 12      // Weapon enable pin

const float sensitivity = 0.020;   // V/A of sensor
const float Voffset = 2.5;         // Zero-current output
const int CURRENT_LIMIT = 80;      // Weapon protection limit 

const int CH1_MIN = 982, CH1_MAX = 1947;
const int CH2_MIN = 978, CH2_MAX = 1944;
const int CH3_MIN = 980, CH3_MAX = 1960;

int SWITCH;
int ch3Value;
int ch2Value;
int CH1Value;
bool INVERT;

int spdA, spdB, spdW;  // motor speed commands

// Weapon state machine
static int weaponState = 0;       // 0=normal, 1=tripped, 2=ramping
static unsigned long tripTime = 0;
static int rampSpd = 0;

//FIXED readRaw()
int readRaw(int channelInput) {
  // use pulseInLong for more reliable reading on pins like 13 and 4
  unsigned long val = pulseInLong(channelInput, HIGH, 25000UL);  
  return (int)val;
}

int pwmToSpeed(int us, int minVal, int maxVal) {
  if (us < 100) return 0;               // failsafe
  us = constrain(us, minVal, maxVal);
  int val = map(us, minVal, maxVal, -255, 255); 
  if (abs(val) < 20) val = 0;           // deadband
  return val;
}

int pwmToWeapon(int us, int minVal, int maxVal) {
  if (us < 100) return 0; // failsafe
  us = constrain(us, minVal, maxVal);
  return map(us, minVal, maxVal, 0, 255); // forward only, invert handled later
}

bool readCH5(byte channelInput, bool defaultValue) {
  int ch = readRaw(channelInput);
  if (ch < 100) return defaultValue;  
  return (ch > 1500);
}

int readCH6(byte channelInput, int defaultValue) {
  int ch = readRaw(channelInput);
  if (ch < 100) return defaultValue;
  if (ch < 1300) return -1;
  if (ch > 1700) return 1;
  return 0;
}

void setMotor(int forward, int backward, int LPWM, int RPWM) {
  analogWrite(LPWM, forward);
  analogWrite(RPWM, backward);
}

void setup() {
  Serial.begin(115200);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);

  pinMode(MOT_A_LPWM, OUTPUT);
  pinMode(MOT_A_RPWM, OUTPUT);
  pinMode(MOT_B_LPWM, OUTPUT);
  pinMode(MOT_B_RPWM, OUTPUT);
  pinMode(MOT_W_LPWM, OUTPUT);
  pinMode(MOT_W_RPWM, OUTPUT);

  pinMode(enable, OUTPUT);
  pinMode(enableW, OUTPUT);
  pinMode(WeaponSW, OUTPUT);

  digitalWrite(enable, LOW);   // enable drive by default
  digitalWrite(enableW, LOW);   // weapon enabled
  digitalWrite(WeaponSW, LOW);  // weapon off
}

void loop() {
  //Read current
  int adcValue = analogRead(currentW);
  float Vout = (adcValue / 1023.0) * 5.0;
  float Wcurrent = (Vout - Voffset) / sensitivity;

  //Read RX inputs
  CH1Value = readRaw(CH1);
  ch2Value = readRaw(CH2);
  ch3Value = readRaw(CH3);
  INVERT   = readCH5(CH5, 0);
  SWITCH   = readCH6(CH6, 0);

  //Failsafe check
  if (CH1Value < 100 && ch2Value < 100 && ch3Value < 100) {
    setMotor(0,0,MOT_A_LPWM,MOT_A_RPWM);
    setMotor(0,0,MOT_B_LPWM,MOT_B_RPWM);
    setMotor(0,0,MOT_W_LPWM,MOT_W_RPWM);
    Serial.println("FAILSAFE: No RX signal, all motors stopped!");
    return;
  }

  //Map to speeds
  if (!INVERT) {
    spdA = pwmToSpeed(CH1Value, CH1_MIN, CH1_MAX);
    spdB = pwmToSpeed(ch2Value, CH2_MIN, CH2_MAX);
    spdW = pwmToWeapon(ch3Value, CH3_MIN, CH3_MAX);   // forward
  } else {
    spdA = -pwmToSpeed(ch2Value, CH2_MIN, CH2_MAX);
    spdB = -pwmToSpeed(CH1Value, CH1_MIN, CH1_MAX);
    spdW = -pwmToWeapon(ch3Value, CH3_MIN, CH3_MAX);  // invert weapon
  }

  //Drive motors
  if (spdA > 0) setMotor(spdA, 0, MOT_A_LPWM, MOT_A_RPWM);
  else if (spdA < 0) setMotor(0, -spdA, MOT_A_LPWM, MOT_A_RPWM);
  else setMotor(0, 0, MOT_A_LPWM, MOT_A_RPWM);

  if (spdB > 0) setMotor(spdB, 0, MOT_B_LPWM, MOT_B_RPWM);
  else if (spdB < 0) setMotor(0, -spdB, MOT_B_LPWM, MOT_B_RPWM);
  else setMotor(0, 0, MOT_B_LPWM, MOT_B_RPWM);

  //Weapon control
  if (SWITCH < 0) {
    // Weapon OFF
    digitalWrite(WeaponSW, LOW);
    setMotor(0, 0, MOT_W_LPWM, MOT_W_RPWM);
    weaponState = 0;
    rampSpd = 0;
  }

  else if (SWITCH == 0) {
    // Protected mode (ramp + current limit)
    digitalWrite(WeaponSW, HIGH);

    switch (weaponState) {
      case 0:  // Normal
        if (Wcurrent > CURRENT_LIMIT) {
          digitalWrite(enableW, HIGH);   // disable
          tripTime = millis();
          weaponState = 1;
          rampSpd = 0;
        } else {
          if (spdW > 0) setMotor(spdW, 0, MOT_W_LPWM, MOT_W_RPWM);
          else if (spdW < 0) setMotor(0, -spdW, MOT_W_LPWM, MOT_W_RPWM);
          else setMotor(0, 0, MOT_W_LPWM, MOT_W_RPWM);
        }
        break;

      case 1:  // Tripped
        if (millis() - tripTime > 1000) {  // wait 1s
          digitalWrite(enableW, LOW);      // re-enable
          weaponState = 2;
          rampSpd = 0;
        }
        break;

      case 2:  // Ramping
        if (rampSpd < abs(spdW)) rampSpd += 5;
        else if (rampSpd > abs(spdW)) rampSpd = abs(spdW);

        if (spdW > 0) setMotor(rampSpd, 0, MOT_W_LPWM, MOT_W_RPWM);
        else if (spdW < 0) setMotor(0, rampSpd, MOT_W_LPWM, MOT_W_RPWM);
        else setMotor(0, 0, MOT_W_LPWM, MOT_W_RPWM);

        if (Wcurrent > CURRENT_LIMIT) {
          digitalWrite(enableW, HIGH);
          tripTime = millis();
          weaponState = 1;
          rampSpd = 0;
        }
        if (spdW == 0) weaponState = 0;
        break;
    }
  }

  else if (SWITCH == 1) {
    // Direct mode (no protection)
    digitalWrite(WeaponSW, HIGH);
    if (spdW > 0) setMotor(spdW, 0, MOT_W_LPWM, MOT_W_RPWM);
    else if (spdW < 0) setMotor(0, -spdW, MOT_W_LPWM, MOT_W_RPWM);
    else setMotor(0, 0, MOT_W_LPWM, MOT_W_RPWM);
    weaponState = 0;
    rampSpd = 0;
  }

  //Debugging Output
  Serial.print("CH1 raw: "); Serial.print(CH1Value);
  Serial.print(" -> spdA: "); Serial.print(spdA);
  Serial.print(" | CH2 raw: "); Serial.print(ch2Value);
  Serial.print(" -> spdB: "); Serial.print(spdB);
  Serial.print(" | CH3 raw: "); Serial.print(ch3Value);
  Serial.print(" -> spdW: "); Serial.print(spdW);
  Serial.print(" | CH5 (INVERT): "); Serial.print(INVERT);
  Serial.print(" | CH6 (SWITCH): "); Serial.print(SWITCH);
  Serial.print(" | Wcurrent: "); Serial.print(Wcurrent);
  Serial.print(" | WeaponState: "); Serial.println(weaponState);
}
