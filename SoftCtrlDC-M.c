/*
Core control software for a DC motor driver
Ver: 1.0
Autor: Jose Niembro
Fecha: 11/01/2023

Closed-loop speed control for a DC motor: 38ZY63-1230/36JX30KG8D20.
Controllers: PID (Kp, Ki, Kd), Fuzzy logic controller for proportional gain with Integral (FP+I - FKi = 15.0), PID (Ki = 1.2, Kp = 0.085, Kd = 0.0015).
Encoder: E4P / 360 CPR.
*/

// Pins configuration
const byte IN1 = 7; // IN1 for H-Bridge
const byte IN2 = 8; // IN2 for H-Bridge
const byte PWM = 5; // PWM output
const byte ChA = 2; // Encoder ChA
const byte ChB = 4; // Encoder ChB

// Current sensor setting7 config
int Iter = 0, Counts = 75; // Iterations to calculate an Electrical Current average, 100 samples to average
double Sens = 0.125; // Sensor Sensibility
double MsVoltageACS = 0.0;

// Init output for PWM
int Speed = 0;
int Output = 0;

// MCU timer setting
int Tmr_Ctr = 0;

// General settings
double MtrCurrentSpeed = 0.0;
double Err = 0.0;
double Voltage_Ref = 0.0;
double PWM_Ref = 0.0;
double RefSpeed = 0.0;

double Voltage_fbk = 0.0;
double PWM_fbk = 0.0;

unsigned long t;

// PID controller gains set up
double Kp = 0.09, Ki = 1.1, FKi = 0.9, Kd = 0.0015;

// Init for loop parameters
double PastErr = 0.0;
double SumErr = 0.0;
double IntegralTerm = 0.0;

// Integers
int Encoder = 0;

// GEneral function for MCU setting
void setup() {

  // "Set up" reading E4P / 360 CPR
  pinMode (ChA, INPUT_PULLUP);
  pinMode (ChB, INPUT_PULLUP);

  // H-bridge config
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (PWM, OUTPUT);

  // IN1 HIGH & IN2 LOW is CCW
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);

  attachInterrupt(digitalPinToInterrupt(ChA), Detect_ChA, RISING);

  // Serial comm set up
  Serial.begin (9600);

  noInterrupts (); // No interrputs setting

  // ATMega328P config
  TCCR1A = 0;
  TCCR1B = 0;
  Tmr_Ctr = 64286; // (0.02 seg)

  TCNT1 = Tmr_Ctr;
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts ();

}

// Controller selector
char ctrl = 'p'; // 'p' is PID, 'f' is Fuzzy, 'a' is PI

double Out = 0.0;
double Proportional = 0.0;
double Derivada = 0.0;

double Integral = 0.0;
double PWM2 = 0.0, Otp = 0.0;
double PIDPastErr = 0.0;

double FPWM = 0.0, Fuzzy = 0.0;
double f1 = 0.0, f2 = 0.0, f3 = 0.0;
double sug1 = 0.0, sug2 = 0.0, sug3 = 0.0;
double FIntegral = 0.0;

// Loop execution
void loop() {

  // Lectura del puerto serial
  if (Serial.available() > 0) {
      String data = Serial.readString();
      Speed = data.toInt();
      }

      // Current reading
      // MsVoltageACS = GetMeasuredVoltage (Counts);
      // double Current = (((MsVoltageACS - 2.5) / Sens) + 0.065) * 1.0;

      // Cast from int to double
      RefSpeed = (double)Speed;

      // Voltage calc for reference speed
      Voltage_Ref = Calc_Voltage (RefSpeed);

      // PWM calc depending on voltage reference
      //PWM_Ref = Calc_PWM (Voltage_Ref);

      // Current DC motor speed calc
      double GearCurrentSpeed = MtrCurrentSpeed / 50.89;

      // Current DC motor speed calc in volts
      Voltage_fbk = Calc_Voltage (GearCurrentSpeed);

      // PWM feedback cacl
      //PWM_fbk = Calc_PWM (Voltage_fbk);

      // Error calc
      double VErr = Voltage_Ref - Voltage_fbk;

      // Control selector
      switch (ctrl) {
        case 'p':

          Integral += Ki * VErr;

          Proportional = VErr * Kp;
          Derivada = (VErr - PastErr) * Kd;

          PWM2 = Integral + (VErr * Kp) + ((VErr - PastErr) * Kd);

          if (PWM2 < 19.1 & PWM2 > 0){
            Otp = PWM2;
          }
          else {
            if (PWM2 > 19.1) {
              Otp = 19.1;
            }
            else {
              Otp = 0.0;
            }
          }

          PIDPastErr = VErr;

          Serial.print ("Integral: ");
          Serial.print (Integral);
          Serial.print (", ");
          Serial.print ("Proportional: ");
          Serial.print (Proportional);
          Serial.print (", ");
          Serial.print ("Derivative: ");
          Serial.print (Derivada);
          Serial.print (", ");
          Serial.print ("PID_u: ");
          Serial.print (Otp);
          Serial.print (", ");

          Out = Calc_PWM (Otp);
          break;

        case 'f':

          if (VErr <= -10.0) {
            f1 = 1.0;
            f2 = 0.0;
            f3 = 0.0;
          }

          else if (VErr > -10.0 & VErr < 0) {
            f1 = -0.1 * VErr;
            f2 = (0.1 * VErr) + 1;
            f3 = 0.0;
          }

          else if (VErr >= 0.0 & VErr < 10.0) {
            f1 = 0.0;
            f2 = (-0.1 * VErr) + 1;
            f3 = 0.1 * VErr;
          }

          else {
            f1 = 0.0;
            f2 = 0.0;
            f3 = 1.0;
          }

          sug1 = -0.1 * VErr;
          sug3 = 0.1 * VErr;

          Serial.print ("f1: ");
          Serial.print (f1);
          Serial.print (", ");
          Serial.print ("f2: ");
          Serial.print (f2);
          Serial.print (", ");
          Serial.print ("f3: ");
          Serial.print (f3);
          Serial.print (", ");

          Fuzzy = ((sug1 * f1) + (sug2 * f2) + (sug3 * f3)) / (f1 + f2 + f3);

          Serial.print ("Fuzzy: ");
          Serial.print (Fuzzy);
          Serial.print (", ");

          FIntegral += FKi * VErr;

          Serial.print ("FIntegral: ");
          Serial.print (FIntegral);
          Serial.print (", ");

          FPWM = FIntegral + Fuzzy;

          Serial.print ("FP+I_u: ");
          Serial.print (FPWM);
          Serial.print (", ");

          if (FPWM < 19.1 & FPWM > 0){
            Otp = FPWM;
          }
          else {
            if (FPWM > 19.1) {
              Otp = 19.1;
            }
            else {
              Otp = 0.0;
            }
          }

          Out = Calc_PWM (Otp);
          break;
      }

      //double Out = Calc_PWM (Output);

      // Casting PWM value
      int OPWM = (int) Out;

      if (OPWM > 255) {
        analogWrite (PWM, 255);
      }
      else if (OPWM < 255 & OPWM > 0) {
        analogWrite (PWM, OPWM);
      }
      else {
          analogWrite (PWM, 0);
      }

      //analogWrite (PWM, 50);

      t = millis ();
      Serial.print ("Time: ");
      Serial.print (t);
      Serial.print (", ");
      Serial.print ("Referencia: ");
      Serial.print (Speed);
      Serial.print (", ");
      //Serial.print ("RefSpeed: ");
      //Serial.print (RefSpeed);
      //Serial.print (", ");
      Serial.print ("Voltage: ");
      Serial.print (Voltage_Ref);
      Serial.print (", ");
      //Serial.print ("PWMRef: ");
      //Serial.print (PWM_Ref);
      //Serial.print (", ");
      Serial.print ("GearBox_Speed: ");
      Serial.print (GearCurrentSpeed, 2);
      Serial.print(",");
      Serial.print ("Voltage_fbk: ");
      Serial.print (Voltage_fbk);
      Serial.print (", ");
      //Serial.print ("PWM_fbk: ");
      //Serial.print (PWM_fbk);
      //Serial.print (", ");
      Serial.print ("Err_Voltaje: ");
      Serial.print (VErr);
      Serial.print (", ");
      //Serial.print ("Error_PWM: ");
      //Serial.print (Err);
      //Serial.print (", ");
      Serial.print ("OPWM: ");
      Serial.println (OPWM);

      // "Set up" for PWM value display

  delay (20);

}

double Calc_Voltage (double RefSpeed) {

  double Ref = 0.0;
  Ref = RefSpeed * 0.216;

  return (Ref);
}

double Calc_PWM (double Voltage_Ref) {

  double refPWM = 0.0;
  float VR1 = Voltage_Ref * Voltage_Ref * Voltage_Ref;
  float VR2 = Voltage_Ref * Voltage_Ref;

  if (Voltage_Ref <= 0.5) {
    refPWM = 0.0;
  }
  else if (Voltage_Ref > 0.5 & Voltage_Ref < 19.1) {
    refPWM = (0.0192 * VR1) - (0.281 * VR2) + (4.51 * Voltage_Ref) + 36.96;
  }
  else if (Voltage_Ref >= 19.1) {
    refPWM = 255;
  }

  return (refPWM);
}

/*
double GetMeasuredVoltage (int Counts) {
    double ACS712_Voltage = 0.0;

    for (int i=0; i < Counts; i++) {
        ACS712_Voltage = ACS712_Voltage + analogRead (A1) * (5.0 / 1023);
      }

      ACS712_Voltage = ACS712_Voltage / Counts;
    return (ACS712_Voltage);
  }
*/

// Detecting positive flange for ChA
void Detect_ChA () {
    Encoder += 1;
  }

// Rutina de interrupción
ISR(TIMER1_OVF_vect) {
    TCNT1 = Tmr_Ctr;
    MtrCurrentSpeed = (60 * (Encoder / 360.0) / 0.02);
    Encoder = 0;
  }

/*
// Current sensor reading
double GetCurrent (int Iter) {
    double Volts;
    double SumCurrent = 0.0;

    for (int i=0; i < Iter; i++) {
        Volts = analogRead (A1) * 5.0 / 1024.0;
        SumCurrent += (Volts - 2.5) / Sens;
      }

    return (SumCurrent / Iter);
  }
  */
