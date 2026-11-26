#include <SoftSPIB.h>
#include <LiquidCrystal_AIP31068_I2C.h>
#include <LiquidCrystal_AIP31068_SPI.h>

#define AN_ADC   A0
#define AN_TEMP  A1
#define DAC_OUTP 9
#define SEL_BTN  2
#define LED_G_1  4
#define LED_Y_1  5
#define TRIG_S   6
#define ECHO_S   7
#define LED_R_1  8
#define MOTORPWM 10

const long LOG_PERIOD = 1000;
unsigned long log_prev = 0;
int seconds_r=0, minutes_r=0, hours_r=0;
float outV_now = 0.0;

const long SONAR_PERIOD = 100;
unsigned long sonar_prev = 0;
unsigned long echo_s_mic = 0;
unsigned long echo_e_mic = 0;

float cm_val = 0.0;
unsigned long echo_len = 0;
float dist_val_f = 0.0;

unsigned long tmp_last = 0;
const float KDVAL = 0.0008895 / 10.0;
const long TEMP_PERIOD = 200;
bool red_blink = false;
float temp_celsius = 0.0;
float temp_fahrenheit = 0.0;

const float KP_CONST2 = 0.2183688;
const float KI_CONST2 = 18.6724;
const float KD_CONST2 = KDVAL;

float Ppart = 0.0;
float Ipart = 0.0;
float Dpart = 0.0;

float setpt = 2.0;
const long PID_PERIOD = 10;
const float STEP_DT2 = 0.01;
float prev_err = 0.0;
unsigned long motor_last = 0;
unsigned long pid_last = 0;
int motor_out_pwm = 0;

bool btn_prev_state = HIGH;
bool btn_curr_state;
unsigned long btn_change_time = 0;
const unsigned long BTN_DELAY_MS = 50;
int menu = 0;
float duty_pct_now = 0.0;

LiquidCrystal_AIP31068_I2C lcdx(62,16,2);

void setup(){
  pinMode(DAC_OUTP, OUTPUT);
  pinMode(LED_G_1, OUTPUT);
  pinMode(LED_Y_1, OUTPUT);
  pinMode(LED_R_1, OUTPUT);
  pinMode(MOTORPWM, OUTPUT);
  pinMode(TRIG_S, OUTPUT);
  pinMode(ECHO_S, INPUT);
  pinMode(SEL_BTN, INPUT_PULLUP);

  TCCR1A = (1<<WGM11)|(1<<WGM10)|(1<<COM1A1);
  TCCR1B = (1<<WGM12)|(1<<CS11);

  Serial.begin(9600);
  Serial.println("Timestamp_ms,Uptime_HHMMSS,Distance_cm,Temperature_C,MotorDuty_pct,PID_Setpoint_V,PID_Output_V");
  lcdx.init();
  lcdx.setCursor(0,0); lcdx.print("PID Control");
  lcdx.setCursor(0,1); lcdx.print("TEMP + MOTOR");
  delay(200);
}

int menuButton(unsigned long tnow){
  btn_curr_state = digitalRead(SEL_BTN);
  if(btn_prev_state == HIGH && btn_curr_state == LOW){
    if(tnow - btn_change_time >= BTN_DELAY_MS){
      btn_change_time = tnow;
      return menu++;
      if(menu > 4) menu = 0;
    }
  }
  btn_prev_state = btn_curr_state;
  return menu;
}

void putDisplay(int idx){
  if(idx == 1){
    lcdx.clear();
    lcdx.setCursor(0,0); lcdx.print("Distance: "); lcdx.print(dist_val_f); lcdx.print("cm");
    lcdx.setCursor(0,1); lcdx.print("Setpoint: "); lcdx.print(setpt); lcdx.print("V");
  } else if(idx == 2){
    lcdx.clear();
    lcdx.setCursor(0,0); lcdx.print("Temperature");
    lcdx.setCursor(0,1); lcdx.print(temp_celsius); lcdx.print(" °C : "); lcdx.print(temp_fahrenheit); lcdx.print(" °F");
  } else if(idx == 3){
    lcdx.clear();
    lcdx.setCursor(0,0); lcdx.print("PID");
    lcdx.setCursor(0,1); lcdx.print("kp: "); lcdx.print(Ppart); lcdx.print(" ki: "); lcdx.print(Ipart);
  } else if(idx == 4){
    lcdx.clear();
    lcdx.setCursor(0,0); lcdx.print("PID");
    lcdx.setCursor(0,1); lcdx.print("kd: "); lcdx.print(Dpart);
  } else {
    lcdx.clear();
    lcdx.setCursor(0,0); lcdx.print("Duty Cycle");
    lcdx.setCursor(0,1); lcdx.print(duty_pct_now); lcdx.print("%");
    idx = 0;
  }
}

void loop(){
  unsigned long tnow = millis();

  if(tnow - log_prev >= LOG_PERIOD){
    log_prev = tnow;
    seconds_r++;
    if(seconds_r == 60){ minutes_r++; seconds_r = 0; if(minutes_r == 60){ hours_r++; minutes_r = 0; } }
    Serial.print(tnow); Serial.print(","); Serial.print(hours_r); Serial.print(minutes_r); Serial.print(seconds_r); Serial.print(",");
    Serial.print(cm_val); Serial.print(","); Serial.print(temp_celsius); Serial.print(","); Serial.print(duty_pct_now); Serial.print(","); Serial.print(setpt); Serial.print(","); Serial.println(outV_now);
  }

  if(tnow - tmp_last >= TEMP_PERIOD){
    tmp_last = tnow;
    float rawtemp = analogRead(AN_TEMP);
    float vtemp = (rawtemp / 1023.0) * 5.0;
    temp_celsius = (vtemp - 0.5) * 100.0;
    temp_fahrenheit = ((9.0/5.0) * temp_celsius) + 32.0;

    if(temp_celsius < 28.0){
      digitalWrite(LED_G_1, HIGH); digitalWrite(LED_Y_1, LOW); digitalWrite(LED_R_1, LOW);
    } else if(temp_celsius > 28.0 && temp_celsius < 32.0){
      digitalWrite(LED_G_1, LOW); digitalWrite(LED_Y_1, HIGH); digitalWrite(LED_R_1, LOW);
    } else if(temp_celsius > 32.0 && temp_celsius < 35.0){
      digitalWrite(LED_G_1, LOW); digitalWrite(LED_Y_1, LOW); digitalWrite(LED_R_1, HIGH);
    } else if(temp_celsius > 35.0){
      red_blink = !red_blink;
      digitalWrite(LED_G_1, LOW); digitalWrite(LED_Y_1, LOW); digitalWrite(LED_R_1, red_blink);
    }
  }

  if(tnow - pid_last >= PID_PERIOD){
    pid_last = tnow;
    setpt = constrain(setpt, 0.0, 5.0);
    float adcV = analogRead(AN_ADC);
    float set_scaled = (setpt * 1023.0)/5.0;
    float err = set_scaled - adcV;

    Ppart = KP_CONST2 * err;
    Ipart += KI_CONST2 * STEP_DT2 * err;
    Ipart = constrain(Ipart, 0.0, 1023.0);
    Dpart = -KD_CONST2 * (err - prev_err) / STEP_DT2;

    float outpid = Ppart + Ipart + Dpart;
    outpid = constrain(outpid, 0.0, 1023.0);

    prev_err = err;
    OCR1A = outpid / 4;
    outV_now = (outpid / 1023.0) * 5.0;
    duty_pct_now = (outpid / 1023.0) * 100.0;
    motor_out_pwm = OCR1A;
  }

  if(tnow - sonar_prev >= SONAR_PERIOD){
    sonar_prev = tnow;
    digitalWrite(TRIG_S, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_S, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_S, LOW);
    echo_len = pulseIn(ECHO_S, HIGH, 30000);
    dist_val_f = (echo_len * 0.034 / 2.0);
    setpt = constrain((dist_val_f / 30.0) * 3.0, 0.0, 3.0);

    putDisplay(menuButton(tnow));
  }

  if(tnow - motor_last >= SONAR_PERIOD){
    motor_last = tnow;
    analogWrite(MOTORPWM, motor_out_pwm);
  }
}