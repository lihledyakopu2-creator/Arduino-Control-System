#include <LiquidCrystal_I2C.h>

#define ADC10         A0
#define TEMP10        A1
#define PWM10         9
#define SELECT_BTN10  2
#define LED_OK10      4
#define LED_MID10     5
#define LED_HIGH10    8
#define PING_T10      6
#define PING_E10      7
#define MOTOR10       10

const long LOG_MS10 = 1000;
unsigned long lastLog10 = 0;
int s10=0,m10=0,h10=0;
float vOut10 = 0.0;

const long PING_MS10 = 100;
unsigned long lastPing10 = 0;

unsigned long lastTemp10 = 0;
const float KD10 = 0.0008895 / 10.0;
const long TEMP_MS10 = 200;
bool flash10 = false;
float tc10 = 0.0, tf10 = 0.0;

const float KP10 = 0.2183688;
const float KI10 = 18.6724;
const float KDCONST10 = KD10;

float P10=0.0,I10=0.0,D10=0.0;
float setV10 = 2.0;
const long PID_MS10 = 10;
const float dt10 = 0.01;
float prevErr10 = 0.0;
unsigned long lastMotor10 = 0, lastPid10 = 0;
int pwm10 = 0;

bool prevBtn10 = HIGH, curBtn10;
unsigned long btnMs10 = 0;
const unsigned long DB10 = 50;
int scr10 = 0;
float duty10 = 0.0;

LiquidCrystal_I2C lcd10(0x27,16,2);

int btnCheck10(unsigned long now){
  curBtn10 = digitalRead(SELECT_BTN10);
  if(prevBtn10 == HIGH && curBtn10 == LOW){
    if(now - btnMs10 >= DB10){
      btnMs10 = now;
      return scr10++;
    }
  }
  prevBtn10 = curBtn10;
}

void show10(int mode){
  if(mode == 1){
    lcd10.clear();
    lcd10.setCursor(0,0);
    lcd10.print("Dist:");
    lcd10.setCursor(0,1);
    lcd10.print("Set:");
    lcd10.print(setV10); lcd10.print("V");
  } else if(mode == 2){
    lcd10.clear();
    lcd10.setCursor(0,0);
    lcd10.print("TempC:");
    lcd10.print(tc10);
    lcd10.setCursor(0,1);
    lcd10.print("F:");
    lcd10.print(tf10);
  } else if(mode == 3){
    lcd10.clear();
    lcd10.setCursor(0,0);
    lcd10.print("P:");
    lcd10.print(P10);
    lcd10.setCursor(0,1);
    lcd10.print("I:");
    lcd10.print(I10);
  } else if(mode == 4){
    lcd10.clear();
    lcd10.setCursor(0,0);
    lcd10.print("D:");
    lcd10.print(D10);
  } else {
    lcd10.clear();
    lcd10.setCursor(0,0);
    lcd10.print("Duty:");
    lcd10.setCursor(0,1);
    lcd10.print(duty10); lcd10.print("%");
    if(scr10 > 4) scr10 = 0;
  }
}

void setup(){
  pinMode(PWM10, OUTPUT);
  pinMode(LED_OK10, OUTPUT);
  pinMode(LED_MID10, OUTPUT);
  pinMode(LED_HIGH10, OUTPUT);
  pinMode(MOTOR10, OUTPUT);
  pinMode(PING_T10, OUTPUT);
  pinMode(PING_E10, INPUT);
  pinMode(SELECT_BTN10, INPUT_PULLUP);

  TCCR1A = (1<<WGM11)|(1<<WGM10)|(1<<COM1A1);
  TCCR1B = (1<<WGM12)|(1<<CS11);

  Serial.begin(9600);
  Serial.println("Timestamp_ms,Uptime,Distance_cm,TempC,MotorDuty,SetV,OutV");

  lcd10.init();
  lcd10.backlight();
  lcd10.setCursor(0,0);
  lcd10.print("PID Control");
  lcd10.setCursor(0,1);
  lcd10.print("Temp & Motor");
  delay(200);
}

void loop(){
  unsigned long now = millis();

  if(now - lastLog10 >= LOG_MS10){
    lastLog10 = now;
    if(++s10 == 60){ s10 = 0; if(++m10 == 60){ m10 = 0; h10++; } }
    Serial.print(now); Serial.print(",");
    Serial.print(h10); Serial.print(m10); Serial.print(s10); Serial.print(",");
    Serial.print("0.0,");
    Serial.print(tc10); Serial.print(",");
    Serial.print(duty10); Serial.print(",");
    Serial.print(setV10); Serial.print(",");
    Serial.println(vOut10);
  }

  if(now - lastTemp10 >= TEMP_MS10){
    lastTemp10 = now;
    float raw = analogRead(TEMP10);
    float v = (raw / 1023.0) * 5.0;
    tc10 = (v - 0.5) * 100.0;
    tf10 = ((9.0/5.0) * tc10) + 32.0;

    digitalWrite(LED_OK10, tc10 < 28.0);
    digitalWrite(LED_MID10, tc10 > 28.0 && tc10 < 32.0);
    if(tc10 > 32.0 && tc10 < 35.0) digitalWrite(LED_HIGH10, HIGH);
    else if(tc10 > 35.0){ flash10 = !flash10; digitalWrite(LED_HIGH10, flash10); }
    else digitalWrite(LED_HIGH10, LOW);
  }

  if(now - lastPid10 >= PID_MS10){
    lastPid10 = now;
    setV10 = constrain(setV10, 0.0, 5.0);
    float adc = analogRead(ADC10);
    float setraw = (setV10 * 1023.0) / 5.0;
    float err = setraw - adc;

    P10 = KP10 * err;
    I10 += KI10 * dt10 * err;
    I10 = constrain(I10, 0.0, 1023.0);
    D10 = -KDCONST10 * (err - prevErr10) / dt10;

    float pout = P10 + I10 + D10;
    pout = constrain(pout, 0.0, 1023.0);
    prevErr10 = err;

    OCR1A = pout / 4.0;
    vOut10 = (pout / 1023.0) * 5.0;
    duty10 = (pout / 1023.0) * 100.0;
    pwm10 = OCR1A;
  }

  if(now - lastPing10 >= PING_MS10){
    lastPing10 = now;
    digitalWrite(PING_T10, LOW); delayMicroseconds(2);
    digitalWrite(PING_T10, HIGH); delayMicroseconds(10);
    digitalWrite(PING_T10, LOW);

    unsigned long pulse = pulseIn(PING_E10, HIGH, 30000);
    if(pulse > 0){
      float dist = pulse * 0.0343 / 2.0;
      if(dist <= 30.0){
        float m = constrain(dist, 0.0, 30.0);
        setV10 = (m / 30.0) * 3.0;
      }
    }
    show10(btnCheck10(now));
  }

  if(now - lastMotor10 >= PING_MS10){
    lastMotor10 = now;
    analogWrite(MOTOR10, pwm10);
  }
}