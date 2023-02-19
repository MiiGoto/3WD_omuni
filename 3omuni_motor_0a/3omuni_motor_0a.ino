#include <Wire.h>
#include <stdlib.h>
#define SLAVE_ADDRESS 0x0a
#define Kp 1
#define Ki 11
#define Kd 0.8
// Ki:Kd=4:1~6:1 がよい（らしい）
#define f_RATE 0.5
#define MAX_RPS 100.0
static char cmd = 0;
static float speedCommand = 0, errorTimer = 0;
static int num = 0, errorFlag = 0;
static unsigned long loopTime;
const int pinA = 2; // A相 割り込み0
const int pinB = 3; // B相 割り込み1
const int motorf = 9;
const int motorb = 10;
volatile int enc_count = 0;
float P, I, D;
static float preP = 0;

void setup(void)
{
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);   // I2Cパスにスレーブとして接続
  Wire.onReceive(receive_i2c); // マスタからデータが送られてきたときのハンドラを設定
  // Wire.onRequest(send_i2c);// マスタからリクエストが来たときのハンドラを設定
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(motorf, OUTPUT);
  pinMode(motorb, OUTPUT);
  pinMode(13, OUTPUT);
  attachInterrupt(0, enc_changedPinA, CHANGE); // pinAの信号変化に合わせて割り込み処理
  attachInterrupt(1, enc_changedPinB, CHANGE); // pinBの信号変化に合わせて割り込み処理
}

void loop()
{
  unsigned long now;
  float dt = 0;
  static float motorPower = 0;
  static float rps, last_rps;
  now = micros();
  dt = (now - loopTime) / 1000;
  dt = dt / 1000;
  if (errorTimer >= 10)
  {
    speedCommand = 0;
    errorTimer = 10;
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
  // Serial.println(errorTimer);
  if (dt > 0.05)
  {
    errorTimer++;
    // Serial.println(enc_count);
    rps = f_RATE * last_rps - (1 - f_RATE) * (float)enc_count / (2048 * dt);
    enc_count = 0;
    last_rps = rps;
    motorPower = PID(dt, rps);
    motorPower = constrain(motorPower, -245, 245);
    // motorPower=60;
    if (errorTimer >= 10)
    {
      speedCommand = 0;
      motorPower = 0;
      errorTimer = 10;
      digitalWrite(13, HIGH);
    }
    else
    {
      digitalWrite(13, LOW);
    }
    if (motorPower < 0)
    {
      analogWrite(motorf, abs(motorPower));
      analogWrite(motorb, 0);
    }
    else
    {
      analogWrite(motorf, 0);
      analogWrite(motorb, abs(motorPower));
    }
    loopTime = now;
    // Serial.print(dt);
    // Serial.print("   ");
    // Serial.println(rps);
    // Serial.print("  rot/sec  motor power = ");
    // Serial.println(motorPower);
  }
}

// PID制御用の関数
inline float PID(unsigned int dt, float rps)
{
  static float P = 0, I = 0, D = 0, preP = 0;
  speedCommand = cmd * (MAX_RPS / 127);
  P = speedCommand - rps;
  // if(P<0.5) return 0;
  I += (P + preP) / 2 * dt;
  // I+=(P+preP)/2;
  I = constrain(I, -245, 245);
  D = P - preP;
  preP = P;
  // Serial.println(P);
  // Serial.println(speedCommand);
  return Kp * P + Ki * I + Kd * D;
}

// 送信処理 今回は使用しない
/*
void send_i2c(){
  byte snd_msg[3]={};
  detachInterrupt(0);
  detachInterrupt(1);
  Serial.print("  ");
  Serial.print(enc_count);
  Serial.print("  ");
  if(enc_count<0){
    snd_msg[2]=255;
    snd_msg[1]=1;//(char)-ceil(enc_count/100);
    snd_msg[0]=1;//(char)(-enc_count-snd_msg[1]);
  }else{
    snd_msg[2]=1;
    snd_msg[1]=0;//(char)-floor(enc_count/100);
    snd_msg[0]=0;//(char)(enc_count-snd_msg[1]);
  }
  enc_count=0;
  for(int i=0;i<3;i++){
    Wire.write(snd_msg[i]);
    //Serial.print(snd_msg[i]);
  }
  Serial.println("");
  attachInterrupt(0, enc_changedPinA, CHANGE);
  attachInterrupt(1, enc_changedPinB, CHANGE);
}
 */
// 受信処理
// n: 送られてきたデータのbyte数(cmdも数に含まれていることに注意)
void receive_i2c(int n)
{
  if (Wire.available())
  {
    cmd = (char)Wire.read();
    errorTimer = 0;
    // Serial.println((int)cmd);
  }
}

void count_inc()
{
  enc_count++;
}
void count_dec()
{
  enc_count--;
}

// pinAの割り込み処理
void enc_changedPinA()
{
  if (digitalRead(pinA))
  {
    if (digitalRead(pinB))
      count_dec();
    else
      count_inc();
  }
  else
  {
    if (digitalRead(pinB))
      count_inc();
    else
      count_dec();
  }
}
void enc_changedPinB()
{
  if (digitalRead(pinB))
  {
    if (digitalRead(pinA))
      count_inc();
    else
      count_dec();
  }
  else
  {
    if (digitalRead(pinA))
      count_dec();
    else
      count_inc();
  }
}
