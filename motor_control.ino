#include <Encoder.h>


//For mouse 1, M1 is Left Motor, M2 is Right Motor
const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

//Interrupt Pins
const unsigned int M1_IN_1 = 2; //pin numbers for the motors
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;


//Analog Input
const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

//Variable for encoders
float m1PrevRead = 0;
float m2PrevRead = 0;
float m1CurrRead = 0;
float m2CurrRead = 0;
float m1AdjRead = 0;
float m2AdjRead = 0;

float m1PWM = 255;
float m2PWM = 255;

float prevTime = 0;
float currTime = 0;
float delTime = 0;

float delM1Pos = 0;
float delM2Pos = 0;

/*  Rotary Encoder Values - universal, close enough across all robots */
const float revolution_ticks = 360; //  360 ticks/wheel rotation
const float revolution_distance = 10.5; //  10.5 cm per rotation
const float dist_per_tick = revolution_distance / revolution_ticks; //cm equivalent of the rotary encoder ticks
const float junction_ticks = 15;  //  514.5 ticks per 15 cm

//ripped these values from fall2021team2

float targetVel = 0.2;
float curVel1, curVel2;
float targetPos = 0; //initial position of mouse is 0

//u = Kp*e + Ki * integral(e) + Kd * de/dt

float m1PrevError, m2PrevError = 0;
float m1Kp, m2Kp = 3;
float m1Ki, m2Ki = 1.2;
float m1Kd, m2Kd = 0;

float m1P, m2P, m1I, m2I, m1D, m2D;



const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120; //roughly 0.02685

const unsigned int MAX_PWM_VALUE = 255; //PMW Value needs to be variable, effected by the Mx_I_Counts

void M1_backward(unsigned int PWM) {
  analogWrite(M1_IN_1, PWM); //so we're writing to pin 2 the value 255
  analogWrite(M1_IN_2, 0); //and then writing to pin 3 the value 0
}

void M1_forward(unsigned int PWM) {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM);
}

void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward(unsigned int PWM) {
  analogWrite(M2_IN_1, PWM);
  analogWrite(M2_IN_2, 0);
}

void M2_forward(unsigned int PWM) {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM);
}

void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

void setup() {
  Serial.begin(115200); //sets up tick rate to transmit
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);
  //initialize the encoder value on start up
  m1PrevRead = enc1.read();
  m2PrevRead = -enc2.read();
}

void loop() {
  //M1_stop();
  //M2_stop();
  currTime = micros();
  m1CurrRead = enc1.read();
  m2CurrRead = -enc2.read(); //m2 is naturall negative

  m1AdjRead = m1CurrRead - m1PrevRead;
  float m1Pos = m1AdjRead * dist_per_tick;
  
  m2AdjRead = m2CurrRead - m2PrevRead;
  float m2Pos = m2AdjRead * dist_per_tick;


  delTime = (currTime - prevTime)/ 1e6;


  float m1Error = m1Pos - targetPos;
  float m2Error = m2Pos - targetPos;

  m1P = m1Error;
  m1I = m1I + (m1Error * delTime);
  m1D = (m1Error - m1PrevError)/delTime;
  m1PrevError = m1Error;

  m2P = m2Error;
  m2I = m2I + (m2Error * delTime);
  m2D = (m2Error - m2PrevError)/delTime;
  m2PrevError = m2Error;

  m1PWM = -(m1P * m1Kp + m1I * m1Ki + m1D * m1Kd);
  m2PWM = -(m2P * m2Kp + m2I * m2Ki + m2D * m2Kd);

  if (m1PWM > MAX_PWM_VALUE) {
    m1PWM = MAX_PWM_VALUE;
  }
  if (m1PWM < 0) {
    m1PWM = 0;
  }

  if (m2PWM > MAX_PWM_VALUE) {
    m2PWM = MAX_PWM_VALUE;
  }
  if (m2PWM < 0) {
    m2PWM = 0;
  }

  targetPos = targetPos + (targetVel * 100 * delTime);
  prevTime = currTime;

  M1_forward(m1PWM);
  M2_forward(m2PWM);
  
  //Left motor is definitely faster/stronger
  delM1Pos = (m1CurrRead - m1PrevRead)/(currTime - prevTime); // velocity (delx/delt) = 0.003864
  delM2Pos = (m2CurrRead - m2PrevRead)/(currTime - prevTime); // velocity (delx/delt) = 0.003647


}
