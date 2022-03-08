#include <Encoder.h>
#include <Arduino.h>
//Adafriut
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

//For mouse 1, M1 is Left Motor, M2 is Right Motor
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

//Encoder enc1(M1_ENC_A, M1_ENC_B);
//Encoder enc2(M2_ENC_A, M2_ENC_B);

//Interrupt Pins
const unsigned int M1_IN_1 = 13; //pin numbers for the motors
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 1;
const unsigned int M1_IN_2_CHANNEL = 2;
const unsigned int M2_IN_1_CHANNEL = 3;
const unsigned int M2_IN_2_CHANNEL = 4;

//Analog Input
const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

//Variable for encoders
long m1PrevRead = 0;
long m2PrevRead = 0;
long m1CurrRead = 0;
long m2CurrRead = 0;
long m1AdjRead = 0;
long m2AdjRead = 0;

float m1PWM = 255;
float m2PWM = 255;
float prevM1PWM = m1PWM;
float prevM2PWM = m2PWM;

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

float targetVel = 0.05; //usually 0.2
float curVel1, curVel2;
float targetPos1, targetPos2 = 0; //initial position of mouse is 0

//u = Kp*e + Ki * integral(e) + Kd * de/dt
//When doing a comma declaration, it'll automatically set the first value as 0, contrary to C
float m1PrevError = 0;
float m2PrevError = 0;
float m1Kp = 3;
float m2Kp = 3;
float m1Ki = 1.2;
float m2Ki = 1.2;
float m1Kd = 0;
float m2Kd = 0;

float m1P, m2P, m1I, m2I, m1D, m2D;



const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120; //roughly 0.02685

const unsigned int MAX_PWM_VALUE = 126; //PMW Value needs to be variable, effected by the Mx_I_Counts

void M1_backward(unsigned int PWM_VALUE) {
  ledcWrite(M1_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M1_forward(unsigned int PWM_VALUE) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, PWM_VALUE);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M2_backward(unsigned int PWM_VALUE) {
  ledcWrite(M2_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M2_forward(unsigned int PWM_VALUE) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, PWM_VALUE);
}

void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void left_turn(unsigned int m1PWM, unsigned int m2PWM){
  //use IMU (gyroscope/Mag field) to turn?
  //Right turn Mag value : 233 - 50, left turn 50 to -10. The values are realy wacky, and are affected by other magnetic options
  //gyro gz does show when a turn is taking place, but it'll have to be timing based.
  M1_backward(m1PWM);
  M2_forward(m2PWM);
}
void right_turn(unsigned int m1PWM, unsigned int m2PWM){
  M1_forward(m1PWM);
  M2_backward(m2PWM);
}

void PID_Move_Controller(float targetVel1, float targetVel2, int turn){
  currTime = micros();
//  m1CurrRead = enc1.read();
//  m2CurrRead = -enc2.read(); //m2 is naturally negative
  m1AdjRead = m1CurrRead - m1PrevRead;
  float m1Pos = m1AdjRead * dist_per_tick;
  
  m2AdjRead = m2CurrRead - m2PrevRead;
  float m2Pos = m2AdjRead * dist_per_tick;


  delTime = (currTime - prevTime)/ 1e6;


  float m1Error = m1Pos - targetPos1;
  float m2Error = m2Pos - targetPos2;

  m1P = m1Error;
  m1I = m1I + (m1Error * delTime);
  m1D = (m1Error - m1PrevError)/delTime;
  

  m2P = m2Error;
  m2I = m2I + (m2Error * delTime);
  m2D = (m2Error - m2PrevError)/delTime;


  //if( (abs(m1Error) < 1) && (abs(m2Error) < 1)){
  //  m1PWM = prevM1PWM;
  //  m2PWM = prevM2PWM;
  //}else{
  m1PWM = -((m1P * m1Kp) + (m1I * m1Ki) + (m1D * m1Kd));
  m2PWM = -(m2P * m2Kp + m2I * m2Ki + m2D * m2Kd);
  //}

  m1PrevError = m1Error;
  m2PrevError = m2Error;

  
  Serial.print("m1PWM:");
  Serial.print(m1PWM);
  Serial.print('\t');
  Serial.print(m1P*m1Kp);
  Serial.print('\t');
  Serial.print(m1I*m1Ki);
  Serial.print('\t');
  Serial.print(m1D*m1Kd);
  Serial.print('\n');
  Serial.print("m2PWM:");
  Serial.print(m2PWM);
  Serial.print('\t');
  Serial.print(m2P);
  Serial.print('\t');
  Serial.print(m2I);
  Serial.print('\t');
  Serial.print(m2D);
  Serial.print('\n');

  
  
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
  
  targetPos1 = targetPos1 + (targetVel1 * 100 * delTime);
  targetPos2 = targetPos2 + (targetVel2 * 100 * delTime);
  prevTime = currTime;

//  prevM1PWM = m1PWM;
//  prevM2PWM = m2PWM;

//  M1_forward(m1PWM);
//  M2_forward(m2PWM);
  switch(turn){
    case 1:
      M1_forward(m1PWM);
      M2_forward(m2PWM);
      break;
    case 2:
      M1_forward(m1PWM);
      M2_backward(m2PWM);
      break;
    case 3:
      M1_backward(m1PWM);
      M2_forward(m2PWM);
      break;
  }
  
}


//encoder changes by 200 to turn left and right
void left90(){
  PID_Move_Controller(-0.2, 0.2, 3);
  delay(100);
}
void right90(){
  PID_Move_Controller(0.2, -0.2, 2);
  delay(100);
}

void move_forward(){
  PID_Move_Controller(0.2, 0.2, 1);
  delay(500);
  M1_stop();
  M2_stop();
}

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200); //sets up tick rate to transmit

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

}

void loop() {

  /* 
  //this is the motor test for the esp32 from the TA's repo. When ran on my individual mouse, only M1 turns, and it also
  //only turns backwards. However, if i were to run my own code/test, only M2 turns, and M1 stays absolutely still
  //Both M1PWM and M2PWM are positive values, as tested through M2, since M1 refuses to move. I've actually got no clue what's wrong
  M1_stop();
  M2_stop();

  delay(5000);
  
  M1_forward(MAX_PWM_VALUE);
  M2_forward(MAX_PWM_VALUE);

  for(int i = 0; i < 500; i++) { 
    // 2/7/22 Levi: these are reading zero,
    // these pins use a very low voltage and
    // according to this resource the pins may not be
    // be able to read voltages below 100mV
    // https://deepbluembedded.com/esp32-adc-tutorial-read-analog-voltage-arduino/
    // Nobody used motor current last semester so ignore for now
    int M1_I_counts = analogRead(M1_I_SENSE);
    int M2_I_counts = analogRead(M2_I_SENSE);

    Serial.print(M1_I_counts);
    Serial.print("\t");
    Serial.print(M1_I_counts * M_I_COUNTS_TO_A);
    Serial.print("\t");
    Serial.print(M2_I_counts);
    Serial.print("\t");
    Serial.print(M2_I_counts * M_I_COUNTS_TO_A);
    Serial.println();
    delay(1);
  }

  M1_backward(MAX_PWM_VALUE);
  M2_backward(MAX_PWM_VALUE);
  delay(500);
  */

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
    //initialize the encoder value on start up
  m1PrevRead = enc1.read();
  m2PrevRead = -enc2.read();

  //M1_stop();
  //M2_stop();

  while(true){
    m1CurrRead = enc1.read();
    m2CurrRead = -enc2.read(); //m2 is naturally negative
    move_forward();
    delay(1000);
    m1CurrRead = enc1.read();
    m2CurrRead = -enc2.read(); //m2 is naturally negative
    left90();
    delay(75);
    
    M1_stop();
    M2_stop();
    delay(5000);
    
    //Left motor is definitely faster/stronger
    delM1Pos = (m1CurrRead - m1PrevRead)/(currTime - prevTime); // velocity (delx/delt) = 0.003864
    delM2Pos = (m2CurrRead - m2PrevRead)/(currTime - prevTime); // velocity (delx/delt) = 0.003647
  }

}
