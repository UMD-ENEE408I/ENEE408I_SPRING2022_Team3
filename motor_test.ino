#include <Encoder.h>
#include <PID_v1.h>

//Encoder Initialization
const unsigned int M1_ENC_A = 39; //guaranteed to have interrupt capabilities
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37; //this one too
const unsigned int M2_ENC_B = 36;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

long m1PrevRead = 0;
long m2PrevRead = 0;
long m1CurrRead = 0;
long m2CurrRead = 0;


//Motor Initialization
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 1;
const unsigned int M1_IN_2_CHANNEL = 2;
const unsigned int M2_IN_1_CHANNEL = 3;
const unsigned int M2_IN_2_CHANNEL = 4;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int MAX_PWM_VALUE = 255; // Max PWM given 8 bit resolution
const unsigned int MIN_PWM_VALUE = 30; // Just a Min PWM to make sure motor moves

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

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

void move_forward() {
  M1_forward(100);
  M2_forward(100);
}

void move_backward(){
  M1_backward(100);
  M2_backward(100);
}


//motor timing
unsigned long currTime = 0; //updates every loop
unsigned long m1StartTime = 0; //timing for m1 interrupts
unsigned long m2StartTime = 0; //timing for m2 interrupts
unsigned long countIntM1 = 0;
unsigned long countIntM2 = 0; //counter for interrupts
double m1Period = 0;
double m2Period = 0; //motor period

//PID variables
const unsigned long SAMPLE_TIME = 100; //time between pid updates
const unsigned long INT_COUNT = 20; //num of interrupts needed for accurate timing

double setpointM1 = 150; //This is rotational speed in HZ (rev/s)
double m1PWM = 0; //PWM to m1
double m1Out = 0; //output in Hz

double setpointM2 = 150; //This is rotational speed in HZ (rev/s)
double m2PWM = 0; //PWM to m2
double m2Out = 0; //output in Hz

double m1Kp = 3, m2Kp = 3;
double m1Ki = 1.2, m2Ki = 1.2;
double m1Kd = 0, m2Kd = 0;

PID motorM1(&m1PWM, &m1Out, &setpointM1, m1Kp, m1Ki, m1Kd, DIRECT);
PID motorM2(&m2PWM, &m2Out, &setpointM2, m2Kp, m2Ki, m2Kd, DIRECT);

double debugStore = 0; //debug printing

void isr_A(){
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntM1++;
  if (countIntM1 == INT_COUNT){
    m1PWM = (float) INT_COUNT * 1000 / (float)(currTime - m1StartTime);
    m1StartTime = currTime;
    countIntM1 = 0;
  }
}
void isr_B(){
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntM2++;
  if (countIntM2 == INT_COUNT){
    m2PWM = (float) INT_COUNT * 1000 / (float)(currTime - m2StartTime);
    m2StartTime = currTime;
    countIntM2 = 0;
  }
}

void setup() {
  Serial.begin(115200);
  //init of motors
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

  //encoder init
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr_A, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr_B, RISING);

  m1PrevRead = enc1.read();
  m2PrevRead = enc2.read();

  //PWM init
  m1StartTime = millis();
  m2StartTime = millis();
  motorM1.SetOutputLimits(MIN_PWM_VALUE, MAX_PWM_VALUE);
  motorM2.SetOutputLimits(MIN_PWM_VALUE, MAX_PWM_VALUE);
  motorM1.SetSampleTime(SAMPLE_TIME);
  motorM2.SetSampleTime(SAMPLE_TIME);
  motorM1.SetMode(AUTOMATIC);
  motorM2.SetMode(AUTOMATIC);
  
}

void loop() {
  
  currTime = millis();
  motorM1.Compute();
  motorM2.Compute();

  M1_forward((int)m1Out);
  M2_forward((int)m2Out);
  
  if (debugStore != m2Out){
    debugStore = m2Out;
    Serial.println("m1input, m2inputB, m1error, m2error");
    Serial.print(m1PWM); Serial.print("  ");
    Serial.print(m2PWM); Serial.print("  ");
    Serial.print(100*(setpointM1-m1PWM)/setpointM1); Serial.print("  ");
    Serial.print(100*(setpointM2-m2PWM)/setpointM2); Serial.println("\n");
  }

  /*
  M1_stop();
  M2_stop();

  delay(5000);
  
  move_forward();

  for(int i = 0; i < 500; i++) { 
    // 2/7/22 Levi: these are reading zero,
    // these pins use a very low voltage and
    // according to this resource the pins may not be
    // be able to read voltages below 100mV
    // https://deepbluembedded.com/esp32-adc-tutorial-read-analog-voltage-arduino/
    // Nobody used motor current last semester so ignore for now
    m1CurrRead = enc1.read();
    m2CurrRead = enc2.read();
    
    Serial.print(m1CurrRead);
    Serial.print("\t");
    Serial.print(m2CurrRead);
    Serial.print("\t");
    Serial.println();
    delay(1);
  }

  move_backward();
  delay(500);*/
}
