#include <Adafruit_MCP3008.h>
#include <AutoPID.h>
#include <Encoder.h>
#include <WiFi.h>

#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const char* ssid = "GoTerps";
const char* password =  "goterps2022";

Adafruit_MPU6050 mpu;
unsigned long gyro_prev_time = 0; // extern 
unsigned long gyro_current_time = 0; // extern 
float gyro_degrees = 0.00; // extern  

WiFiServer wifiServer(8000);

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

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

float m1PWM = 120;
float m2PWM = 120;
float prevM1PWM = m1PWM;
float prevM2PWM = m2PWM;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;
const int center_pos = 6;

//Variable for encoders
long m1PrevRead = 0;
long m2PrevRead = 0;
long m1CurrRead = 0;
long m2CurrRead = 0;
long m1AdjRead = 0;
long m2AdjRead = 0;


//encoder pid variables
//u = lfm1Kp*e + lfm1Ki * integral(e) + lfm1Kd * de/dt
//When doing a comma declaration, it'll automatically set the first value as 0, contrary to C
float lfm1errorSig = 0;
float lfm1Kp = 0.8;  //the lfm1Kp value cannot be too low or else not enough pwm is supplied to the motors, it'll hover at 60-70, as opposed to 90-120
float lfm1Ki = 0.02; //1.3
float lfm1Kd = 4;
float lfm1prevError = 0;

float lfm1P,lfm1D;
float avg_pos;
float m1pos_control;
float lfm1I;// = 90/lfm1Ki;


float lfm2errorSig = 0;
float lfm2Kp = 1;  //the lfm1Kp value cannot be too low or else not enough pwm is supplied to the motors, it'll hover at 60-70, as opposed to 90-120
float lfm2Ki = 0.04; //1.3
float lfm2Kd = 4;
float lfm2prevError = 0;

float lfm2P,lfm2D;
float m2pos_control;
float lfm2I;// = 90/lfm2Ki;







//vel control pid

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
float m1Kp = 0.3;
float m2Kp = 0.4;
float m1Ki = 0.2;
float m2Ki = 0.27;
float m1Kd = 0.1;
float m2Kd = 0.1;

float m1P, m2P, m1D, m2D;
float m1I = -90/m1Ki;
float m2I = -90/m2Ki;
long randNum;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120; //roughly 0.02685





float prevTime = 0;
float currTime = 0;
float delTime = 0;


int initEnc= 0;
int encMv = 0;


//Light Bar Sensor variables
int adcfull[16];
int adc1_buf[8];
int adc2_buf[8];
boolean adc_loc[13];
int count = 0;



//AutoPID linePID();
const unsigned int MAX_PWM_VALUE = 110;
const unsigned int MIN_PWM_VALUE = 85;//PMW Value needs to be variable, effected by the Mx_I_Counts. We cap at 120 or else it'll spin wildly out of control

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

int get_adc_count(){
  count = 0;
  avg_pos = 0;
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
    adcfull[2*i] = adc1_buf[i];
    adcfull[2*i+1] = adc2_buf[i];
  }

  for (int i = 0; i < 13; i++){
    //if(adcfull[i] > wht_val[i] - 20 && adcfull[i] < wht_val[i] + 20){
    if(adcfull[i] < 650){  //the line above has been commented out because the difference between black and tape is super high
      adc_loc[i] = true;
	  avg_pos += i;
	  count++; //this measures how many sensors are turned on at a time. useful for junction and left right decisions. 
    }else{
	  adc_loc[i] = false;
    }
  }
  return count;
}
void PID_Move_Controller(float targetVel1, float targetVel2){
  currTime = micros();
//  m1CurrRead = enc1.read();
//  m2CurrRead = -enc2.read(); //m2 is naturally negative
  m1AdjRead = m1CurrRead - m1PrevRead; //prevRead basically initRead
  float m1Pos = m1AdjRead * dist_per_tick;
  
  m2AdjRead = m2CurrRead - m2PrevRead;
  float m2Pos = -m2AdjRead * dist_per_tick;

  delTime = (currTime - prevTime)/ 1e4;

  float m1Error = m1Pos - targetPos1;
  float m2Error = m2Pos - targetPos2;

  Serial.print(m1Error);
  Serial.print('\t');
  Serial.print(m2Error);
  Serial.print('\t');
  Serial.println(delTime);

  m1P = m1Error;
  m1I = m1I + (m1Error * delTime);
  m1D = (m1Error - m1PrevError)/delTime;
  

  m2P = m2Error;
  m2I = m2I + (m2Error * delTime);
  m2D = (m2Error - m2PrevError)/delTime;


  m1PWM = -((m1P * m1Kp) + (m1I * m1Ki) + (m1D * m1Kd));
  m2PWM = -((m2P * m2Kp) + (m2I * m2Ki) + (m2D * m2Kd));
  
  m1PrevError = m1Error;
  m2PrevError = m2Error;
  
  
  if (m1PWM > 120) {
    m1PWM = 120;
  }
  if (m1PWM < 100) {
    m1PWM = 100;
  }

  if (m2PWM > 120) {
    m2PWM = 120;
  }
  if (m2PWM < 100) {
    m2PWM = 100;
  }
  
  targetPos1 = (targetVel1 * 0.01 * delTime);
  targetPos2 = (targetVel2 * 0.01 * delTime);
  prevTime = currTime;
    
  
}

int junction(boolean adc_loc[]){
  int left_count  = 0;
  int right_count = 0;
  for(int i = 0; i < 5; i++){
    if(adc_loc[i]){
      left_count++;
    }
  }
  for(int i = 6; i < 13; i++){
    if(adc_loc[i]){
      right_count++;
    }
  }

  if(right_count > 3 || left_count > 3){ //up until  here
    Serial.println("Junction");
    return 1;
  }
  return  0;
}

void getGyroDeg(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyro_current_time = millis();
  gyro_degrees += (g.gyro.z + .010403) * (((float)gyro_current_time)/1000.00 - ((float)gyro_prev_time)/1000.00)*180.00/PI; //returns degrees travelled since last call.
  gyro_prev_time = gyro_current_time;
}

void reset_variables(){
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = 0;
    adc2_buf[i] = 0;
    adcfull[2*i] = 0;
    adcfull[2*i+1] = 0;
  }
}



void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200);

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
  
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
  prevTime = micros();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  wifiServer.begin();
  
}

void loop() {

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  
  WiFiClient client = wifiServer.available();
  m1PrevRead = enc1.read();
  m2PrevRead = -enc2.read();
  targetPos1 = enc1.read()*dist_per_tick;
  targetPos2 = -enc2.read()*dist_per_tick;
  if(client) {
  while(1) {

  //Serial.println("Direction:");
  char direction = 's';
  
  int blk_val[13] = {722, 703, 706, 698, 703, 706, 709, 702, 707, 707, 712, 707, 724};
  //int wht_val[13] = {637, 540, 566, 455, 514, 543, 591, 533, 539, 614, 644, 607, 655}; //cutoff array
  int wht_val[13] = {72,  64,  65,  59,  68,  65,  67,  62,  62,  82,  84,  71,  63}; //values of the reflective tape under perfect conditions
  
  int turnflag = 0;
  int forwardflag = 0;
  //gotta reset count and avg_pos on each loop, this is more applicable when we merge the line following with the other main motor control
  
  
  
  
  avg_pos = 0;
  int currTime = micros();
  int t_start = micros();

  reset_variables();
  
  m1CurrRead = enc1.read();
  m2CurrRead = -enc2.read(); //m2 is naturally negative
  PID_Move_Controller(0.025, 0.025);
  m1PrevRead = m1CurrRead;
  m2PrevRead = m2CurrRead;

  
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
    adcfull[2*i] = adc1_buf[i];
    adcfull[2*i+1] = adc2_buf[i];
  }
  int t_end = micros();

  for (int i = 0; i < 13; i++){
    //if(adcfull[i] > wht_val[i] - 20 && adcfull[i] < wht_val[i] + 20){
    if(adcfull[i] < 650){  //the line above has been commented out because the difference between black and tape is super high
      adc_loc[i] = true; //CHANGED FOR MY FLOOR
      avg_pos += i;
      count++; //this measures how many sensors are turned on at a time. useful for junction and left right decisions. 
    }
    else { 
      adc_loc[i] = false; 
      }
  }



    //Junction detection
    initEnc = enc1.read();
    int jflag = junction(adc_loc);
    if(jflag > 0){
      M1_stop();
      M2_stop();
      delay(100);


      encMv = enc1.read();
      while(abs(encMv-initEnc) < 100){
        M1_backward(100);
        M2_backward(100);
        delay(50);
        encMv = enc1.read();
      }



      M1_stop();
      M2_stop();
      delay(100);
      
      Serial.println("Python");
      client.write("Begin");
  
      delay(1500); //Delay to make time for python to make decision
  
      direction = client.read();

      //Serial.println(direction);

      delay(2000);
  
      Serial.println("HERE IS THE DIRECTION:");
      //Serial.println(direction);
      initEnc = enc1.read();
      encMv = enc1.read();
      while(abs(encMv-initEnc) < 250){
        M1_forward(100);
        M2_forward(100);
        delay(50);
        encMv = enc1.read();
      }
      //delay(420);
      M1_stop();
      M2_stop();//It should now be at the center of the intersection
      delay(500);

      gyro_degrees = 0.00;

      if(direction == 'R') {
        M1_forward(110);
        M2_backward(110);
        while(gyro_degrees > -62) {
          getGyroDeg();
          Serial.println(gyro_degrees);
        }
        M1_stop();
        M2_stop();
      }
      else if(direction == 'L') {
        M2_forward(110);
        M1_backward(110);
        while(gyro_degrees < 62) {
          getGyroDeg();
          Serial.println(gyro_degrees);
        }
        M1_stop();
        M2_stop();
      }
	  forwardflag = 1;
    delay(500);
    }

  Serial.println(direction);
  if(direction == 'F'){
	  forwardflag = 0;
  }
  if(count > 0 && forwardflag < 1){ //in bound or not after a turn
    avg_pos = avg_pos/count;
    lfm2errorSig = avg_pos - center_pos;
    lfm1errorSig = center_pos - avg_pos;
  
    delTime = (t_end - t_start)/1e4;
  
    lfm1P = lfm1errorSig;
    lfm1I = lfm1I + (lfm1errorSig * delTime);  //Integrator portion turned off as it provides nothing of value in this situation
    lfm1D = (lfm1errorSig - lfm1prevError)/delTime;  //this makes the changes in the calculated pwm a little smoother
    
    m1pos_control = (lfm1P * lfm1Kp) + (lfm1I * lfm1Ki) + (lfm1D * lfm1Kd);
    lfm1prevError = lfm1errorSig;
  
  
    lfm2P = lfm2errorSig;
    lfm2I = lfm2I + (lfm2errorSig * delTime);  //Integrator portion turned off as it provides nothing of value in this situation
    lfm2D = (lfm2errorSig - lfm2prevError)/delTime;  //this makes the changes in the calculated pwm a little smoother
    
    m2pos_control = (lfm2P * lfm2Kp) + (lfm2I * lfm2Ki) + (lfm2D * lfm2Kd);
    lfm2prevError = lfm2errorSig;
  
    if(count > 0){
      
      m1PWM += m1pos_control;
      m2PWM += m2pos_control;
      
      if (m1PWM > MAX_PWM_VALUE) {
        m1PWM = MAX_PWM_VALUE;
      }
      if (m1PWM < 90) {
        m1PWM = 90;
      }
    
      if (m2PWM > MAX_PWM_VALUE) {
        m2PWM = MAX_PWM_VALUE;
      }
      if (m2PWM < 90) {
        m2PWM = 90;
      }
  
      M1_forward(m1PWM);
      M2_forward(m2PWM);
   //    Serial.println(m1PWM);
    }
	
  } else { //out of bound
    avg_pos = 100;
    //lfm1errorSig = 0;
    //lfm2errorSig = 0;
    //not sure how this ^ might affect the new movement control
    m1I = -100/m1Ki;
    m2I = -100/m2Ki;

    
    gyro_degrees = 0.00;

    M1_stop();
    M2_stop();
    delay(500);
    
    M1_forward(100);
    M2_backward(100);
    turnflag = get_adc_count();
    while(abs(gyro_degrees) < 175 && turnflag < 3) {
      getGyroDeg();
      Serial.println(gyro_degrees);
      turnflag = get_adc_count();
    }
    M1_stop();
    M2_stop();
    delay(500);    
  }
  prevTime = currTime;
  delay(50);
//M1_stop();
//M2_stop();
  
    
  }
  }
}
