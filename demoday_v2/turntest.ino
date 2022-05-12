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

float m1PWM = 110; //changed from 120
float m2PWM = 110;
float prevM1PWM = m1PWM;
float prevM2PWM = m2PWM;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;
const int center_pos = 6;

//u = Kp*e + Ki * integral(e) + Kd * de/dt
//When doing a comma declaration, it'll automatically set the first value as 0, contrary to C
float errorSig = 0;
float errorSig2 = 0;
float prevError = 0;

//trying two loop pid
float m1PrevError = 0;
float m2PrevError = 0;
float m1Kp = 7; //8 with delay(100)
float m2Kp = 7;
float m1Ki = .15;
float m2Ki = .15;
float m1Kd = 0.1;
float m2Kd = 0.1;

float m1P, m2P, m1D, m2D;
float m1I = 80/m1Ki;
float m2I = 80/m1Ki; //80 with delay(100)

float P,D;
float I = 70;
float avg_pos;
float pos_control;
float pos_control2;

float prevTime = 0;
float currTime = 0;
float delTime = 0;

int initEnc= 0;
int encMv = 0;


//AutoPID linePID();
const unsigned int MAX_PWM_VALUE = 95;
const unsigned int MIN_PWM_VALUE = 80;//PMW Value needs to be variable, effected by the Mx_I_Counts. We cap at 120 or else it'll spin wildly out of control

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
  int adcfull[16];
  int adc1_buf[8];
  int adc2_buf[8];
  int count = 0;
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
    adcfull[2*i] = adc1_buf[i];
    adcfull[2*i+1] = adc2_buf[i];
  }

  for (int i = 0; i < 13; i++){
    //if(adcfull[i] > wht_val[i] - 20 && adcfull[i] < wht_val[i] + 20){
    if(adcfull[i] < 450){  //the line above has been commented out because the difference between black and tape is super high
      count++; //this measures how many sensors are turned on at a time. useful for junction and left right decisions. 
    }
  }
  return count;
}

int junction(boolean adc_loc[]) {
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

  if(right_count > 4 || left_count > 4){ //up until  here
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

  if(client) {
    while(1) {

  //Serial.println("Direction:");
  char direction = 's';
  
  int blk_val[13] = {722, 703, 706, 698, 703, 706, 709, 702, 707, 707, 712, 707, 724};
  //int wht_val[13] = {637, 540, 566, 455, 514, 543, 591, 533, 539, 614, 644, 607, 655}; //cutoff array
  int wht_val[13] = {72,  64,  65,  59,  68,  65,  67,  62,  62,  82,  84,  71,  63}; //values of the reflective tape under perfect conditions
  boolean adc_loc[13];
  int adcfull[16];
  int turnflag = 0;
  int adc1_buf[8];
  int adc2_buf[8];
  int count = 0;  //gotta reset count and avg_pos on each loop, this is more applicable when we merge the line following with the other main motor control
  avg_pos = 0;
  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
    adcfull[2*i] = adc1_buf[i];
    adcfull[2*i+1] = adc2_buf[i];
  }
  int t_end = micros();

  for (int i = 0; i < 13; i++){
    //if(adcfull[i] > wht_val[i] - 20 && adcfull[i] < wht_val[i] + 20){
    if(adcfull[i] < 450){  //the line above has been commented out because the difference between black and tape is super high
      adc_loc[i] = true; //CHANGED FOR MY FLOOR
      avg_pos += i;
      count++; //this measures how many sensors are turned on at a time. useful for junction and left right decisions. 
    }
    else { 
      adc_loc[i] = false; 
      }
  }


    //replace with junction function

    
    initEnc = enc1.read();
    int jflag = junction(adc_loc);
    if(jflag == 1){
      M1_stop();
      M2_stop();
      delay(100);
      encMv = enc1.read();
      while(abs(encMv-initEnc) < 170){
        M1_backward(100);
        M2_backward(100);
        delay(50);
        M1_stop();
        M2_stop();
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
      while(abs(encMv-initEnc) < 390){
        M1_forward(120);
        M2_forward(120);
        delay(50);
        M1_stop();
        M2_stop();
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
        while(gyro_degrees > -65) {
          getGyroDeg();
          Serial.println(gyro_degrees);
        }
        M1_stop();
        M2_stop();
      }
      else if(direction == 'L') {
        M2_forward(110);
        M1_backward(110);
        while(gyro_degrees < 65) {
          getGyroDeg();
          Serial.println(gyro_degrees);
        }
        M1_stop();
        M2_stop();
      }
      else if(direction == 'F') {
        /*
        M2_forward(90);
        M1_forward(100);
        delay(400);
        M1_stop();
        M2_stop();
        */
      }

      delay(500);
    
    }

  Serial.println(direction);
  
  if(count > 0){ //in bound
    avg_pos = avg_pos/count;
    errorSig = avg_pos - center_pos;
    errorSig2 = center_pos - avg_pos;
  } else { //out of bound
    avg_pos = 100;
    errorSig = 0;
    errorSig2 = 0;

    gyro_degrees = 0.00;

    M1_stop();
    M2_stop();
    delay(500);
    
    M1_forward(100);
    M2_backward(130);
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
  
  //This needs to be frozen or reset upon 180 turn ie. count < 1.
  delTime = (t_end - t_start)/1e4;

  m1P = errorSig;
  m1I = m1I + (errorSig * delTime);  //Integrator portion turned off as it provides nothing of value in this situation
  m1D = (errorSig - prevError)/delTime;  //this makes the changes in the calculated pwm a little smoother
  
  pos_control = (m1P * m1Kp) + (m1I * m1Ki) + (m1D * m1Kd);
  prevError = errorSig;

  m2P = errorSig2;
  m2I = m2I + (errorSig2 * delTime);  //Integrator portion turned off as it provides nothing of value in this situation
  m2D = (errorSig2 - m2PrevError)/delTime;  //this makes the changes in the calculated pwm a little smoother
  
  pos_control2 = (m2P * m2Kp) + (m2I * m2Ki) + (m2D * m2Kd);
  m2PrevError = errorSig2;

  
  


  if(count > 0){
    if(errorSig < 0){ //the right bar is lit, we need to turn right
    }else if(errorSig > 0){
    }else{
      //do nothing
    }
    m1PWM = pos_control2;
    m2PWM = pos_control;


    if (m1PWM > MAX_PWM_VALUE) {
      m1PWM = MAX_PWM_VALUE;
    }
    if (m1PWM < MIN_PWM_VALUE) {
      m1PWM = MIN_PWM_VALUE;
    }
  
    if (m2PWM > MAX_PWM_VALUE) {
      m2PWM = MAX_PWM_VALUE;
    }
    if (m2PWM < MIN_PWM_VALUE) {
      m2PWM = MIN_PWM_VALUE;
    }

    M1_forward(m1PWM);
    M2_forward(1.01 * m2PWM);
    Serial.println(m1PWM);
    //delay(100);
    //M1_stop();
    //M2_stop();
  }else{  //This where the error catching/decision making should be made. In this situation, the bot is likely off the line. The logic should be to reverse the operation of the previous movement, or just backup a set distance until count > 0.
    
    M1_stop();
    M2_stop();
    M1_backward(100);
    M2_backward(100);
    delay(100);
    M1_stop();
    M2_stop();
    //stop
  }

  //delay(100); //a delay of 100 is to let the motor move a little before the next loop
    
  }
  }
}
