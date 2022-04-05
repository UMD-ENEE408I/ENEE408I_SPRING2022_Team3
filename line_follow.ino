#include <Adafruit_MCP3008.h>
#include <AutoPID.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;
const int center_pos = 6;

//u = Kp*e + Ki * integral(e) + Kd * de/dt
//When doing a comma declaration, it'll automatically set the first value as 0, contrary to C
float errorSig = 0;
float Kp = 2;
float Ki = 0; //1.3
float Kd = 0.0;
float prevError = 0;

float P,I,D;
float avg_pos;
float pos_control;

float prevTime = 0;
float currTime = 0;
float delTime = 0;


//AutoPID linePID();

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
  prevTime = micros();
}

void loop() {
  int blk_val[13] = {722, 703, 706, 698, 703, 706, 709, 702, 707, 707, 712, 707, 724};
  int wht_val[13] = {637, 540, 566, 455, 514, 543, 591, 533, 539, 614, 644, 607, 655}; //cutoff array
  boolean adc_loc[13];
  int adcfull[16];
  
  int adc1_buf[8];
  int adc2_buf[8];
  int count = 0;
  avg_pos = 0;
  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
    adcfull[2*i] = adc1_buf[i];
    adcfull[2*i+1] = adc2_buf[i];
  }
  int t_end = micros();
  /*
  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); Serial.print("\t");
    Serial.print(adc2_buf[i]); Serial.print("\t");
  }*/
  for (int i = 0; i < 13; i++){
    if(adcfull[i] > wht_val[i] - 20 && adcfull[i] < wht_val[i] + 20){
      adc_loc[i] = true;
      avg_pos += i;
      count++;
    }
  }
  if(count > 0){ //in bound
    avg_pos = avg_pos/count;
    errorSig = avg_pos - center_pos;
  } else { //out of bound
    avg_pos = 100;
    errorSig = 0;
  }
  

  delTime = (t_end - t_start)/1e4;

  P = errorSig;
  I = I + (errorSig * delTime);
  D = (errorSig - prevError)/delTime;
  
  pos_control = (P * Kp) + (I * Ki) + (D * Kd);
  prevError = errorSig;

  if(count > 0){
    if(errorSig < 0){
      Serial.print("Right\t");
      //augment pwm to correct
    }else if(errorSig > 0){
      Serial.print("Left\t");
      //likewise
    }else{
      Serial.print("Straight\t");
      //do nothing
    }
    Serial.print(avg_pos);
    Serial.print('\t');
    Serial.print(pos_control);  //when it prints Right, pos_control decreases, when it prints Left pos_control increases
    Serial.print('\n');

  }else{
    Serial.print("No Read");
    Serial.println();
  }

  delay(100);
}
