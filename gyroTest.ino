#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
unsigned long gyro_prev_time = 0; // extern 
unsigned long gyro_current_time = 0; // extern 
float gyro_degrees = 0.00; // extern     

void getGyroDeg(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyro_current_time = millis();
  gyro_degrees += (g.gyro.z + .010403) * (((float)gyro_current_time)/1000.00 - ((float)gyro_prev_time)/1000.00)*180.00/PI; //returns degrees travelled since last call.
  gyro_prev_time = gyro_current_time;
}
void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(115200);

  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // put your setup code here, to run once:

}

void loop() {
  getGyroDeg();
  Serial.println(gyro_degrees); //left turn is +90, right turn is -90
  // put your main code here, to run repeatedly:

}
