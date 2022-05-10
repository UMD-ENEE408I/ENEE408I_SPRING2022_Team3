#include <WiFi.h>
#include <Encoder.h>
 
const char* ssid = "Fios-TNVw7"; // GoTerps";; //
const char* password = "hid37omega22met"; //"goterps2022"; // 
long m2Curr;
long m1Curr;
long m2Prev;
long m1Prev;
char d = 'b';
 
WiFiServer wifiServer(80);


const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);
 
void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
 
  Serial.begin(115200);
 
  delay(1000);

  m1Curr = 0;
  m2Curr = 0;
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
 
  wifiServer.begin();
}
 
void loop() {
 
  WiFiClient client = wifiServer.available();
 
  if (client) {
 
    while (client.connected()) {

      m1Prev = m1Curr;
      m2Prev = m2Curr;
      m1Curr = enc1.read();
      m2Curr = -enc2.read();
      Serial.println(m1Curr - m1Prev);
      Serial.println(m2Curr - m2Prev);
  
      //while (client.available() == 0) {}
      //client.read();
      
      client.write(m1Curr - m1Prev);
      client.write(m2Curr - m2Prev);
      client.write("N"); //type 0f turn -- aka L/R/S (1eft right straight)
    }
 
    client.stop();
    Serial.println("Client disconnected");
 
  }
   //Serial.println("failed");
 
}
