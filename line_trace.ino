#include <Encoder.h>
#include <WiFi.h>


const char* ssid = "GoTerps";
const char* password =  "goterps2022";
WiFiServer wifiServer(80);


const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);


float phi, cX, cY, r, rX, rY, theta, delTime; //i think we get delta time from the line follower not 100% sure. can always call micros()
long m1Prev, m1Curr, m2Prev, m2Curr = 0;

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  m1Curr = 0;
  m2Curr = 0;
  cX = 0;
  cY = 0;
  phi = 0;

  Serial.begin(115200);
  
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

  if(client){
    
    m1Prev = m1Curr;
    m2Prev = m2Curr;
    m1Curr = enc1.read();
    m2Curr = enc2.read();
    
    int n1 = m1Curr - m1Prev;
    int n2 = m2Curr - m2Prev;
    
    if(n1 > n2){ //right turn
      r = (4*(n1 + n2)/(n1 - n2));
      theta = (180*n2)/(2*PI*(r-4));
      rX = cX + cos(phi - 90)*r;
      rY = cY + sin(phi - 90) * r;
      phi = phi - theta;
    }else if(n1 < n2){ //left turn
      r = (4*(n2 + n1)/(n2 - n1));
      theta = (180*n1)/(2*PI*(r-4));
      rX = cX + cos(phi + 90)*r;
      rY = cY + sin(phi + 90) * r;
      phi = phi + theta;
    }else{ //base case of n1 = n2
      cX = cX + n1*cos(phi);
      cY = cY + n1*sin(phi);
    }
    client.write(rX);
    client.write(rY);
  }
}
