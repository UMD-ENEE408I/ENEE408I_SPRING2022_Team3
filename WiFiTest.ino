#include <WiFi.h>

const char* ssid = "4814";
const char* password =  "romans30amateur50discord";

WiFiServer wifiServer(80);

void setup() {

  Serial.begin(115200);

  delay(1000);

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

      while (client.available()>0) {
        char c = client.read();
        Serial.print(c);
        client.write(c);
      }

      delay(10);
    }

    client.stop();
    Serial.println("Client disconnected");

  }
}
