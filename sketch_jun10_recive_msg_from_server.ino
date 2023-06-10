#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient_Generic.h>
#include <SocketIOclient_Generic.h>
#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;
IPAddress serverIP(49, 249, 200, 132);
uint16_t serverPort = 3000;

bool socketIOEvent(const socketIOmessageType_t& type, uint8_t * payload, const size_t& length) {
  bool handled = false;

  switch (type) {
    case sIOtype_DISCONNECT:
      Serial.println("[IOc] Disconnected");
      handled = true;
      break;

    case sIOtype_CONNECT:
      Serial.print("[IOc] Connected to url: ");
      Serial.println((char*) payload);
      socketIO.send(sIOtype_CONNECT, "/");
      handled = true;
      break;

    case sIOtype_EVENT:
      Serial.print("[IOc] Get event: ");
      Serial.println((char*) payload);
      // handleIncomingEvent(payload, length);
      handled = true;
      break;

    case sIOtype_ACK:
      Serial.print("[IOc] Get ack: ");
      Serial.println(length);
      hexdump(payload, length);
      handled = true;
      break;

    case sIOtype_ERROR:
      Serial.print("[IOc] Get error: ");
      Serial.println(length);
      hexdump(payload, length);
      handled = true;
      break;

    case sIOtype_BINARY_EVENT:
      Serial.print("[IOc] Get binary: ");
      Serial.println(length);
      hexdump(payload, length);
      handled = true;
      break;

    case sIOtype_BINARY_ACK:
      Serial.print("[IOc] Get binary ack: ");
      Serial.println(length);
      hexdump(payload, length);
      handled = true;
      break;

    case sIOtype_PING:
      Serial.println("[IOc] Get PING");
      handled = true;
      break;

    case sIOtype_PONG:
      Serial.println("[IOc] Get PONG");
      handled = true;
      break;
  }

  return handled;
}



void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.print("\nStart ESP8266_WebSocketClientSocketIO on ");
  Serial.println(ARDUINO_BOARD);
  Serial.println(WEBSOCKETS_GENERIC_VERSION);

  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }

  WiFiMulti.addAP("mkp", "11111111");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println();

  Serial.print("WebSockets Client started @ IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Connecting to WebSockets Server @ IP address: ");
  Serial.print(serverIP);
  Serial.print(", port: ");
  Serial.println(serverPort);

  socketIO.setReconnectInterval(10000);
  socketIO.setExtraHeaders("Authorization: 1234567890");
  socketIO.setExtraHeaders("data:test,test123,station");
  socketIO.begin(serverIP, serverPort);
  socketIO.onEvent(socketIOEvent);
}


void handleIncomingEvent(uint8_t* payload, size_t length) {
  // Parse the received JSON payload
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  // Check if parsing succeeded
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract the lock or unlock value from the JSON object
  if (doc.containsKey("lock")) {
    int lockValue = doc["lock"];
    Serial.print("Received lock value: ");
    Serial.println(lockValue);
    // Handle the lock value here
  } else if (doc.containsKey("unlock")) {
    int unlockValue = doc["unlock"];
    Serial.print("Received unlock value: ");
    Serial.println(unlockValue);
    // Handle the unlock value here
  }
}

void loop() {
  socketIO.loop();
  uint64_t now = millis();
}
