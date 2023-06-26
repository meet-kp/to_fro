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

#define LED_PIN 0       // D3 pin
#define BUTTON_PIN 14   // D5 pin

bool ledState = LOW;
bool buttonState = LOW;
bool lastButtonState = LOW;
bool lastLedState = 0;
int buttonPressCount = 0;
unsigned long buttonPressTime = 0;

bool socketIOEvent(const socketIOmessageType_t& type, uint8_t * payload, const size_t& length) {
  bool handled = false;

  switch (type) {
    // case sIOtype_DISCONNECT:
    //   Serial.println("[IOc] Disconnected");
    //   handled = true;
    //   break;

    case sIOtype_CONNECT:
      Serial.print("[IOc] Connected to url: ");
      Serial.println((char*) payload);
      socketIO.send(sIOtype_CONNECT, "/");
      handled = true;
      break;

    case sIOtype_EVENT:
      Serial.print("[IOc] Get event: ");
      Serial.println((char*) payload);
    
      handleIncomingEvent(payload, length);
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

void handleIncomingEvent(uint8_t* payload, size_t length) {
  // Parse the received JSON payload
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
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

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  socketIO.loop();
  uint64_t now = millis();

  buttonState = digitalRead(BUTTON_PIN);

  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      buttonPressTime = millis();
      buttonPressCount++;
      
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      
      Serial.print("Button Pressed! Count: ");
      Serial.println(buttonPressCount);

      StaticJsonDocument<128> doc;
      doc["ledStatus"] = ledState;

      String jsonPayload;
      serializeJson(doc, jsonPayload);

      socketIO.sendEVENT(jsonPayload);

      delay(2000);
    }

    lastButtonState = buttonState;
  }

  if (ledState != lastLedState) {
    Serial.print("LED State: ");
    Serial.println(ledState);
    lastLedState = ledState;
  }
}

void handleIncomingEvent(uint8_t* payload, size_t length) {
  // Parse the received JSON payload
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  // Check if parsing was successful
  if (error) {
    Serial.print("Error parsing JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract the "lock" and "unlock" values from the JSON
  const char* lockValue = doc["lock"];
  const char* unlockValue = doc["unlock"];

  // Check if "lock" value is present and equal to "0"
  if (lockValue && strcmp(lockValue, "0") == 0) {
    digitalWrite(LED_PIN, LOW);  // Turn off the LED
    Serial.println("LED turned off");
  }

  // Check if "unlock" value is present and equal to "1"
  if (unlockValue && strcmp(unlockValue, "1") == 0) {
    digitalWrite(LED_PIN, HIGH);  // Turn on the LED
    Serial.println("LED turned on");
  }
}

