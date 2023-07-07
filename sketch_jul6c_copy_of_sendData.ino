/****************************************************************************************************************************
  ESP8266_WebSocketClientSocketIO.ino
  For ESP8266

  Based on and modified from WebSockets library https://github.com/Links2004/arduinoWebSockets
  to support other boards such as SAMD21, SAMD51, Adafruit's nRF52 boards, etc.

  Built by Khoi Hoang https://github.com/khoih-prog/WebSockets_Generic
  Licensed under MIT license
 
  Originally Created on: 06.06.2016
  Original Author: Markus Sattler
*****************************************************************************************************************************/

#if !defined(ESP8266)
#error This code is intended to run only on the ESP8266 boards! Please check your Tools->Board setting.
#endif

#define _WEBSOCKETS_LOGLEVEL_ 2

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ArduinoJson.h>

#include <WebSocketsClient_Generic.h>
#include <SocketIOclient_Generic.h>

#include <Hash.h>

#define LED_PIN 0     // D3 pin
#define BUTTON_PIN 14 // D5 pin

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

// Initialize the LED state and button state variables
bool ledState = LOW;
bool buttonState = LOW;
bool lastButtonState = LOW;
int buttonPressCount = 0;             // Initialize the button press count variable
unsigned long buttonPressTime = 0;    // Initialize the button press time variable

// Select the IP address according to your local network
IPAddress serverIP(49, 249, 200, 132);
uint16_t serverPort = 3000; //8080;    //3000;

void sendInstructionToStationEventHandler(uint8_t *payload)
{
  // Lock or unlock event handler logic
  if (strcmp((char *)payload, "SendInstructionToStation") == 0)
  {
    // Handle the SendInstructionToStation event
    Serial.println("SendInstructionToStation"); // Print "SendInstructionToStation" message
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle the LED state
  }
}
void socketIOEvent(const socketIOmessageType_t &type, uint8_t *payload, const size_t &length)
{
  switch (type)
  {
  case sIOtype_CONNECT:
    Serial.print("[IOc] Connected to url: ");
    Serial.println((char *)payload);

    // join default namespace (no auto join in Socket.IO V3)
    socketIO.send(sIOtype_CONNECT, "/");

    break;

  case sIOtype_EVENT:
    Serial.print("[IOc] Get event: ");
    Serial.println((char *)payload);

    if (strcmp((char *)payload, "SendInstructionToStation") == 0)
    {
      sendInstructionToStationEventHandler(payload);
    }

    break;

  case sIOtype_ACK:
    Serial.print("[IOc] Get ack: ");
    Serial.println(length);

    hexdump(payload, length);
    break;

  case sIOtype_ERROR:
    Serial.print("[IOc] Get error: ");
    Serial.println(length);

    hexdump(payload, length);
    break;

  case sIOtype_BINARY_EVENT:
    Serial.print("[IOc] Get binary: ");
    Serial.println(length);

    hexdump(payload, length);
    break;

  case sIOtype_PONG:
    Serial.println("[IOc] Get PONG");

    break;

  default:
    break;
  }
}


void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  while (!Serial)
    ;

  Serial.print("\nStart ESP8266_WebSocketClientSocketIO on ");
  Serial.println(ARDUINO_BOARD);
  Serial.println(WEBSOCKETS_GENERIC_VERSION);

  WiFiMulti.addAP("mkp", "11111111");

  while (WiFiMulti.run() != WL_CONNECTED)
  {
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

void loop()
{
  socketIO.loop();

  buttonState = digitalRead(BUTTON_PIN);

  if (buttonState != lastButtonState)
  {
    if (buttonState == HIGH)
    {
      buttonPressTime = millis();
      buttonPressCount++;

      delay(5000);
      digitalWrite(LED_PIN, HIGH);

      // Create a JSON payload with messageKey and value
      StaticJsonDocument<64> doc;
      doc["Status"] = "unlock";

      // Serialize the JSON payload
      size_t len = measureJson(doc);
      char jsonPayload[len + 1];
      serializeJson(doc, jsonPayload, len + 1);

      // Send the JSON payload to the server
      socketIO.sendEVENT("SendInstructionToStation", strlen(jsonPayload));
    }
    else
    {
      digitalWrite(LED_PIN, LOW);

      // Create a JSON payload with messageKey and value
      StaticJsonDocument<64> doc;
      doc["Status"] = "lock";

      // Serialize the JSON payload
      size_t len = measureJson(doc);
      char jsonPayload[len + 1];
      serializeJson(doc, jsonPayload, len + 1);

      // Send the JSON payload to the server
      socketIO.sendEVENT("SendInstructionToStation", strlen(jsonPayload));
    }
  }

  lastButtonState = buttonState;
}


