/*************************************************************************************
 * Sandbox for developing MQTT apps at Lipscomb
 ************************************************************************************/

/*************************************************************************************
 *     Program name: Lab07.cpp
 *          Version: 2.2
 *             Date: Feb 11, 2017
 *           Author: Greg Nordstrom
 *         Modified: John Hutton
 *             Date: Mar 26, 2025
 *         Platform: ESP32
 *    Additional HW: Buzzer, 7-seg LED, 74HC595, rotary encoder w/ button, 220
 *                   ohm resistors for 7-seg LED.
 * Additional files: Lab07.h
 * Req'd libraries: WiFi, PubSubClient, ArduinoJson, Bounce2, Rotary
 *
 * This program implements a simple communication network consisting of up to 16
 * Huzzah32 Feather with an ESP32 processors. Nodes communicate via an MQTT
 * broker (typically a RPi running Mosquitto) to send heartbeat messages as well
 * as "ringring" messages. If a ringring message is received from a valid node
 * (0-16), the receiver plays a brief dual-tone sound using a local buzzer.
 *Nodes must also respond to "topics" messages by sending a "registeredFor"
 *message listing all the topics for which it is registered. These are listed
 *below:
 *
 * Modified for ESP32 board (fundamentally the same as original ESP8266 design)
 *
 * Received messages:
 *    Topic: "ece/node01/topics"
 *    Usage: To request a list of all topics this node is registered for
 *  Payload: none
 *
 * Emitted messages:
 *    Topic: "ece/node01/registeredFor"
 *    Usage: Reply with all N topics this node is registered for
 *  Payload: {"NodeName":"node01", "Topic0":"topic0",...,"TopicN-1":"topicN-1"}
 *
 *    Topic: "ece/node01/ringring"
 *    Usage: Sends a ringring request to other nodes
 *  Payload: {"srcNode":"nodess","dstNode":"nodedd"}
 *           where ss and dd are source and destination node numbers,
 *           respectively
 *
 * This program uses the public domain PubSubClient library to perform MQTT
 * messaging functions, and all message payloads are encoded in JSON format.
 * IMPORTANT NOTES ON PubSubClient MQTT library for Arduino:
 * 1. Default message size, including header, is only 128 bytes (rather small).
 * 2. Increase/decrease by changing MQTT_MAX_PACKET_SIZE inside PubSubClient.h.
 * 3. As of 6/14/16, this value is set to 512 (in PubSubClient.h), allowing 7-10
 *    topics to be displayed (depends on individual topic lengths of course).
 *
 * This code does not use interrupts for pushbutton processing (a fool's errand
 * when the switches are not hardware debounced). Instead, each button is
 * debounced by a Bounce2 object.
 *
 * This node responds to topics messages by sending a registeredFor message:
 *
 *    Topic: "ece/node01/registeredFor"
 *    Usage: Reply with all N topics this node is registered for
 *  Payload: {"NodeName":"node01", "Topic0":"topic0",...,"TopicN-1":"topicN-1"}
 *
 * It should be noted that this node's functionality can be fully exercised
 * using an MQTT sniffing program such as MQTT-Spy (avaliable on GitHub).
 * MQTT-Spy is a Java program, so Java must be installed on the host machine.
 * MQTT-spy is invoked on a Windows machine from the command line as follows:
 *
 *     C:>java -jar mqtt-spy-x.x.x-jar-with-dependencies.jar
 *
 * where x.x.x is the current version of MQTT.
 *
 * Other tools that may be used instead are MQTT Explorer or MQTTX.
 *
 * Modification Notes:
 * - Lab05 had similar modifications...
 * - Pins have been adapted for the ESP32
 * - Libraries have been adapted for the ESP32
 * - The rotary encoder library has been changed to ESP32Encoder
 * - The tone library has been changed to ToneESP32
 * - The ArduinoJson has been upgraded to v7 (minor changes)
 *
 ************************************************************************************/
// included configuration file and support libraries
#include <ArduinoJson.h>  // for encoding/decoding MQTT payloads in JSON format
#include <Bounce2.h>  // debounce pushbutton (by Thomas O Frederics vTBD version)
#include <ESP32Encoder.h>  // New libary (in arduino libraries) for rotary encoder
#include <Esp.h>           // Esp32 support
#include <PubSubClient.h>  // MQTT client (by Nick O'Leary?? vTBD version)
#include <WiFi.h>          // wi-fi support
                           // Slightly simpler to implement
                           // https://github.com/madhephaestus/ESP32Encoder
#include <ToneESP32.h>  // Dedicated tone library for ESP32 (gets rid of ledc errors)

#include "Lab07.h"  // included in this project

WiFiClient wfClient;              // create a wifi client
PubSubClient psClient(wfClient);  // create a pub-sub object (must be
                                  // associated with a wifi client)
bool MQTTConnected;               // Bool to hold MQTT connected state

Bounce2::Button pbButton;
ToneESP32 buzzer(buzzerPin, 0);
ESP32Encoder encoder;

// Global Variables
// Prof Note:  You will need to play your design and create globals to support
// your code
//             What I have here is just a starting point.

// Need some character arrays for processing strings and json payloads
char sbuf[80];
// Likely will need more buffers to work with jsons.
char json_ExampleBuffer[100];  // Example - May need more than one json buffer

// Rotary encoder variables.  (Suggest using ESP32Encoder library)
int currentDisplay = BLANK_7SEG_NUMBER;  // 16 = blank, 0-15 = hex digit
long lastEncoderCount = 0;

// Generally good to have some timing variables to track tasks for your main
// loop
unsigned long mscurrent = millis();
unsigned long mslastHeartbeat = mscurrent;
unsigned long mslastMQTTCheck = mscurrent;

// 7-segment patterns for hex digits 0-F + blank (index 16)
// Bit order: bit0=seg-a, bit1=seg-b, ..., bit6=seg-g (common cathode, active HIGH)
// Uses lowercase 'b' and 'd' to distinguish from '8' and '0'
const byte SEG7[] = {
  0x3F,  // 0
  0x06,  // 1
  0x5B,  // 2
  0x4F,  // 3
  0x66,  // 4
  0x6D,  // 5
  0x7D,  // 6
  0x07,  // 7
  0x7F,  // 8
  0x6F,  // 9
  0x77,  // A
  0x7C,  // b
  0x39,  // C
  0x5E,  // d
  0x79,  // E
  0x71,  // F
  0x00   // blank (index 16)
};

void write7Seg(byte pattern) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, pattern);
  digitalWrite(latchPin, HIGH);
}

/* Prototype functions */
void connectWifi();
void setupMQTT();
void connectMQTT();
void sendHeartbeatMessage();
void sendRingRingMessage(int destNode);
void registerForTopics();
void processMQTTMessage(char*, byte*, unsigned int);
void BIST();

void setup() {
  // Prof Note:  I have provided a setup that is similar to what I have
  // suggested in the past.
  //             This will not be complete.  Feel free to modify as needed for
  //             your design.
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial connection
    delay(1);
  }
  Serial.println("Serial ready!");

  // Button setup
  pbButton.attach(pdButton, INPUT_PULLUP);
  pbButton.interval(DEBOUNCE_INTERVAL);
  pbButton.setPressedState(LOW);

  // 74HC595 pins
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Rotary encoder setup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachSingleEdge(dtPin, clkPin);
  encoder.setCount(0);

  // Prof Note:  With complicated projects, I like having a
  // built in self test (BIST) for the 1st time I run my hardware.
  // Simply comment this out after your hardware checks.
  BIST();

  // setup connections
  connectWifi();
  setupMQTT();
  connectMQTT();
  // Flash the on-board LED five times to let user know
  // that the Huzzah32 board has been initialized and ready to go.
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, 0);  // active low
    delay(200);
    digitalWrite(LED_BUILTIN, 1);
    delay(150);
  }
}

void loop() {
  // Prof Note:  I have provided a basic loop similar to what I have suggested
  // in the past.
  //             This will not be complete.  Feel free to modify as needed for
  //             your design.

  // Prof Note2:  I have included working loop code for connection checks
  // and heartbest.  You will need to complete the code for the
  // sendHeartbeatMessage() to have this working properly.
  mscurrent = millis();
  // This is largely a reactive program, and as such only uses
  // the main loop to maintain the MQTT broker connection and
  // regularly call the psClient.loop() code to check for new
  // messsages
  psClient.loop();
  // reconnect to MQTT server if connection lost
  if ((mscurrent - mslastMQTTCheck) > MQTT_CONNECT_CHECK) {
    mslastMQTTCheck = mscurrent;
    MQTTConnected = psClient.connected();
    if (!MQTTConnected) {
      Serial.println("MQTT not connected! Trying reconnect.");
      connectMQTT();
    }
  }
  // send heartbeat (HEARTBEAT_INTERVAL = 0 means no heartbeat)
  if (((mscurrent - mslastHeartbeat) > HEARTBEAT_INTERVAL) &&
      HEARTBEAT_INTERVAL != 0 && MQTTConnected) {
    mslastHeartbeat = mscurrent;
    sendHeartbeatMessage();
  }

  // Read rotary encoder and update 7-seg display
  long encoderCount = encoder.getCount();
  if (encoderCount != lastEncoderCount) {
    delay(5);  // let encoder signals settle before acting
    encoderCount = encoder.getCount();  // re-read after settling
    if (encoderCount != lastEncoderCount) {
      int delta = (encoderCount > lastEncoderCount) ? 1 : -1;
      lastEncoderCount = encoderCount;
      currentDisplay += delta;
      if (currentDisplay > BLANK_7SEG_NUMBER) currentDisplay = 0;
      if (currentDisplay < 0) currentDisplay = BLANK_7SEG_NUMBER;
      write7Seg(SEG7[currentDisplay]);
    }
  }

  // Poll pushbutton and send ringring if pressed
  pbButton.update();
  if (pbButton.pressed() && currentDisplay != BLANK_7SEG_NUMBER) {
    sendRingRingMessage(currentDisplay);
  }

  // ProfNote:  I gave you finished network check and heartbeat.
  // You will need to fill out the loop for anything else you need to
  // make your program work!
}

/**********************************************************
 * Helper functions - Provided by Professor
 *********************************************************/
void connectWifi() {
  // ProfNote:  Provided fully functional.
  // attempt to connect to the WiFi network
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" network");
  delay(10);
#ifdef LIPSCOMB
  WiFi.begin(ssid);  // Lipscomb WiFi does NOT require a password
#elif defined(ETHERNET)
  WiFi.begin(ssid);
#else
  WiFi.begin(ssid, password);  // For WiFi networks that DO require a password
#endif

  // advance a "dot-dot-dot" indicator until connected to WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // report to console that WiFi is connected and print IP address
  Serial.print("MAC address = ");
  Serial.print(WiFi.macAddress());
  Serial.print(", connected as ");
  Serial.print(WiFi.localIP());
  Serial.println(".");
}
void setupMQTT() {
  // ProfNote:  Provided fully functional.
  // specify MQTT broker's domain name (or IP address) and port number
  Serial.print("Initalizing MQTT object with broker=");
  Serial.print(mqttBroker);
  Serial.print(" and port=");
  Serial.print(mqttPort);
  Serial.print("..");
  psClient.setServer(mqttBroker, mqttPort);

  // Specify callback function to process messages from the broker.
  psClient.setCallback(processMQTTMessage);
  Serial.println(".done");
}
void connectMQTT() {
  // ProfNote:  Provided fully functional.
  // Ping the server before trying to reconnect
  int WiFistatus = WiFi.status();
  if (WiFistatus != WL_CONNECTED) {
    Serial.println("WiFi check failed!  Trying to reconnect...");
    connectWifi();
  } else {
    // Serial.println("passed!");
    //  Try to connect to the MQTT broker (let loop() take care of retries)
    Serial.print("Connecting to MQTT with nodeName=");
    Serial.print(nodeName);
    Serial.print(" ... ");
    if (psClient.connect(nodeName.c_str())) {
      Serial.println("connected.");
      // clientID (<nodename>) MUST BE UNIQUE for all connected clients
      // can also include username, password if broker requires it
      // (e.g. psClient.connect(clientID, username, password)
      // once connected, register for topics of interest
      registerForTopics();
      sprintf(sbuf, "MQTT initialization complete\r\nReady!\r\n\r\n");
      Serial.print(sbuf);
    } else {
      // reconnect failed so print a console message, wait, and try again
      Serial.println(" failed!");
      Serial.print("MQTT client state=");
      Serial.println(psClient.state());
      Serial.print("(Is processor whitelisted?  ");
      Serial.print("MAC=");
      Serial.print(WiFi.macAddress());
      Serial.println(")");
    }
  }
}
/**********************************************************
 * ENDHelper functions - Provided by Professor
 *********************************************************/

void registerForTopics() {
  // register with MQTT broker for topics of interest to this node
  Serial.print("Registering for topics...");

  for (int i = 0; i < numTopics; i++) {
    psClient.subscribe(topics[i].c_str());
    Serial.print(" Subscribed to: ");
    Serial.println(topics[i]);
  }

  Serial.println("Registration done.");
}

void sendRingRingMessage(int destNode) {
  // topic: "ece/nodeDD/ringring"
  // payload: {"srcNode":"node04","dstNode":"nodeDD"}

  char destStr[7];
  sprintf(destStr, "node%02d", destNode);

  JsonDocument doc;
  doc["srcNode"] = nodeName;
  doc["dstNode"] = destStr;
  char payload[100];
  serializeJson(doc, payload);

  char topic[30];
  sprintf(topic, "ece/%s/ringring", destStr);
  psClient.publish(topic, payload);

  Serial.print("Sent ringring to: ");
  Serial.println(destStr);
}

void processMQTTMessage(char* topic, byte* json_payload, unsigned int length) {
  // This code is called whenever a message previously registered for is
  // RECEIVED from the broker. Incoming messages are selected by topic,
  // then the payload is parsed and appropriate action taken. (NB: If only
  // a single message type has been registered for, then there's no need to
  // select on a given message type. In practice this may be rare though...)
  //
  // For info on sending MQTT messages inside the callback handler, see
  // https://github.com/bblanchon/ArduinoJson/wiki/Memory-model.
  //
  // NB: for guidance on creating proper jsonBuffer size for the json object
  // tree, see https://github.com/bblanchon/ArduinoJson/wiki/Memory-model

  // process messages by topic
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // null-terminate the payload so we can parse it as a string
  char payload[length + 1];
  memcpy(payload, json_payload, length);
  payload[length] = '\0';

  // build expected topic strings for comparison
  String ringringTopic = "ece/" + nodeName + "/ringring";
  String topicsTopic   = "ece/" + nodeName + "/topics";

  if (String(topic) == ringringTopic) {
    // parse srcNode from payload: {"srcNode":"nodeNN","dstNode":"nodeNN"}
    JsonDocument doc;
    deserializeJson(doc, payload);
    const char* srcNode = doc["srcNode"];

    // extract the node number from "nodeNN" (last 2 chars)
    int srcID = atoi(srcNode + 4);  // skip "node" prefix
    Serial.print("Ring-ring from: ");
    Serial.println(srcNode);

    // display sender ID on 7-seg and update currentDisplay
    currentDisplay = srcID;
    write7Seg(SEG7[currentDisplay]);

    // play ring-ring tone
    buzzer.tone(1300, 100);
    buzzer.tone(1800, 300);
    delay(400);
    buzzer.tone(1300, 100);
    buzzer.tone(1800, 300);

  } else if (String(topic) == topicsTopic) {
    // respond with registeredFor message
    JsonDocument doc;
    doc["NodeName"] = nodeName;
    JsonArray arr = doc["topics"].to<JsonArray>();
    for (int i = 0; i < numTopics; i++) {
      arr.add(topics[i]);
    }
    char response[200];
    serializeJson(doc, response);
    String responseTopic = "ece/" + nodeName + "/registeredFor";
    psClient.publish(responseTopic.c_str(), response);
    Serial.println("Sent registeredFor response.");
  }
}
void sendHeartbeatMessage() {
  // message topic: "ece/nodeNN/heartbeat"
  // payload: {"NodeName":"nodeNN","NodeType":"ESP32"}

  JsonDocument doc;
  doc["NodeName"] = nodeName;
  doc["NodeType"] = nodeType;
  char payload[100];
  serializeJson(doc, payload);

  String topic = "ece/" + nodeName + "/heartbeat";
  psClient.publish(topic.c_str(), payload);

  Serial.println("Sent heartbeat.");
}

void BIST() {
  Serial.println("BIST: Walking 7-seg display through blank, 0-F, blank...");

  write7Seg(SEG7[BLANK_7SEG_NUMBER]);
  delay(500);

  for (int i = 0; i <= 15; i++) {
    Serial.printf("  Showing: %X\n", i);
    write7Seg(SEG7[i]);
    delay(600);
  }

  write7Seg(SEG7[BLANK_7SEG_NUMBER]);
  Serial.println("BIST: 7-seg test complete.");

  // Buzzer test: two-tone ring-ring
  Serial.println("BIST: Testing buzzer...");
  buzzer.tone(1300, 100);
  buzzer.tone(1800, 300);
     delay(400);
  buzzer.tone(1300, 100);
  buzzer.tone(1800, 300);
     delay(400);
  Serial.println("BIST: Buzzer test complete.");

  // Encoder test: wait for CW turn, CCW turn, then button press
  Serial.println("BIST: Encoder test -- turn CW...");
  encoder.setCount(0);
  while (encoder.getCount() <= 0) { delay(10); }
  Serial.println("  CW detected!");

  Serial.println("BIST: Encoder test -- turn CCW...");
  encoder.setCount(0);
  while (encoder.getCount() >= 0) { delay(10); }
  Serial.println("  CCW detected!");

  Serial.println("BIST: Encoder test -- press the button...");
  while (true) {
    pbButton.update();
    if (pbButton.pressed()) break;
    delay(PB_UPDATE_TIME);
  }
  Serial.println("  Button press detected!");
  Serial.println("BIST: Encoder test complete.");
}