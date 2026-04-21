// This file accompanies the Lab07.cpp program
// A simple ESP32 messaging system using MQTT

// system defines
#define DEBOUNCE_INTERVAL 5       // 5mS works well for circuit-mount PBs
#define HEARTBEAT_INTERVAL 30000  // heartbeat interval (0=no heartbeat)
#define MQTT_CONNECT_CHECK 60000  // Connection check
#define PB_UPDATE_TIME 8          // number of mS between button status checks
#define SWITCH_OPEN 0
#define SWITCH_CLOSED 1

#define MIN_NODE_NUMBER 0
#define MAX_NODE_NUMBER 15
#define BLANK_7SEG_NUMBER 16

// Define PINs
#define buzzerPin 25  // buzzer for individual sender tones
#define dataPin 4     // DS pin of 74HC595
#define latchPin 5    // ST_CP pin of 74HC595 (RCLK)
#define clockPin 18   // SH_CP pin of 74HC595
#define pdButton 16   // Encoders PB button
#define dtPin 17      // the dt (encoder "B") pin
#define clkPin 21     // the clk (encoder "A") pin
// led activation values (active high)
#define LED_ON 1
#define LED_OFF 0

// uncomment for Lipscomb broker, comment out for "brokerX"
// #define LIPSCOMB
#define ETHERNET
// #define HUTTONHOME

// Network and MQTT broker credentials
#ifdef LIPSCOMB
const char* mqttBroker = "10.51.97.101";  // ECE mosquitto server (wlan0)
const char* ssid = "LipscombGuest";  // no PW needed for Lipscomb guest wifi
#elif defined(ETHERNET)
const char* mqttBroker = "10.200.97.100";  // Wired ECE mosquitto server (eth0)
const char* ssid = "LipscombGuest";        // same as straight WiFi
#elif defined(HUTTONHOME)
const char* mqttBroker = "192.168.0.251";  // address of brokerX
const char* ssid = "HuttonWireless2-4G";   // ssid of brokerX's network
const char* password = "Testing01";        // password for brokerX's network
#else
const char* mqttBroker = "1.1.1.1";
const char* ssid = "";
const char* password = "password";  // password for brokerX's network
#endif
const int mqttPort = 1883;

// get ID of this node's ESP32 (undocumented: last three bytes of MAC address)
// this can be useful when making unique client ID values
// const uint32_t ID = ESP.getChipId();

// ECE node name and type
#define myNodeID "node02"
//compile_error_clear_after_updateing_XX_above();
String nodeName = myNodeID;
String nodeType = "ESP32";

// message topics to register for
// Using string literal tricks.  "txt1" "-txt2" = "txt1-txt2"
// ProfNote:  This is one way to do this!  You can choose other
// options if this does not work easily for you.
const int numTopics = 2;  // Set to match the topics array
String topics[] = {"ece/" myNodeID "/ringring\0", "ece/" myNodeID "/topics\0"};