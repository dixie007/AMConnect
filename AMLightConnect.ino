//======================================================================
//  Program: AMConnect
//
//  Description:  Connects Husqvarna Automower Generaton 2 to MQTT.
//                It allows status to be sent to MQTT and commands
//                to be sent to the Automower. 
//
//
//  Requirements: Supported Automower (Generation 2)
//                ESP8266 WEMOS D1 Mini Pro
//                
//
//======================================================================
#include <Arduino.h>

// ESP8266 statt ESP32 Bibliotheken 
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <uptime_formatter.h>
#include <SoftwareSerial.h>

// Include config
#include "AMLightConnect_config.h"



// include stuff
#include <AMConnect.h>


// Define the client
WiFiClient espClient;
PubSubClient client(espClient);

Preferences preferences;

// Restart and reconnect variables
const unsigned long BAUD_AM = 9600;

// Timer variables for status polling
unsigned long pollMillis;
unsigned int localPollInterval;

uint8_t lastCommand[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; 

// Timer for Restart
unsigned long startupMillis;
unsigned long restartInterval = 3600000;
bool restartIntervalActive = false;

// Timer for Wifi reconnect
unsigned long wifiPreviousMillis = 0;
unsigned long wifiInterval = 30000;
unsigned long wifiReconnectCount = 0;
unsigned long wifiReconnectRestartInterval = 60;

// Timer for MQTT reconnect
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long mqttReconnectInterval = 5000; // 5 Sekunden zwischen Verbindungsversuchen
unsigned long mqttReconnectCount = 0;

bool enableSerialDebug = false;
bool enableMQTTDebug = true;
bool stopStatus = false; // Track whether stop mode is active

#define D1_PIN 5  // GPIO5 = D1 auf D1 Mini Pro
#define D2_PIN 4  // GPIO4 = D2 auf D1 Mini Pro
// D1 & D2 Best Pins to Use – ESP8266 see also https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/

// Define Software Serial D2 == RX and D1 == TX
SoftwareSerial mySoftSerial(D2_PIN, D1_PIN); // RX, TX


void setup()
{
  DEBUG_PORT.begin(9600);
  while (!Serial)
    ;
  DEBUG_PORT.print( F("AMConnect: started\n") );

  startupMillis = millis();

  // Pins explizit konfigurieren
  pinMode(D1_PIN, OUTPUT); // TX Pin als Ausgang
  pinMode(D2_PIN, INPUT);  // RX Pin als Eingang
  
  // Starte die Software-Serial auf den neuen Pins to AutoMower
  mySoftSerial.begin(BAUD_AM);
  delay(10000);

  // Start wifi 
  setup_wifi();

  // Connect MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Read/Save preferences at startup
  savePreferences();
}

//--------------------------


// Main loop
void loop()
{

  if (restartIntervalActive && millis() - startupMillis >= restartInterval) 
  {
    handle_debug(true, (String)"Restarting in 10 seconds...");
    delay(10000);
    ESP.restart();
  }

  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (millis() - wifiPreviousMillis >=wifiInterval)) 
  {
    reconnect_wifi();
  }
  // Check if MQTT is connected 
  if (!client.connected()) 
  {
    reconnect();
  }
  // Do the MQTT Client Loop
  client.loop();
  
  // Korrektur: mySoftSerial statt Serial1 verwenden
  if(mySoftSerial.available())
  {
    // Handle the automower
    handle_am();
  }

  // Poll mower for status
  if (millis() - pollMillis >= localPollInterval*1000) 
  {
    // do a poll for status
    handle_command("getStatus");
    pollMillis = millis();
  }

}

// Functions

void savePreferences() 
{
    preferences.begin("amPreferences", false);
    if(readStoredPreferences == 1)
    {
      handle_debug(false, (String)"Using stored values from preferences, skipping config values...");
      //Poll-interval
      localPollInterval = preferences.getUInt("pollInterval");
      handle_debug(true, "stored pollInterval is set to: " + preferences.getUInt("pollInterval"));
      
    }
    else
    {
      handle_debug(false, (String)"Using values from config, overwriting preferences...");

      //Poll-interval
      preferences.putUInt("pollInterval", pollInterval);
      localPollInterval = pollInterval;
      
    }
    preferences.end();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  handle_debug(false, "");

  WiFi.begin(ssid, password);
  delay(5000);
  String ip = WiFi.localIP().toString();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PORT.print(".");
  }
  
  handle_debug(false, "");
}


void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;

  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  handle_debug(false, (String)"Message arrived on topic: " + (String)topic + (String)". Message: " + (String)messageTemp);

  //Commands
  if ((String)topic == (String)mqtt_command_topic) {
    // We got a command, lets handle it!
    handle_command(messageTemp);
    handle_debug(false, "Got command via MQTT");
  }

  //Preferences
  if ((String)topic == (String)mqtt_preferences_topic) {
    // We got a preferences, lets handle it!
    handle_preferences(messageTemp);
    handle_debug(false, "Got a preference via MQTT");
  }
}

void reconnect_wifi() {
    // if WiFi has not been able to reconnect in X tries, reboot just to clear up everything
    if ( wifiReconnectCount >= wifiReconnectRestartInterval ) 
    {
        handle_debug(true, (String)"Restarting in 10 seconds...");
        delay(10000);
        ESP.restart();
    }
    else {
         handle_debug(false, (String)millis());
         handle_debug(false, (String)"Reconnecting to WiFi...");
         WiFi.disconnect();
         WiFi.reconnect();
         wifiPreviousMillis = millis();
         wifiReconnectCount = wifiReconnectCount + 1; 
    }
}

void reconnect() 
{
  // Nicht-blockierende Wiederverbindung implementieren
  if (!client.connected()) 
  {
    // WLAN-Verbindung prüfen und ggf. wiederherstellen
    if (WiFi.status() != WL_CONNECTED && millis() - wifiPreviousMillis >= wifiInterval) 
    {
      reconnect_wifi();
      return; // Warte bis WLAN verbunden ist, bevor MQTT versucht wird
    }
    
    // MQTT-Verbindung nur alle mqttReconnectInterval Millisekunden versuchen
    if (millis() - lastMqttReconnectAttempt >= mqttReconnectInterval) 
    {
      lastMqttReconnectAttempt = millis();
      handle_debug(false, "Attempting MQTT connection...");
      
      // Verbindungsversuch mit eindeutiger Client-ID
      char clientId[20];
      sprintf(clientId, "AMClient-%d", random(1000)); // Zufällige Client-ID, um Konflikte zu vermeiden
      
      if (client.connect(clientId, mqtt_username, mqtt_password, mqtt_lwt_topic, 1, true, "Offline", 0)) 
      {
        handle_debug(false, "MQTT connected");
        // Topics abonnieren
        client.subscribe(mqtt_command_topic, 1);
        client.subscribe(mqtt_preferences_topic);
        // LWT auf Online setzen
        client.publish(mqtt_lwt_topic, "Online", true);
        mqttReconnectCount = 0; // Zurücksetzen bei erfolgreicher Verbindung
      } 
      else 
      {
        char buffer[50];
        sprintf(buffer, "MQTT failed, rc=%d. Will retry...", client.state());
        handle_debug(false, buffer);
        mqttReconnectCount++;
        
        // Optionale Fehlerbehandlung nach zu vielen fehlgeschlagenen Versuchen
        if (mqttReconnectCount > 20) {
          handle_debug(false, "Too many MQTT reconnect attempts, restarting...");
          ESP.restart(); // Neustart des ESP nach zu vielen Versuchen
        }
      }
    }
  }
}

void handle_debug(bool sendmqtt, String debugmsg) 
{
  // Handle the debug output
  if (enableSerialDebug)
  {
    DEBUG_PORT.println(debugmsg);
  }
  
  // send to mqtt_command_topic
  if (enableMQTTDebug && sendmqtt && client.connected())
  {
    char debugChar[50];
    debugmsg.toCharArray(debugChar,50);
    client.publish(mqtt_debug_topic, debugChar);
  }
}

void handle_status(int statusCode, String statusMsg) 
{
  // Send to debug
  handle_debug(true, (String)"Status: " + statusMsg);
  
  // send to mqtt_status_topic
  if (client.connected())
  {
    char statusChar[50];
    statusMsg.toCharArray(statusChar,50);
    client.publish(mqtt_status_topic, statusChar);
  }
}

void handle_uptime() 
{
  String uptime = (String)uptime_formatter::getUptime();
  
  // send to mqtt_debug_topic
  if (client.connected())
  {
    char statusChar[50];
    uptime.toCharArray(statusChar,50);
    //client.publish(mqtt_debug_topic, statusChar);
    handle_debug(true, (String)"Up: " + uptime);
    client.publish(mqtt_uptime_topic, statusChar);
  }
}

void handle_wifi_rssi() 
{
  String rssi = (String)WiFi.RSSI();
  
  // send to mqtt_debug_topic
  if (client.connected())
  {
    char statusChar[50];
    rssi.toCharArray(statusChar,50);
    //client.publish(mqtt_debug_topic, statusChar);
    handle_debug(true, (String)"RSSI: " + rssi);
    client.publish(mqtt_rssi_topic, statusChar);
  }
}

void handle_wifi_ip() 
{
  String ip = WiFi.localIP().toString();
    
  // send to mqtt_debug_topic
  if (client.connected())
  {
    char statusChar[50];
    ip.toCharArray(statusChar,50);
    //client.publish(mqtt_debug_topic, statusChar);
    handle_debug(true, (String)"IP-adress: " + ip);
    client.publish(mqtt_ip_topic, statusChar);
  }
}

void handle_free_heap() 
{
  String freeHeap = (String)ESP.getFreeHeap();
  
  // send to mqtt_debug_topic
  if (client.connected())
  {
    char statusChar[50];
    freeHeap.toCharArray(statusChar,50);
    //client.publish(mqtt_debug_topic, statusChar);
    handle_debug(true, (String)"FreeHeap: " + freeHeap);
    client.publish(mqtt_freeheap_topic, statusChar);
  }
  
}

void handle_wifi_reconnect_count() 
{
  String count = (String)wifiReconnectCount;
  
  // send to mqtt_debug_topic
  if (client.connected())
  {
    char statusChar[50];
    count.toCharArray(statusChar,50);
    //client.publish(mqtt_debug_topic, statusChar);
    handle_debug(true, (String)"WiFi Reconnect Count: " + count);

  }
  
}

void handle_mqtt_reconnect_count() 
{
  String count = (String)mqttReconnectCount;
  
  // send to mqtt_debug_topic
  if (client.connected())
  {
    char statusChar[50];
    count.toCharArray(statusChar,50);
    //client.publish(mqtt_debug_topic, statusChar);
    handle_debug(true, (String)"MQTT Reconnect Count: " + count);
  }
  
}


void setStop(bool stop) {
  if(stop) {
    mySoftSerial.write(STOP_ON_DATA, sizeof(STOP_ON_DATA));
    handle_debug(true, "Stop mode activated");
  } else {
    mySoftSerial.write(STOP_OFF_DATA, sizeof(STOP_OFF_DATA));
    handle_debug(true, "Stop mode deactivated");
  }
}

void handle_am() 
{
  uint8_t statusAutomower[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  uint8_t empty[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; 
  
  // Daten vom RX-Pin (D2) lesen
  mySoftSerial.readBytes(statusAutomower, 5);

  if(memcmp(statusAutomower, empty, 5) == 0) 
  {
    handle_debug(true, "Unusable data received on serial");
  } 
  else 
  {
    // values comes as DEC and not HEX
    handle_debug(true, "Byte1: " + (String)(statusAutomower[0]) + " Byte2: " + (String)(statusAutomower[1]) + " Byte3: " + (String)(statusAutomower[2]) + " Byte4: " + (String)(statusAutomower[3]) + " Byte5: " + (String)(statusAutomower[4]));
   
  }

  // Merge the last two bytes to status
  unsigned int statusInt = statusAutomower[4] << 8 | statusAutomower[3];
  handle_debug(true, "Status Code: " + (String)statusInt);

  if (statusAutomower[0] == 0x0F) 
  {
    // A command

    if (statusAutomower[1] == 0x80) 
    {
      // Keypresses
      if (statusAutomower[2] == 0x5F) 
      {
        // Keypress
        switch (statusInt) 
        {
          case 0: 
            handle_debug(true, "Key 0 pressed"); 
            break;
          case 1: 
            handle_debug(true, "Key 1 pressed"); 
            break;
          case 2: 
            handle_debug(true, "Key 2 pressed");  
            break;
          case 3: 
            handle_debug(true, "Key 3 pressed");  
            break;
          case 4: 
            handle_debug(true, "Key 4 pressed");  
            break;
          case 5: 
            handle_debug(true, "Key 5 pressed");  
            break;
          case 6: 
            handle_debug(true, "Key 6 pressed"); 
            break;
          case 7: 
            handle_debug(true, "Key 7 pressed");  
            break;
          case 8: 
            handle_debug(true, "Key 8 pressed");  
            break;
          case 9: 
            handle_debug(true, "Key 9 pressed"); 
            break;
          case 10: 
            handle_debug(true, "Key Program A pressed");  
            break;
          case 11: 
            handle_debug(true, "Key Program B pressed"); 
            break;
          case 12: 
            handle_debug(true, "Key Program C pressed"); 
            break;
          case 13:  
            handle_debug(true, "Key Home pressed");
            break;
          case 14: 
            handle_debug(true, "Key Man/Auto pressed"); 
            break;
          case 15: 
            handle_debug(true, "Key C pressed");
            break;
          case 16: 
            handle_debug(true, "Key Up pressed");
            break;
          case 17: 
            handle_debug(true, "Key Down pressed"); 
            break;
          case 18: 
            handle_debug(true, "Key YES pressed");
            break;
          default: //no valid parameter: send status
            handle_debug(true, "Unkown keypress. Code: " + (String)statusInt);
            break;
        }
      }
    }
    else if (statusAutomower[1] == 0x81) 
    {
      // Mode set
      if (statusAutomower[2] == 0x2C) 
      {
        // Mode Set
        switch (statusInt) 
        {
          case 0: 
            handle_debug(true, "Manual Mode"); 
            break;
          case 1: 
            handle_debug(true, "Auto Mode"); 
            break;
          case 3: 
            handle_debug(true, "Home Mode");
            break;
          case 4: 
            handle_debug(true, "Demo Mode"); 
            break;
          default: //no valid parameter: send status
            handle_debug(true, "Unkown keypress. Code: " + (String)statusInt);
            break;
        }
      }
    }
    else if (statusAutomower[1] == 0xCA) 
    {
      // Timer actions
      if (statusAutomower[2] == 0x4E) 
      {
        // Timer actions
        switch (statusInt) 
        {
          case 0: 
            handle_debug(true, "Timer activated");
            break;
          case 1: 
            handle_debug(true, "Timer deactivated");
            break;
          default: //no valid parameter: send status
            handle_debug(true, "Unknown timer action. Code: " + (String)statusInt);
            break;
        }
      }
    }
    else if (statusAutomower[1] == 0x00)
    {
      // Mowing time and strength
      if (statusAutomower[2] == 0x38) 
      {
        // Mowing time
        handle_debug(true, "Mowing time: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x4D) 
      {
        // rpm
        handle_debug(true, "Mowing rpm: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1]  == 0x01)
    {
      // Info, Battery and square mode
      if (statusAutomower[2] == 0x34) 
      {
        // Square mode procent
        handle_debug(true, "Square mode procent: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x37) 
      {
        // Square mode reference
        handle_debug(true, "Square mode reference: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x38) 
      {
        // Square mode status
        handle_debug(true, "Square mode status: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xEB) 
      {
        // Battery capacity (mA)
        handle_debug(true, "Battery capacity (mA): " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xEC) 
      {
        // Charging time
        handle_debug(true, "Charging time: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xEF) 
      {
        // Battery capacity mAh
        handle_debug(true, "Battery capacity mAh: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xF0) 
      {
        // Battery seek start capacity
        handle_debug(true, "Battery seek start capacity: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xF1) 
      {
        // Status
        switch (statusInt) 
        {
          case 6: //Status
            handle_status(statusInt, "Linker Radmotor blockiert"); // Left wheel motor blocked
            break;
          case 12: //Status
            handle_status(statusInt, "Kein Schleifensignal"); // No loop signal
            break;
          case 14: //Status
            handle_status(statusInt, "Festgefahren"); // Stuck
            break;
          case 16: //Status
            handle_status(statusInt, "Ausserhalb"); // Outside working area
            break;
          case 18: //Status
            handle_status(statusInt, "Niedrige Batteriespannung"); // Low battery voltage
            break;
          case 24: //Status
            handle_status(statusInt, "Schlupf bei Rad"); // Wheel spinning
            break;
          case 26: //Status
            handle_status(statusInt, "Ladestation blockiert"); // Charging station blocked  
            break;
          case 28: //Status
            handle_status(statusInt, "Benötigt manuelles laden"); // Manual charging needed
            break;
          case 30:
            handle_status(statusInt, "Messerteller blockiert"); // CDB Cutting disc blocked 1e
            break;
          case 32:
            handle_status(statusInt, "Messerteller blockiert"); // CD2 Cutting disc blocked 20
            break;
          case 34: //Status
            handle_status(statusInt, "Mäher hochgehoben"); // Mower lifted
            break;
          case 52: //Status
            handle_status(statusInt, "Kein Kontakt in der Ladestation"); // Charging station no contact
            break;
          case 54: //Status
            handle_status(statusInt, "Pin abgelaufen"); // Pin expired
            break;
          case 56: //Status
            handle_status(statusInt, "Linker Stossensor defekt"); // Left collision sensor defective
            break;
          case 58: //Status
            handle_status(statusInt, "Rechter Stossensor defekt"); // Right collision sensor defective
            break;
          case 1000: //Status
            handle_status(statusInt, "Verlässt Ladestation"); // Leaves charging station
            break;
          case 1002: //Status
            handle_status(statusInt, "Mähen"); // Mowing
            break;
          case 1006: //Status
            handle_status(statusInt, "Mähwerk starten"); // Mowing starting
            break;
          case 1008: //Status
            handle_status(statusInt, "Mähwerk gestartet"); // Mower started
            break;
          case 1012: //Status
            handle_status(statusInt, "Starte Mähen"); // Start mowing
            break;
          case 1014: //Status
            handle_status(statusInt, "Laden"); // Charging
            break;
          case 1016: //Status
            handle_status(statusInt, "Wartet in Ladestation");  // Waiting in charging station 
            break;
          case 1024: //Status
            handle_status(statusInt, "Fährt zur Ladestation"); // Drives to charging station
            break;
          case 1036: //Status
            handle_status(statusInt, "Viereckmodus"); // Square mode
            break;
          case 1038: //Status
            handle_status(statusInt, "Festgefahren"); // Stuck
            break;
          case 1040: //Status
            handle_status(statusInt, "Kollision"); // Collision
            break;
          case 1042: //Status
            handle_status(statusInt, "Suchen"); // Searching
            break;
          case 1044: //Status
            handle_status(statusInt, "Stop"); // Stop
            break;
          case 1048: //Status
            handle_status(statusInt, "Andocken"); // Docking
            break;
          case 1050: //Status
            handle_status(statusInt, "Verlässt Ladestation"); // Leaving charging station
            break;   
          case 1052: //Status
            handle_status(statusInt, "Fehler"); // Error
            break;
          case 1056: //Status
            handle_status(statusInt, "Wartet auf Einsatz"); // Waiting for use
            break;
          case 1058: //Status
            handle_status(statusInt, "Folgt Begrenzung"); // Following limit cable
            break;
          case 1060: //Status
            handle_status(statusInt, "N-Signal gefunden"); // N signal found
            break;
          case 1062: //Status
            handle_status(statusInt, "Festgefahren"); // Stuck
            break;
          case 1064: //Status
            handle_status(statusInt, "Suchen"); // Searching
            break;
          case 1070: //Status
            handle_status(statusInt, "Folgt Suchschleife"); // Following guide cable
            break;
          case 1072: //Status
            handle_status(statusInt, "Schleife folgen"); // Following loop
            break;
          default: //no valid parameter: send status
            handle_status(statusInt, "Unbekannter Status Code"); // Unknown statuscode
            break;
        }
      }
    }

    else if (statusAutomower[1] == 0x01 && statusAutomower[2] == 0x2F) 
    {
      // Handle stop status response
      if (statusInt == 0x0000) 
      {
        // case 0x0000 - Stop = false
        stopStatus = false;
        handle_debug(true, "Stop status: Inactive");
      } 
      else if (statusInt == 0x0002) 
      {
        // case 0x0002 - Stop = true
        stopStatus = true;
        handle_debug(true, "Stop status: Active");
      } 
      else 
      {
        //
        handle_debug(true, "Unknown stop status code: " + String(statusInt, HEX));
      }
    }
    
    else if (statusAutomower[1] == 0x02)
    {
      // Battery info
      if (statusAutomower[2] == 0x33) 
      {
        // 
        handle_debug(true, "Battery temperature: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x34) 
      {
        // 
        handle_debug(true, "Last charging time: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x35) 
      {
        // 
        handle_debug(true, "Battery charging temperature: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x36) 
      {
        // 
        handle_debug(true, "Battery next temperature reading: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1] == 0x24)
    {
      // Wheel speeds
      if (statusAutomower[2] == 0xBF) 
      {
        // 
        handle_debug(true, "Right wheel speed: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xC0) 
      {
        // 
        handle_debug(true, "Left wheel speed: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1] == 0x2E)
    {
      // Battery info
      if (statusAutomower[2] == 0xE0) 
      {
        // 
        handle_debug(true, "Battery Capacity Used (mAh): " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xEA) 
      {
        // 
        handle_debug(true, "Speed of Knife engine: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xF4) 
      {
        // 
        handle_debug(true, "Battery Voltage: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1] == 0x33) 
    {
      // Firmware version
      if (statusAutomower[2] == 0x90) 
      {
        // 
        handle_debug(true, "Firmware version: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1] == 0x36)
    {
      // time and date
      if (statusAutomower[2] == 0xB3) 
      {
        // 
        handle_debug(true, "Current minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xB5) 
      {
        // 
        handle_debug(true, "Current hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xB7) 
      {
        // 
        handle_debug(true, "Current day: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xB9) 
      {
        // 
        handle_debug(true, "Current month: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0xBD) 
      {
        // 
        handle_debug(true, "Current year: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1] == 0x3A)
    {
      // Voice version
      if (statusAutomower[2] == 0xC0) 
      {
        // 
        handle_debug(true, "Voice version: " + (String)statusInt);
      }
    }
    else if (statusAutomower[1] == 0x4A)
    {
      // Timers
      if (statusAutomower[2] == 0x38) 
      {
        // 
        handle_debug(true, "Week Timer1 Start Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x39) 
      {
        // 
        handle_debug(true, "Week Timer1 Start Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x3A) 
      {
        // 
        handle_debug(true, "Week Timer1 Stop Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x3B) 
      {
        // 
        handle_debug(true, "Week Timer1 Stop Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x3C) 
      {
        // 
        handle_debug(true, "Weekend Timer1 Start Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x3D) 
      {
        // 
        handle_debug(true, "Weekend Timer1 Start Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x3E) 
      {
        // 
        handle_debug(true, "Weekend Timer1 Stop Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x3F) 
      {
        // 
        handle_debug(true, "Weekend Timer1 Stop Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x40) 
      {
        // 
        handle_debug(true, "Week Timer2 Start Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x41) 
      {
        // 
        handle_debug(true, "Week Timer2 Start Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x42) 
      {
        // 
        handle_debug(true, "Week Timer2 Stop Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x43) 
      {
        // 
        handle_debug(true, "Week Timer2 Stop Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x44) 
      {
        // 
        handle_debug(true, "Weekend Timer2 Start Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x45) 
      {
        // 
        handle_debug(true, "Weekend Timer2 Start Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x46) 
      {
        // 
        handle_debug(true, "Weekend Timer2 Stop Hour: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x47) 
      {
        // 
        handle_debug(true, "Weekend Timer2 Stop Minute: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x4E) 
      {
        // 
        handle_debug(true, "Timer Status: " + (String)statusInt);
      }
      else if (statusAutomower[2] == 0x50) 
      {
        // 
        handle_debug(true, "Timer Day: " + (String)statusInt);
      }
    }
  }
}

void handle_preferences(String preferencePayload) 
{
  preferences.begin("amPreferences", false);

  if(preferencePayload.startsWith("gpsInterval:")) 
  { 
   }
    
  else if (preferencePayload.startsWith("pollInterval:"))
  { 
    preferences.putUInt("pollInterval", preferencePayload.substring(13).toInt());
    localPollInterval = preferencePayload.substring(13).toInt();
    handle_debug(true, "pollInterval is set to: " + String(preferences.getUInt("pollInterval")));  
    }
    
  else if( preferencePayload == "getGpsInterval")
  { 
   }
    
  else if( preferencePayload = "getPollInterval")
  {
    const byte pollSize =  sizeof localPollInterval;
    char pollChar[pollSize];
    itoa(localPollInterval, pollChar, 10);
    client.publish(mqtt_prefstatus_topic, pollChar); 
    handle_debug(true, "pollInterval is set to: " + String(preferences.getUInt("pollInterval"))); 
    }
    
  else 
  {
    handle_debug(true, "Unknown command recieved on automower/preferences topic: " + preferencePayload); 
    }
  
  preferences.end();
}

void handle_command(String command) 
{
  bool dowrite = true;
  uint8_t commandAutomower[5] = { 0x0F, 0x01, 0xF1, 0x00, 0x00 };

  if (command == "getMowingTime") { memcpy(commandAutomower, amcGetMowingTime, sizeof(commandAutomower)); }
  else if (command == "getMowingStrength") { memcpy(commandAutomower, amcGetMowingStrength, sizeof(commandAutomower)); }
  else if (command == "getSquareModeProcent") { memcpy(commandAutomower, amcGetSquareModeProcent, sizeof(commandAutomower)); }
  else if (command == "getSquareModeReference") { memcpy(commandAutomower, amcGetSquareModeReference, sizeof(commandAutomower)); }
  else if (command == "getSquareModeStatus") { memcpy(commandAutomower, amcGetSquareModeStatus, sizeof(commandAutomower)); }
  else if (command == "getBatteryCapacity") { memcpy(commandAutomower, amcGetBatteryCapacity, sizeof(commandAutomower)); }
  else if (command == "getBatteryChargingTime") { memcpy(commandAutomower, amcGetBatteryChargingTime, sizeof(commandAutomower)); }
  else if (command == "getBatteryCapacitymAh") { memcpy(commandAutomower, amcGetBatteryCapacitymAh, sizeof(commandAutomower)); }
  else if (command == "getBatterySeekStartCapacity") { memcpy(commandAutomower, amcGetBatterySeekStartCapacity, sizeof(commandAutomower)); }
  else if (command == "getStatus") { memcpy(commandAutomower, amcGetStatus, sizeof(commandAutomower)); }
  else if (command == "getBatteryTemperature") { memcpy(commandAutomower, amcGetBatteryTemperature, sizeof(commandAutomower)); }
  else if (command == "getBatteryLastChargingTime") { memcpy(commandAutomower, amcGetBatteryLastChargingTime, sizeof(commandAutomower)); }
  else if (command == "getBatteryChargingTemperature") { memcpy(commandAutomower, amcGetBatteryChargingTemperature, sizeof(commandAutomower)); }
  else if (command == "getBatteryNextTemperature") { memcpy(commandAutomower, amcGetBatteryNextTemperatureReading, sizeof(commandAutomower)); }
  else if (command == "getSpeedRight") { memcpy(commandAutomower, amcGetSpeedRight, sizeof(commandAutomower)); }
  else if (command == "getSpeedLeft") { memcpy(commandAutomower, amcGetSpeedLeft, sizeof(commandAutomower)); }
  else if (command == "getBatteryCapacityUsed") { memcpy(commandAutomower, amcGetBatteryCapacityUsed, sizeof(commandAutomower)); }
  else if (command == "getSpeedKnife") { memcpy(commandAutomower, amcGetSpeedKnife, sizeof(commandAutomower)); }
  else if (command == "getBatteryVoltage") { memcpy(commandAutomower, amcGetBatteryVoltage, sizeof(commandAutomower)); }
  else if (command == "getFirmwareVersion") { memcpy(commandAutomower, amcGetFirmwareVersion, sizeof(commandAutomower)); }
  else if (command == "getTimeSecond") { memcpy(commandAutomower, amcGetTimeSecond, sizeof(commandAutomower)); }
  else if (command == "getTimeMinute") { memcpy(commandAutomower, amcGetTimeMinute, sizeof(commandAutomower)); }
  else if (command == "getTimeHour") { memcpy(commandAutomower, amcGetTimeHour, sizeof(commandAutomower)); }
  else if (command == "getDateDay") { memcpy(commandAutomower, amcGetDateDay, sizeof(commandAutomower)); }
  else if (command == "getDateMonth") { memcpy(commandAutomower, amcGetDateMonth, sizeof(commandAutomower)); }
  else if (command == "getDateYear") { memcpy(commandAutomower, amcGetDateYear, sizeof(commandAutomower)); }
  else if (command == "getVoiceVersion") { memcpy(commandAutomower, amcGetVoiceVersion, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer1StartHour") { memcpy(commandAutomower, amcGetWeekTimer1StartHour, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer1StartMinute") { memcpy(commandAutomower, amcGetWeekTimer1StartMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer1StopHour") { memcpy(commandAutomower, amcGetWeekTimer1StopHour, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer1StopMinute") { memcpy(commandAutomower, amcGetWeekTimer1StopMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer1StartHour") { memcpy(commandAutomower, amcGetWeekendTimer1StartHour, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer1StartMinute") { memcpy(commandAutomower, amcGetWeekendTimer1StartMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer1StopHour") { memcpy(commandAutomower, amcGetWeekendTimer1StopHour, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer1StopMinute") { memcpy(commandAutomower, amcGetWeekendTimer1StopMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer2StartHour") { memcpy(commandAutomower, amcGetWeekTimer2StartHour, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer2StartMinute") { memcpy(commandAutomower, amcGetWeekTimer2StartMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer2StopHour") { memcpy(commandAutomower, amcGetWeekTimer2StopHour, sizeof(commandAutomower)); }
  else if (command == "getWeekTimer2StopMinute") { memcpy(commandAutomower, amcGetWeekTimer2StopMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer2StartHour") { memcpy(commandAutomower, amcGetWeekendTimer2StartHour, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer2StartMinute") { memcpy(commandAutomower, amcGetWeekendTimer2StartMinute, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer2StopHour") { memcpy(commandAutomower, amcGetWeekendTimer2StopHour, sizeof(commandAutomower)); }
  else if (command == "getWeekendTimer2StopMinute") { memcpy(commandAutomower, amcGetWeekendTimer2StopMinute, sizeof(commandAutomower)); }
  else if (command == "getTimerStatus") { memcpy(commandAutomower, amcGetTimerStatus, sizeof(commandAutomower)); }
  else if (command == "getTimerDay") { memcpy(commandAutomower, amcGetTimerDay, sizeof(commandAutomower)); }
  else if (command == "getMode") { memcpy(commandAutomower, amcGetModusAutomower, sizeof(commandAutomower)); }
  else if (command == "setKey0") { memcpy(commandAutomower, amcKey0, sizeof(commandAutomower)); }
  else if (command == "setKey1") { memcpy(commandAutomower, amcKey1, sizeof(commandAutomower)); }
  else if (command == "setKey2") { memcpy(commandAutomower, amcKey2, sizeof(commandAutomower)); }
  else if (command == "setKey3") { memcpy(commandAutomower, amcKey3, sizeof(commandAutomower)); }
  else if (command == "setKey4") { memcpy(commandAutomower, amcKey4, sizeof(commandAutomower)); }
  else if (command == "setKey5") { memcpy(commandAutomower, amcKey5, sizeof(commandAutomower)); }
  else if (command == "setKey6") { memcpy(commandAutomower, amcKey6, sizeof(commandAutomower)); }
  else if (command == "setKey7") { memcpy(commandAutomower, amcKey7, sizeof(commandAutomower)); }
  else if (command == "setKey8") { memcpy(commandAutomower, amcKey8, sizeof(commandAutomower)); }
  else if (command == "setKey9") { memcpy(commandAutomower, amcKey9, sizeof(commandAutomower)); }
  else if (command == "setKeyProgA") { memcpy(commandAutomower, amcKeyProgA, sizeof(commandAutomower)); }
  else if (command == "setKeyProgB") { memcpy(commandAutomower, amcKeyProgB, sizeof(commandAutomower)); }
  else if (command == "setKeyProgC") { memcpy(commandAutomower, amcKeyProgC, sizeof(commandAutomower)); }
  else if (command == "setKeyHome") { memcpy(commandAutomower, amcKeyHome, sizeof(commandAutomower)); }
  else if (command == "setKeyManAuto") { memcpy(commandAutomower, amcKeyManAuto, sizeof(commandAutomower)); }
  else if (command == "setKeyC") { memcpy(commandAutomower, amcKeyC, sizeof(commandAutomower)); }
  else if (command == "setKeyUp") { memcpy(commandAutomower, amcKeyUp, sizeof(commandAutomower)); }
  else if (command == "setKeyDown") { memcpy(commandAutomower, amcKeyDown, sizeof(commandAutomower)); }
  else if (command == "setKeyYes") { memcpy(commandAutomower, amcKeyYes, sizeof(commandAutomower)); }
  else if (command == "setModeManual") { memcpy(commandAutomower, amcModeManual, sizeof(commandAutomower)); }
  else if (command == "setModeAuto") { memcpy(commandAutomower, amcModeAuto, sizeof(commandAutomower)); }
  else if (command == "setModeHome") { memcpy(commandAutomower, amcModeHome, sizeof(commandAutomower)); }
  else if (command == "setModeDemo") { memcpy(commandAutomower, amcModeDemo, sizeof(commandAutomower)); }
  else if (command == "setMenuAdvanced") { memcpy(commandAutomower, amcMenuAdvanced, sizeof(commandAutomower)); }
  else if (command == "setMenuNormal") { memcpy(commandAutomower, amcMenuNormal, sizeof(commandAutomower)); }
  else if (command == "setTimerActivate") { memcpy(commandAutomower, amcTimerActivate, sizeof(commandAutomower)); }
  else if (command == "setTimerDeactivate") { memcpy(commandAutomower, amcTimerDeactivate, sizeof(commandAutomower)); }
  else if (command == "getUptime") { handle_uptime(); dowrite = false; }
  else if (command == "enableSerialDebug") { enableSerialDebug = true; dowrite = false; }
  else if (command == "disableSerialDebug") { enableSerialDebug = false; dowrite = false; }
  else if (command == "enableMQTTDebug") { enableMQTTDebug = true; dowrite = false; }
  else if (command == "disableMQTTDebug") { enableMQTTDebug = false; dowrite = false; }
  else if (command == "getWifiRSSI") { handle_wifi_rssi();  dowrite = false; }
  else if (command == "getWifiIP") { handle_wifi_ip();  dowrite = false; }
  else if (command == "getWifiReconnectCount") { handle_wifi_reconnect_count(); dowrite = false; }
  else if (command == "getMQTTReconnectCount") { handle_mqtt_reconnect_count(); dowrite = false; }
  else if (command == "getStopStatus") { memcpy(commandAutomower, amcReadStopCMD, sizeof(commandAutomower)); }
  else if (command == "setStopOn") { setStop(true); dowrite = false; }
  else if (command == "setStopOff") { setStop(false); dowrite = false; }

  if (dowrite)
  {
    // send it to the automower
    mySoftSerial.write(commandAutomower,sizeof(commandAutomower));
  }
  
}
