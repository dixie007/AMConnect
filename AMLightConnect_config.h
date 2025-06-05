// Wifi settings
const char* ssid = "xxxxxxxxxxxx";
const char* password = "xxxxxxxxxxxx";

// mqtt settings
const char* mqtt_server = "xxxxxxxxxxxx";   // e.g. mqtt.eclipseprojects.io
const int mqtt_port = xxxx;  // e.g. 1883
const char* mqtt_username = "xxxxxxxxxxxx";
const char* mqtt_password = "xxxxxxxxxxxx";
const char* mqtt_command_topic = "xxxxxxxxxxxx/command";
const char* mqtt_debug_topic = "xxxxxxxxxxxx/debug";
const char* mqtt_uptime_topic = "xxxxxxxxxxxx/uptime";
const char* mqtt_rssi_topic = "xxxxxxxxxxxx/rssi";
const char* mqtt_freeheap_topic = "xxxxxxxxxxxx/freeheap";
const char* mqtt_ip_topic = "xxxxxxxxxxxx/ip";
const char* mqtt_status_topic = "xxxxxxxxxxxx/status";
const char* mqtt_preferences_topic = "xxxxxxxxxxxx/preferences";
const char* mqtt_prefstatus_topic = "xxxxxxxxxxxx/prefstatus";
const char* mqtt_lwt_topic = "xxxxxxxxxxxx/lwt";


// Define the debug serial port
#define DEBUG_PORT Serial

//Define the poll interval in seconds
#define pollInterval 60

// Defines if the values for pollInterval & gpsInterval should be read from internal memory in the device
// 1 = read internal values, 0 = overwrite internal values with config-readInternalValues
// This can be used for setting the values over MQTT by sending pollInterval:xx on topic automower/preferences
// and is stored in the device after a reboot.
#define readStoredPreferences 0 