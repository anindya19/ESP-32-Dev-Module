/*
 * PROJECT: Home Automation with ESP-32 via Rainmaker, Google voice, Alexa
 * Organization: BITS Pilani, Goa campus
 * Degree: M.Tech, Embedded Systems
 * Author: Anindya Sundar Gyaen
 * E-Mail: anindya19@gmail.com
 * Supports: 8 Switch modules of 5A each
 * esp32 2.0.6 is only working (ESP32 Dev Module)
*/

/*
Relay
--------
D13	IN1
D15	IN2
D14	IN3
D27	IN4
D26	IN5
D25	IN6
D33	IN7
D32	IN8
*/

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <Preferences.h>
#include <esp_timer.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <time.h>

// For Turning On and Off the debug logs
#define DEBUG_SW 1

// By default ON/OFF sequence timing is limited to 3 Sec to factory reset for Wi-Fi provisioning
#define METHOD2_FACTORY_RST_SW_TIMING 3000

Preferences pref, pref2;

// Relay State
bool switch_state[8] = {LOW, LOW, LOW, LOW, LOW/*, LOW, LOW, LOW*/};

// BLE Credentials
const char *service_name = "PROV_home_automation"; // BLE node name
const char *pop = "123456"; // password

// Define the Node Name
char nodeName[] = "Smart_Home";

// GPIO for Relay (Appliance Control)
static uint8_t relays[] = {13, 15, 14, 27, 26/*, 25, 33, 32*/};

// GPIO for h/w reset pin & Temp/humidity sensor
static uint8_t gpio_reset = 0;   // Reset Pin

// Rainmaker side variables
static Switch *switches[8] = {
    new Switch("Switch1", &relays[0]),
    new Switch("Switch2", &relays[1]),
    new Switch("Switch3", &relays[2]),
    new Switch("Switch4", &relays[3]),
    new Switch("Switch5", &relays[4])/*,
    new Switch("Switch6", &relays[5]),
    new Switch("Switch7", &relays[6]),
    new Switch("Switch8", &relays[7])*/
};

// Wi-Fi connection handler
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id)
    {
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
            if (DEBUG_SW) Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
            printQR(service_name, pop, "ble");
#else
			Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      		WiFiProv.printQR(service_name, pop, "softap");
#endif
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            if (DEBUG_SW) Serial.print("\nConnected IP address : ");
            if (DEBUG_SW) Serial.println(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            if (DEBUG_SW) Serial.println("\nDisconnected. Connecting to the AP again...");
            WiFi.disconnect();
            WiFi.reconnect();
            break;
        case ARDUINO_EVENT_PROV_CRED_RECV:
            if (DEBUG_SW) Serial.println("\nReceived Wi-Fi credentials");
            if (DEBUG_SW) Serial.print("\tSSID : ");
            if (DEBUG_SW) Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid);
            break;
        case ARDUINO_EVENT_PROV_INIT:
            WiFiProv.disableAutoStop(10000);
            break;
        case ARDUINO_EVENT_PROV_CRED_SUCCESS:
            WiFiProv.endProvision(); break;
		default: ;
    }
}

// Callback for switch power state change
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    for (int i = 0; i < 5; ++i)
    {
        if (strcmp(device->getDeviceName(), switches[i]->getDeviceName()) == 0)
        {
            if (DEBUG_SW) Serial.printf("Switch %d value = %s\n", i + 1, val.val.b ? "true" : "false");
            if (strcmp(param->getParamName(), "Power") == 0)
            {
                switch_state[i] = val.val.b;
                digitalWrite(relays[i], switch_state[i] == LOW ? HIGH : LOW);
                pref.putBool(("Relay" + String(i + 1)).c_str(), switch_state[i]);
                param->updateAndReport(val);
            }
            break;
        }
    }
}

// Function to recall the last state
void getRelayState()
{
    for (int i = 0; i < 5; ++i)
    {
        switch_state[i] = pref.getBool(("Relay" + String(i + 1)).c_str(), 0);
        Serial.print("Last State Relay" + String(i + 1) + " - ");
        Serial.println(switch_state[i]);
        digitalWrite(relays[i], switch_state[i] == LOW ? HIGH : LOW);
        switches[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state[i]);
        delay(200);
    }
}

// NTP server settings
const char* ntpServer = "time.nist.gov";
const long gmtOffset_sec = 19800; // Adjust as per your timezone
const int daylightOffset_sec = 0; // Adjust for daylight saving time if applicable

void detect_Power_cycle(unsigned int sec_time) 
{
  //unsigned int cnt = 0;
    // Ensure WiFi connection
    /*while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        WiFi.reconnect();
        delay(5000);
        cnt++;
        if (cnt > 5) {
          Serial.println("detect power cycle timeout");
          RMakerFactoryReset(2);
        }
    }*/

    // Open Preferences storage
    if (!pref2.begin("timestamp", false)) {
        Serial.println("Failed to initialize Preferences");
        return;
    }

    // Synchronize time with NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("Synchronizing time...");
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo, 10000)) {
        Serial.println("Time sync failed. Retrying...");
        delay(5000);
    }

    // Log current time
    uint32_t currentTimestamp = time(NULL);
    Serial.printf("Current time: %04d-%02d-%02d %02d:%02d:%02d\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // Retrieve last stored timestamp
    uint32_t lastTimestamp = pref2.getUInt("lastTime", 0);
    Serial.printf("Last timestamp: %u, Current timestamp: %u\n", lastTimestamp, currentTimestamp);

    // Detect power cycle
    if (lastTimestamp && (currentTimestamp - lastTimestamp) <= sec_time) {
        Serial.printf("Power cycle detected within %u seconds!\n", currentTimestamp - lastTimestamp);
        RMakerFactoryReset(2);
    } else {
        Serial.println("No power cycle detected.");
    }

    // Update timestamp and close Preferences
    pref2.putUInt("lastTime", currentTimestamp);
    pref2.end();
}

void setup()
{
    if (DEBUG_SW) Serial.begin(115200);

    pref.begin("Relay_State", false);

    // Set the Relays GPIOs as output mode
    for (int i = 0; i < 5; ++i)
    {
        pinMode(relays[i], OUTPUT);
        digitalWrite(relays[i], HIGH); // Turn All Relays Off by default
    }

    pinMode(gpio_reset, INPUT);

    esp_log_level_set("*", ESP_LOG_DEBUG);
    Node my_node;
    my_node = RMaker.initNode(nodeName);

    // For preparing the 8 widgets in Rainmaker
    for (int i = 0; i < 5; ++i)
    {
        switches[i]->addCb(write_callback);
        my_node.addDevice(*switches[i]);
        switches[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state[i]);
        delay(500);
    }
    RMaker.setTimeZone("Asia/Kolkata");
    RMaker.enableOTA(OTA_USING_TOPICS);
    RMaker.enableTZService();
    RMaker.enableSchedule();
    RMaker.enableScenes();
    if (DEBUG_SW)Serial.printf("\nStarting ESP-RainMaker\n");
    RMaker.start();

  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(NETWORK_PROV_SCHEME_BLE, NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM, NETWORK_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(NETWORK_PROV_SCHEME_SOFTAP, NETWORK_PROV_SCHEME_HANDLER_NONE, NETWORK_PROV_SECURITY_1, pop, service_name);
#endif

  getRelayState(); // Get the last state of Relays
  detect_Power_cycle(30); //30s power cycle duration
}

void loop()
{
  //Method_1: Factory Reset
  // Read GPIO0 (external button to gpio_reset device
  if (digitalRead(gpio_reset) == LOW) {
    //Push button pressed
    if (DEBUG_SW)Serial.printf("reset Button Pressed!\n");
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 5000) {
      // If key pressed for more than 5secs, reset wifi
      if (DEBUG_SW)Serial.printf("reset to wifi.\n");
      RMakerWiFiReset(2);
    } else if((endTime - startTime) > 10000) {
      // If key pressed for more than 10secs, reset all
      if (DEBUG_SW)Serial.printf("Erase complete binary.\n");
      RMakerFactoryReset(2);
    }
  }
}
