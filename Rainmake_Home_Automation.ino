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

//By default ON/OFF sequence timing is limited to 2 Sec to factory reset for wi-fi provisioning
#define METHOD2_FACTORY_RST_SW_TIMING 2000

Preferences pref;

// Relay State
bool toggleState_1 = LOW; //Define integer to remember the toggle state for relay 1
bool toggleState_2 = LOW; //Define integer to remember the toggle state for relay 2
bool toggleState_3 = LOW; //Define integer to remember the toggle state for relay 3
bool toggleState_4 = LOW; //Define integer to remember the toggle state for relay 4
bool toggleState_5 = LOW; //Define integer to remember the toggle state for relay 5
bool toggleState_6 = LOW; //Define integer to remember the toggle state for relay 6
bool toggleState_7 = LOW; //Define integer to remember the toggle state for relay 7
bool toggleState_8 = LOW; //Define integer to remember the toggle state for relay 8

// Relay State
bool switch_state_ch1 = LOW;
bool switch_state_ch2 = LOW;
bool switch_state_ch3 = LOW;
bool switch_state_ch4 = LOW;
bool switch_state_ch5 = LOW;
bool switch_state_ch6 = LOW;
bool switch_state_ch7 = LOW;
bool switch_state_ch8 = LOW;

// BLE Credentials
const char *service_name = "PROV_home_automation"; // BLE node name
const char *pop = "123456"; //password

// Define the Node Name
char nodeName[] = "Smart_Home";

// GPIO for Relay (Appliance Control)
static uint8_t relay1 = 13;   //D13
static uint8_t relay2 = 15;   //D15
static uint8_t relay3 = 14;   //D14
static uint8_t relay4 = 27;   //D27
static uint8_t relay5 = 26;   //D26
static uint8_t relay6 = 25;   //D25
static uint8_t relay7 = 33;   //D33
static uint8_t relay8 = 32;   //D32

// GPIO for h/w reset pin & Temp/humidity sensor
static uint8_t gpio_reset = 0;   // Reset Pin

// Name of the device shown in the App
char Device1[] = "Switch_1";
char Device2[] = "Switch_2";
char Device3[] = "Switch_3";
char Device4[] = "Switch_4";
char Device5[] = "Switch_5";
char Device6[] = "Switch_6";
char Device7[] = "Switch_7";
char Device8[] = "Switch_8";

// Rainmaker side variables
static Switch my_switch1("Switch_1", &relay1);
static Switch my_switch2("Switch_2", &relay2);
static Switch my_switch3("Switch_3", &relay3);
static Switch my_switch4("Switch_4", &relay4);
static Switch my_switch5("Switch_5", &relay5);
static Switch my_switch6("Switch_6", &relay6);
static Switch my_switch7("Switch_7", &relay7);
static Switch my_switch8("Switch_8", &relay8);

// Wi-Fi connection handler
void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id)
  {
    case ARDUINO_EVENT_PROV_START:
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      if (DEBUG_SW)if (DEBUG_SW)Serial.print("\nConnected IP address : ");
      if (DEBUG_SW)if (DEBUG_SW)Serial.println(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if (DEBUG_SW)if (DEBUG_SW)Serial.println("\nDisconnected. Connecting to the AP again... ");
      WiFi.disconnect();
      WiFi.reconnect();
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      if (DEBUG_SW)if (DEBUG_SW)Serial.println("\nReceived Wi-Fi credentials");
      if (DEBUG_SW)if (DEBUG_SW)Serial.print("\tSSID : ");
      if (DEBUG_SW)if (DEBUG_SW)Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid);
      if (DEBUG_SW)if (DEBUG_SW)Serial.print("\tPassword : ");
      if (DEBUG_SW)if (DEBUG_SW)Serial.println((char const *) sys_event->event_info.prov_cred_recv.password);
      break;
    case ARDUINO_EVENT_PROV_INIT:
      wifi_prov_mgr_disable_auto_stop(10000);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      if (DEBUG_SW)if (DEBUG_SW)Serial.println("Stopping Provisioning!!!");
      wifi_prov_mgr_stop_provisioning();
      break;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(device_name, Device1) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_1 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch1 = val.val.b;
      (switch_state_ch1 == false) ? digitalWrite(relay1, HIGH) : digitalWrite(relay1, LOW);
      pref.putBool("Relay1", switch_state_ch1);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device2) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("SSwitch value_2 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch2 = val.val.b;
      (switch_state_ch2 == false) ? digitalWrite(relay2, HIGH) : digitalWrite(relay2, LOW);
      pref.putBool("Relay2", switch_state_ch2);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device3) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_3 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch3 = val.val.b;
      (switch_state_ch3 == false) ? digitalWrite(relay3, HIGH) : digitalWrite(relay3, LOW);
      pref.putBool("Relay3", switch_state_ch3);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device4) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_4 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch4 = val.val.b;
      (switch_state_ch4 == false) ? digitalWrite(relay4, HIGH) : digitalWrite(relay4, LOW);
      pref.putBool("Relay4", switch_state_ch4);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device5) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_5 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch5 = val.val.b;
      (switch_state_ch5 == false) ? digitalWrite(relay5, HIGH) : digitalWrite(relay5, LOW);
      pref.putBool("Relay5", switch_state_ch5);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device6) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_6 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch6 = val.val.b;
      (switch_state_ch6 == false) ? digitalWrite(relay6, HIGH) : digitalWrite(relay6, LOW);
      pref.putBool("Relay6", switch_state_ch6);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device7) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_7 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch7 = val.val.b;
      (switch_state_ch7 == false) ? digitalWrite(relay7, HIGH) : digitalWrite(relay7, LOW);
      pref.putBool("Relay7", switch_state_ch7);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, Device8) == 0) {
    if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Switch value_8 = %s\n", val.val.b ? "true" : "false");
    if (strcmp(param_name, "Power") == 0) {
      if (DEBUG_SW)if (DEBUG_SW)Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      switch_state_ch8 = val.val.b;
      (switch_state_ch8 == false) ? digitalWrite(relay8, HIGH) : digitalWrite(relay8, LOW);
      pref.putBool("Relay8", switch_state_ch8);
      param->updateAndReport(val);
    }
  }
}

// This function will recall the last state
void getRelayState()
{
  toggleState_1 = pref.getBool("Relay1", 0);
  Serial.print("Last State Relay1 - "); Serial.println(toggleState_1);
  digitalWrite(relay1, !toggleState_1);
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);
  delay(200);

  toggleState_2 = pref.getBool("Relay2", 0);
  Serial.print("Last State Relay2- "); Serial.println(toggleState_2);
  digitalWrite(relay2, !toggleState_2);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_2);
  delay(200);

  toggleState_3 = pref.getBool("Relay3", 0);
  Serial.print("Last State Relay3- "); Serial.println(toggleState_3);
  digitalWrite(relay3, !toggleState_3);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_3);
  delay(200);

  toggleState_4 = pref.getBool("Relay4", 0);
  Serial.print("Last State Relay4- "); Serial.println(toggleState_4);
  digitalWrite(relay4, !toggleState_4);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_4);
  delay(200);

  toggleState_5 = pref.getBool("Relay5", 0);
  Serial.print("Last State Relay5- "); Serial.println(toggleState_5);
  digitalWrite(relay5, !toggleState_5);
  my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_5);
  delay(200);

  toggleState_6 = pref.getBool("Relay6", 0);
  Serial.print("Last State Relay6- "); Serial.println(toggleState_6);
  digitalWrite(relay6, !toggleState_6);
  my_switch6.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_6);
  delay(200);

  toggleState_7 = pref.getBool("Relay7", 0);
  Serial.print("Last State Relay7- "); Serial.println(toggleState_7);
  digitalWrite(relay7, !toggleState_7);
  my_switch7.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_7);
  delay(200);

  toggleState_8 = pref.getBool("Relay8", 0);
  Serial.print("Last State Relay8- "); Serial.println(toggleState_8);
  digitalWrite(relay8, !toggleState_8);
  my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_8);
  delay(200);
}

nvs_handle myNvsHandle;
// NTP server settings
const char* ntpServer = "time.nist.gov";
const long gmtOffset_sec = 0; // Adjust as per your timezone
const int daylightOffset_sec = 0; // Adjust for daylight saving time if applicable

void detect_Power_cycle(unsigned int sec_time)
{
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to WiFi. IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect with WiFi");
    return;
  }
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase(); // Erase if necessary
      nvs_flash_init();
  }

  // Open NVS handle
  err = nvs_open("storage", NVS_READWRITE, &myNvsHandle);
  if (err != ESP_OK) {
    Serial.println("Error opening NVS handle!");
    return;
  }
  
  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Synchronizing time...");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) {
    Serial.printf("Failed to obtain time...");
    return;
  }
  Serial.println("Time synchronized");

  // Get the last stored timestamp
  uint32_t lastTimestamp = 0;
  err = nvs_get_u32(myNvsHandle, "lastTime", &lastTimestamp);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    Serial.println("No previous timestamp found.");
  } else if (err == ESP_OK) {
    Serial.print("Last timestamp: ");
    Serial.println(lastTimestamp);
  } else {
    Serial.println("Error reading last timestamp!");
  }

  // Get the current timestamp from the network time
  time_t now = time(NULL);
  uint32_t currentTimestamp = now; // Current time in seconds since the epoch
  Serial.print("Current timestamp: ");
  Serial.println(currentTimestamp);

  // Check if the time difference is less than 5 seconds
  if (lastTimestamp != 0 && (currentTimestamp - lastTimestamp) <= sec_time) {
    Serial.printf("Power cycle detected within %d seconds!", (currentTimestamp - lastTimestamp));
    RMakerFactoryReset(2);
  } else {
    Serial.println("No power cycle detected.");
  }

  // Store the current timestamp in NVS
  err = nvs_set_u32(myNvsHandle, "lastTime", currentTimestamp);
  if (err != ESP_OK) {
    Serial.println("Error setting timestamp!");
  } else {
    Serial.println("Timestamp saved.");
    nvs_commit(myNvsHandle);
  }

  // Close NVS handle
  nvs_close(myNvsHandle);
}

void setup()
{
  if (DEBUG_SW)Serial.begin(115200);

  pref.begin("Relay_State", false);

  // Set the Relays GPIOs as output mode
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(relay5, OUTPUT);
  pinMode(relay6, OUTPUT);
  pinMode(relay7, OUTPUT);
  pinMode(relay8, OUTPUT);

  //Turning All Relays Off by default
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);
  digitalWrite(relay5, HIGH);
  digitalWrite(relay6, HIGH);
  digitalWrite(relay7, HIGH);
  digitalWrite(relay8, HIGH);

  pinMode(gpio_reset, INPUT);

  Node my_node;
  my_node = RMaker.initNode(nodeName);

  // For preparing the 8 widgets in Rainmaker
  my_switch1.addCb(write_callback);
  my_node.addDevice(my_switch1);
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch1);
  delay(500);

  my_switch2.addCb(write_callback);
  my_node.addDevice(my_switch2);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch2);
  delay(500);

  my_switch3.addCb(write_callback);
  my_node.addDevice(my_switch3);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch3);
  delay(500);

  my_switch4.addCb(write_callback);
  my_node.addDevice(my_switch4);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch4);
  delay(500);

  my_switch5.addCb(write_callback);
  my_node.addDevice(my_switch5);
  my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch5);
  delay(500);

  my_switch6.addCb(write_callback);
  my_node.addDevice(my_switch6);
  my_switch6.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch6);
  delay(500);

  my_switch7.addCb(write_callback);
  my_node.addDevice(my_switch7);
  my_switch7.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch7);
  delay(500);

  my_switch8.addCb(write_callback);
  my_node.addDevice(my_switch8);
  my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state_ch8);
  delay(500);

  RMaker.enableOTA(OTA_USING_TOPICS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.enableScenes();
  if (DEBUG_SW)Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
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
      RMakerWiFiReset(2);//RMakerFactoryReset(2);
    } else if((endTime - startTime) > 10000) {
      // If key pressed for more than 10secs, reset all
      if (DEBUG_SW)Serial.printf("reset to factory.\n");
      RMakerFactoryReset(2);
    }
  }
}
