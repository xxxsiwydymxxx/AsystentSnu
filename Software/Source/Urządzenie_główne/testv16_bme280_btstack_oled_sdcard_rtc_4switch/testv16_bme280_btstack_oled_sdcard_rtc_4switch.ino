#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <btstack.h> 



const char* target_name = "XIAO_Health_Native";
const uint16_t target_service_uuid = 0xFF00;
const uint16_t target_char_uuid    = 0xFF01;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static gatt_client_notification_t notification_listener;

static gatt_client_service_t found_service;
static gatt_client_characteristic_t found_char;
static bool service_found = false;
static bool char_found = false;
static bool is_subscribed = false;


volatile int global_hr = 0;
volatile int global_spo2 = 0;
bool bleConnected = false;


#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define OLED_RESET  17


#define SD_SCK  10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_CS   13


#define BTN1_PIN 6
#define BTN2_PIN 7
#define BTN3_PIN 8
#define BTN4_PIN 9

#define BME_ADDR    0x76 
#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
Adafruit_BME280 bme;
RTC_DS3231 rtc;

uint32_t tsLastReport = 0;
uint32_t tsLastLog = 0;
const long REPORT_PERIOD = 100; 
const long LOG_PERIOD    = 60000; 

bool sdStatus = false;
int currentMode = 1; 


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void gatt_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void on_characteristics_discovered(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void on_services_discovered(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

void setup() {
  Serial.begin(115200);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  delay(1000);


  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Init Components...");
  display.display();


  if (!bme.begin(BME_ADDR)) {
    Serial.println("BME280 not found!");
    display.println("BME Fail!");
    display.display();
  }


  if (!rtc.begin()) {
    Serial.println("RTC not found");
    display.println("RTC Fail!");
    display.display();
    while(1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }


  SPI1.setSCK(SD_SCK);
  SPI1.setTX(SD_MOSI);
  SPI1.setRX(SD_MISO);
  if (!SD.begin(SD_CS, SPI1)) {
    Serial.println("SD Mount Failed!");
    display.println("SD Fail!"); 
    sdStatus = false;
  } 
  else {
    Serial.println("SD Mounted Successfully");
    sdStatus = true;
    if (!SD.exists("datalog.txt")) {
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Timestamp, Temp, Hum, Press, HR, SpO2");
        dataFile.close();
      }
    }
  }


  display.println("Init Bluetooth...");
  display.display();
  
  l2cap_init();
  gatt_client_init();
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

  hci_event_callback_registration.callback = &packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);

  hci_power_control(HCI_POWER_ON);
  // --------------------

  delay(1000);
  display.clearDisplay();
}

void loop() {

  delay(10); 

  unsigned long currentMillis = millis();
  

  if (currentMillis - tsLastReport > REPORT_PERIOD) {
    DateTime now = rtc.now();
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;
    
    if (digitalRead(BTN1_PIN) == LOW)      currentMode = 1;
    else if (digitalRead(BTN2_PIN) == LOW) currentMode = 2;
    else if (digitalRead(BTN3_PIN) == LOW) currentMode = 3;
    else if (digitalRead(BTN4_PIN) == LOW) currentMode = 4;
    
    updateScreen(now, t, h, p, sdStatus, currentMode);
    tsLastReport = currentMillis;
  }


  if (currentMillis - tsLastLog > LOG_PERIOD) {
    DateTime now = rtc.now();
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;

    logData(now, t, h, p, global_hr, global_spo2);
    tsLastLog = currentMillis;
  }
}

void logData(DateTime dt, float t, float h, float p, int hr, int spo2) {
  if (!sdStatus) return;

  digitalWrite(LED_BUILTIN, HIGH);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(dt.year(), DEC); dataFile.print('/');
    dataFile.print(dt.month(), DEC); dataFile.print('/');
    dataFile.print(dt.day(), DEC);
    dataFile.print(" ");
    
    dataFile.print(dt.hour(), DEC); dataFile.print(':');
    dataFile.print(dt.minute(), DEC); dataFile.print(':');
    dataFile.print(dt.second(), DEC); dataFile.print(", ");
    
    dataFile.print(t); dataFile.print(", ");
    dataFile.print(h); dataFile.print(", ");
    dataFile.print(p); dataFile.print(", ");
    dataFile.print(hr); dataFile.print(", ");
    dataFile.println(spo2);
    
    dataFile.close();
    Serial.println("Log Saved");
  } else {
    sdStatus = false; 
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void updateScreen(DateTime now, float t, float h, float p, bool sdOk, int mode) {
  display.clearDisplay();
  if (mode == 4) { display.display(); return; }

  display.setTextColor(SSD1306_WHITE);
  
  if (mode == 1) { // CLOCK
    display.setTextSize(1);
    display.setCursor(30, 0);
    if(now.day() < 10) display.print('0');
    display.print(now.day()); display.print('/');
    if(now.month() < 10) display.print('0');
    display.print(now.month()); display.print('/');
    display.print(now.year());

    display.setTextSize(2);
    display.setCursor(18, 25);
    if(now.hour() < 10) display.print('0');
    display.print(now.hour()); display.print(':');
    if(now.minute() < 10) display.print('0');
    display.print(now.minute());
    
    display.print(':');
    if(now.second() < 10) display.print('0');
    display.print(now.second());
  }
  else if (mode == 2) { 

    display.setTextSize(1);
    display.setCursor(0,0);
    if(bleConnected) display.print("Connected");
    else display.print("Searching...");


    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);


    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print("HR: ");
    if(global_hr > 0) display.print(global_hr);
    else display.print("--");
    display.setTextSize(1);
    display.print(" bpm");


    display.setTextSize(2);
    display.setCursor(0, 42);
    display.print("O2: ");
    if(global_spo2 > 0) display.print(global_spo2);
    else display.print("--");
    display.setTextSize(1);
    display.print(" %");
  }
  else if (mode == 3) { 
    display.setTextSize(1);
    display.setCursor(0, 0);
    if(now.day() < 10) display.print('0');
    display.print(now.day()); display.print('/');
    if(now.month() < 10) display.print('0');
    display.print(now.month()); 

    display.setCursor(80, 0);
    if(sdOk) display.print("SD ");
    if(bleConnected) display.print("BT");
    else display.print("--");

    display.setCursor(0, 12);
    display.setTextSize(2);
    if(now.hour() < 10) display.print('0');
    display.print(now.hour()); display.print(':');
    if(now.minute() < 10) display.print('0');
    display.print(now.minute());

    display.drawLine(0, 30, 128, 30, SSD1306_WHITE);


    display.setTextSize(1);
    display.setCursor(0, 34);
    display.print("T: "); display.print(t, 1);
    display.setCursor(0, 44);
    display.print("H: "); display.print(h, 0); display.print("%");
    display.setCursor(0, 54);
    display.print("P: "); display.print(p, 0);

    display.drawLine(64, 30, 64, 64, SSD1306_WHITE);


    display.setCursor(68, 34);
    display.print("HR: "); 
    if(global_hr > 0) display.print(global_hr);
    else display.print("--");

    display.setCursor(68, 44);
    display.print("O2: "); 
    if(global_spo2 > 0) { display.print(global_spo2); display.print("%"); }
    else display.print("--");
  }

  display.display();
}



static void gatt_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  if (packet_type != HCI_EVENT_PACKET) return;
  
  if (hci_event_packet_get_type(packet) == GATT_EVENT_NOTIFICATION) {
    const uint8_t *data = gatt_event_notification_get_value(packet);
    uint16_t length = gatt_event_notification_get_value_length(packet);

    if (length == 2) {
       global_hr = data[0];
       global_spo2 = data[1];
    }
  }
}

static void on_characteristics_discovered(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  if (packet_type != HCI_EVENT_PACKET) return;
  uint8_t event = hci_event_packet_get_type(packet);

  if (event == GATT_EVENT_CHARACTERISTIC_QUERY_RESULT) {
    gatt_client_characteristic_t temp_char;
    gatt_event_characteristic_query_result_get_characteristic(packet, &temp_char);

    if (temp_char.uuid16 == target_char_uuid) {
      found_char = temp_char;
      char_found = true;
    }
  } 
  else if (event == GATT_EVENT_QUERY_COMPLETE) {
    if (char_found && !is_subscribed) {
      Serial.println("FORCING SUBSCRIPTION...");
      

      uint16_t cccd_handle = found_char.value_handle + 1;
      uint8_t enable_data[] = {0x01, 0x00};
      
      gatt_client_write_characteristic_descriptor_using_descriptor_handle(
          packet_handler, 
          connection_handle, 
          cccd_handle, 
          2, 
          enable_data
      );

      
      is_subscribed = true;
    }
  }
}

static void on_services_discovered(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  if (packet_type != HCI_EVENT_PACKET) return;
  uint8_t event = hci_event_packet_get_type(packet);

  if (event == GATT_EVENT_SERVICE_QUERY_RESULT) {
    gatt_client_service_t service;
    gatt_event_service_query_result_get_service(packet, &service);
    if (service.uuid16 == target_service_uuid) {
      found_service = service;
      service_found = true;
    }
  } 
  else if (event == GATT_EVENT_QUERY_COMPLETE) {
    if (service_found) {
      gatt_client_discover_characteristics_for_service(&on_characteristics_discovered, connection_handle, &found_service);
    }
  }
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  if (packet_type != HCI_EVENT_PACKET) return;
  uint8_t event = hci_event_packet_get_type(packet);

  switch (event) {
    case BTSTACK_EVENT_STATE:
      if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
        gap_set_scan_parameters(0, 0x0030, 0x0030);
        gap_start_scan();
      }
      break;

    case GAP_EVENT_ADVERTISING_REPORT: {
      uint8_t event_type = gap_event_advertising_report_get_advertising_event_type(packet);
      if (event_type == 0 || event_type == 1) { 
        bd_addr_t address;
        gap_event_advertising_report_get_address(packet, address);
        uint8_t length = gap_event_advertising_report_get_data_length(packet);
        const uint8_t *data = gap_event_advertising_report_get_data(packet);
        
        int i = 0;
        while (i < length) {
          uint8_t len = data[i];
          if (len == 0) break;
          uint8_t type = data[i+1];
          if (type == 0x09 || type == 0x08) { 
            char name_buffer[32];
            memset(name_buffer, 0, 32);
            memcpy(name_buffer, &data[i+2], (len - 1) > 30 ? 30 : (len - 1));
            
            if (strcmp(name_buffer, target_name) == 0) {
              gap_stop_scan();
              gap_connect(address, (bd_addr_type_t)gap_event_advertising_report_get_address_type(packet));
              return;
            }
          }
          i += len + 1;
        }
      }
      break;
    }

    case HCI_EVENT_LE_META:
      if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
        if (hci_subevent_le_connection_complete_get_status(packet) == 0) {
          connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
          bleConnected = true;
          
          service_found = false;
          char_found = false;
          is_subscribed = false;
          
          gatt_client_discover_primary_services_by_uuid16(on_services_discovered, connection_handle, target_service_uuid);
          gatt_client_listen_for_characteristic_value_updates(&notification_listener, &gatt_event_handler, connection_handle, NULL);
        } else {
          gap_start_scan();
        }
      }
      break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
      bleConnected = false;
      global_hr = 0;
      global_spo2 = 0;
      is_subscribed = false;
      gap_start_scan();
      break;
  }
}