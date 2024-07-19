// Include libraries
#include <Arduino.h>
#include <LittleFS.h>
#include "adc_lookup_table.h" // Custom generated lookup tables per ESP32 chip to compensate for non-linear ADC
#include "BluetoothSerial.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <HDC2080.h>
#include <HardwareSerial.h>
#include "s300i2c.h"
#include <Adafruit_GPS.h>

//Pin definitions
#define FLOW_ANALOG_PIN 36 // Not used
#define SO2_ANALOG_PIN 39 // SO2 voltage sense, 7.49:1 divider
#define BATT_ANALOG_PIN 35 // Battery voltage sense 
#define MOTOR_PWM_PIN 19 // Pump power control

//Constants
const float BATT_VOLTAGE_SCALING = 7.49; // scaling factor for voltage divider
const float SO2_VOLTAGE_SCALING = 1.49;  //  Scaling factor based on resistor divider
const long interval = 2000;       // Desired interval (1 second)
const int ppsDelay = 0;          // Delay in milliseconds after PPS to read NMEA sentences
const int GPS_timeout = 4000;

//Variables
volatile bool ppsTriggered = false;
bool readSensorsAndLog = false;
unsigned long previousMillis = 0; // Stores the last time the loop ran
char c;
bool status_GPS_module = false;
bool Status_File_System = false;
bool status_s300 = false;
bool StatusBMESensor = false;
bool StatusHDCSensor = false;
bool BTSwitchedOn = false;
unsigned int co2 = 0;
int pump_pwm = 0;
float bmeTemperature;
float bmePressure;
float bmeHumidity;
float hdcHumidity = 0;
float hdcTemperature = 0;
String filename = "/datalogger.txt";
String last_NMEA = ""; // GPS last string
bool newGGA = false;
bool newRMC = false;
uint64_t chipid = 0;


// Serial input buffers
char StringInputSpeicher[500];
char BluetoothInputSpeicher[500];
char RFDStringInputSpeicher[500];

BluetoothSerial ESP_BT; //Bluetooth serial object
HardwareSerial SerialRFD(1); // Hardware serial for RFD

HardwareSerial GPSSerial(2); // Hardware serial for GPS
Adafruit_GPS GPS(&GPSSerial); // GPS serial object

Adafruit_BME280 bme; // BME object
HDC2080 hdc(0x40); //HDC2080 Sensor Object
S300I2C s3(Wire); // S300 object



// PPS interrupt service routine
void IRAM_ATTR onPPS() {
  ppsTriggered = true;
}
 
//LittleFS Filesystem
bool initialize_littlefs_format_file_system() {
  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed");
    Serial.println("No filesystem found; formatting...");
    if (!LittleFS.begin(true)) {
      Serial.println("LittleFS mount failed");
      Serial.println("Formatting not possible");
      return false;
    } else {
      Serial.println("Formatting successful");
      Serial.println("Information on the filesystem:");
      Serial.printf("- Bytes total:   %ld\n", LittleFS.totalBytes());
      Serial.printf("- Bytes used: %ld\n\n", LittleFS.usedBytes());
      return true;
    }
  }
  Serial.println("Information on the filesystem:");
  Serial.printf("- Bytes total:   %ld\n", LittleFS.totalBytes());
  Serial.printf("- Bytes used: %ld\n\n", LittleFS.usedBytes());
  return true;
}
bool write_string_to_file(String filename, String string) {
  File file = LittleFS.open(filename, "a");
  if (!file) {
    Serial.println("There was an error opening the file for appending");
    return false;
  }
  if (file.print(string)) {
    file.close();
    return true;
  } else {
    Serial.println("File write failed");
    file.close();
    return false;
  }
  file.close();
  return false;
}
bool read_file_and_print_to_serial(String filename) {
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("There was an error opening the file for reading");
    return false;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  return true;
}
bool delete_file(String filename) {
  Serial.print("Delete file: ");
  Serial.print(filename);
  Serial.print(" ");
  if (LittleFS.remove(filename)) {
    Serial.println("- File deleted");
    return true;
  } else {
    Serial.println("- File could not be deleted");
    return false;
  }
  return false;
}

// Serial communication
void InitialiseSerial(int BaudRate) {
  Serial.begin(BaudRate); // Start serial monitor at 115200 (ESP32)
  delay(500);
  Serial.println("BOOTUP - Serial connection established.");
  if (readSerialTo(StringInputSpeicher)) {
    Serial.println("Old data that was stored in buffer:");
    Serial.println(StringInputSpeicher);
  }
}
bool readSerialTo(char serialSpeicher[]) {
  const int SERIAL_BUFFER_SIZE = 500;
  static char serialBuffer[SERIAL_BUFFER_SIZE];
  static byte index;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' && index > 0) {
      serialBuffer[index] = '\0';
      index = 0;
      strcpy(serialSpeicher, serialBuffer);
      return true;
    }
    else if (c >= 32 && index < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[index++] = c;
    }
  }
  return false;
}

// Bluetooth Serial communication implementation
bool readBTSerialTo(char BTInputBuffer[]) {
  const int BTSERIAL_BUFFER_SIZE = 500;
  static char BTserialBuffer[BTSERIAL_BUFFER_SIZE];
  static byte BTindex;
  while (ESP_BT.available()) {
    char BTc = ESP_BT.read();
    if ((BTc == '\n' || BTc == '\r') && BTindex > 0) {
      BTserialBuffer[BTindex] = '\0';
      BTindex = 0;
      strcpy(BTInputBuffer, BTserialBuffer);
      return true;
    }
    else if (BTc >= 32 && BTindex < BTSERIAL_BUFFER_SIZE - 1) {
      BTserialBuffer[BTindex++] = BTc;
    }
  }
  return false;
}
void InitialiseBluetooth() {
  if (!ESP_BT.begin("ESP32_BluetoothSensor")) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized");
  }
  if (readBTSerialTo(BluetoothInputSpeicher)) {
    Serial.println("Old data that was stored in the buffer:");
    Serial.println(BluetoothInputSpeicher);
  }
  BTSwitchedOn = true;
}

//RFD UHF Radio communicaiton implementation
void InitialiseRFDSerial() {
  SerialRFD.begin(57600, SERIAL_8N1, 13, 14);
  if (SerialRFD) {
    Serial.println("SerialRFD successfully set up");
  }
  if (readSerialRFDTo(RFDStringInputSpeicher)) {
    Serial.println("Old data that was stored in the buffer:");
    Serial.println(RFDStringInputSpeicher);
  }
}
bool readSerialRFDTo(char serialSpeicher[]) {
  const int SERIAL_BUFFER_SIZE = 500;
  static char serialBuffer[SERIAL_BUFFER_SIZE];
  static byte index;
  while (SerialRFD.available()) {
    char c = SerialRFD.read();
    if (c == '\n' && index > 0) {
      serialBuffer[index] = '\0';
      index = 0;
      strcpy(serialSpeicher, serialBuffer);
      return true;
    }
    else if (c >= 32 && index < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[index++] = c;
    }
  }
  return false;
}

// read a SI voltage from a pin
float readSIVoltageFromPin(int volt_pin, int anzahl_spannungs_messung, int x_bit_adc, float
                           max_voltage) {
  float si_voltage = 0;
  long long voltage_sum = 0;
  if (anzahl_spannungs_messung >= 300000) {
    anzahl_spannungs_messung = 300000;
  }
  if (anzahl_spannungs_messung <= 0) {
    return 0;
  }
  delay(3);
  for (int i = 0; i < anzahl_spannungs_messung; i++) {
    voltage_sum += analogRead(volt_pin);
  }
  int total_adc_res = int(pow(2, x_bit_adc));
  float avg_adc_value = float(voltage_sum) / float(anzahl_spannungs_messung);

  //ADC LUT correction here
  float corrected_adc_value = ADC_LUT[int(avg_adc_value)];
  
  si_voltage = (corrected_adc_value * max_voltage) / total_adc_res;
  return si_voltage;
}
// BME280 Temp, Pressure, Humidity Sensor implementation
bool initialize_bme_sensor() {
  StatusBMESensor = bme.begin(0x76);
  if (!StatusBMESensor) {
    Serial.println("Could not find a valid BME280 sensor!");
    StatusBMESensor = false;
    return false;
  }
  Serial.println("Initialization of BME Sensor done.");
  StatusBMESensor = true;
  return true;
}

// HDC2080 Humidity Sensor Implementation
bool initialize_hdc_sensor() {
  hdc.begin();
  hdc.reset();
  hdc.setMeasurementMode(TEMP_AND_HUMID);  // Set measurements to temperature and humidity
  hdc.setRate(ONE_HZ);                     // Set measurement frequency to 1 Hz
  hdc.setTempRes(FOURTEEN_BIT);
  hdc.setHumidRes(FOURTEEN_BIT);
  //begin measuring
  hdc.triggerMeasurement();
  StatusHDCSensor = true;
  Serial.println("HDC Humidity Sensor Initialized");
  return true;
}

// GPS implementation
void clearGPSSimple() { // GPS Clear out any data in the serial buffer
  while (GPSSerial.available()) {
    GPSSerial.read();
  }
}
bool clearGPS() { // Delete previously acquired data from GPS
  int start_millis = millis();
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
    if (((millis() - start_millis) >= GPS_timeout) or (millis() < start_millis)) {
      status_GPS_module = false;
      break;
    }
  }
  GPS.parse(GPS.lastNMEA());
  start_millis = millis();
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
    if (((millis() - start_millis) >= GPS_timeout) or (millis() < start_millis)) {
      status_GPS_module = false;
      break;
    }
  }
  GPS.parse(GPS.lastNMEA());
  start_millis = millis();
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
    if (((millis() - start_millis) >= GPS_timeout) or (millis() < start_millis)) {
      status_GPS_module = false;
      return false;
      break;
    }
  }
  GPS.parse(GPS.lastNMEA());
  return true;
}
bool initialize_GPS() { // Setup of GPS Module
   // Initialize UART2 for the GPS module with 9600 baud rate
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17); // RX on GPIO16, TX on GPIO17

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(100);
  status_GPS_module = clearGPS();
  if (status_GPS_module) {
    Serial.println("GPS connection established!");
    return true;
  }
  else {
    Serial.println("GPS initialization failed!");
    return false;
  }
  return true;
}

// S300 CO2 Sensor
bool check_sensor_presence(uint8_t address) {
  Wire.beginTransmission(address);
  if (Wire.endTransmission() == 0) {
    return true; // Sensor acknowledged
  }
  return false; // Sensor did not acknowledge
}
bool initialize_s300() {
  Serial.print("S300 CO2 sensor... ");
  Wire.begin();
  // Test to see if the CO2 sensor is present, the library "begin" function never returns false
  if (!check_sensor_presence(S300I2C_ADDR)) {
    Serial.println("not found!");
    return false;
  }
    s3.begin(S300I2C_ADDR);
    delay(10000);
    s3.wakeup();
    s3.end_mcdl();
    s3.end_acdl();
    Serial.println("Initialized");
    return true;
}
unsigned int get_co2() {
  unsigned int co2temporary = 0;
  // Test every time to see if the CO2 sensor is still present, else return 0.
  if (check_sensor_presence(S300I2C_ADDR)) {  
    co2temporary = s3.getCO2ppm();
  }
  return co2temporary;
}

//Serial Terminal Menu Code
void processCommand(const char* input, Stream& output) {
  output.print("Serial input: ");
  output.println(input);

  if (strcmp(input, "help") == 0) {
    output.print("Print Datalogging file to serial terminal: ");
    output.println("'print data'");
    output.print("Delete Datalogging file: ");
    output.println("'delete data'");
    output.print("Switch pump on: ");
    output.println("'pump_on'");
    output.print("Switch pump off: ");
    output.println("'pump_off'");
    for (int i = 0; i <= 30; i += 10) {
      output.print("Switch pump ");
      output.print(i);
      output.println("/250: ");
    }
    output.println("...");
    output.println();
  } else if (strcmp(input, "print data") == 0) {
    Serial.println("Printing data...");
    ESP_BT.println("Printing data...");
    SerialRFD.println("Printing data...");
    read_file_and_print_to_serial(filename);
    Serial.println("\nData print complete.");
    ESP_BT.println("\nData print complete.");
    SerialRFD.println("\nData print complete.");
  } else if (strcmp(input, "delete data") == 0) {
    delete_file(filename);
    Serial.println("Data file deleted.");
    ESP_BT.println("Data file deleted.");
    SerialRFD.println("Data file deleted.");
  } else if (strcmp(input, "pump_on") == 0) {
    digitalWrite(MOTOR_PWM_PIN, HIGH);
    ledcWrite(MOTOR_PWM_PIN, 254);
    pump_pwm = 254;
  } else if (strcmp(input, "pump_off") == 0) {
    digitalWrite(MOTOR_PWM_PIN, LOW);
    ledcWrite(MOTOR_PWM_PIN, 0);
    pump_pwm = 0;
  } else {
    for (int i = 0; i <= 250; i += 10) {
      char cmd[4];
      sprintf(cmd, "%d", i);
      if (strcmp(input, cmd) == 0) {
        ledcWrite(MOTOR_PWM_PIN, i);
        pump_pwm = i;
        break;
      }
    }
  }
}

void setup() {
  chipid = ESP.getEfuseMac(); // Get unique ESP32 chip ID which is essentially the MAC address
  InitialiseSerial(115200);
  InitialiseRFDSerial();
  InitialiseBluetooth();
  pinMode(FLOW_ANALOG_PIN, INPUT);
  pinMode(SO2_ANALOG_PIN, INPUT);
  pinMode(BATT_ANALOG_PIN, INPUT);
  Serial.println(String(chipid));
  Status_File_System = initialize_littlefs_format_file_system();
  if (Status_File_System) {
    Serial.println("File system initialized");
  }
  StatusBMESensor = initialize_bme_sensor();
  StatusHDCSensor = initialize_hdc_sensor();
  status_GPS_module = initialize_GPS();
  status_s300 = initialize_s300();
  // PWM Pin implementation (pump)
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  ledcAttach(MOTOR_PWM_PIN,200,8);

  // Setup PPS interrupt
  pinMode(4, INPUT_PULLUP); // PPS is connected to GPIO4
  attachInterrupt(digitalPinToInterrupt(4), onPPS, RISING);

  Serial.println("Setup finished");
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long sampleStartMillis; // Updated when the sensor data collection starts each time
  String write_to_file_string = "";
  c = GPS.read();  //Take one char in from the GPS serial buffer each loop
  
  // Collect sensor data if PPS or time interval
  if (ppsTriggered || (currentMillis - previousMillis >= interval)) {
    ppsTriggered = false; // Reset the flag if PPS triggered
    clearGPSSimple(); // This clears all unparsed bytes from the GPS serial UART queue
    previousMillis = currentMillis; // Update the last run time
    readSensorsAndLog = true;
  }

    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
      if (strstr(GPS.lastNMEA(), "GGA")) {
          newGGA = true;
          //Serial.println("GGA");
        } else if (strstr(GPS.lastNMEA(), "RMC")) {
          newRMC = true;
          //Serial.println("RMC");
        }
      last_NMEA = GPS.lastNMEA();
      //Serial.println(last_NMEA); // Print to Serial (or store as needed)
    }
//    } else {
//      Serial.println("GPS not working!");
//      ESP_BT.println("GPS not working!");
//      SerialRFD.println("GPS not working!");
//      last_NMEA = "GPS not working!";
//  }

  // Try to read serial input and execute command
  if (readSerialTo(StringInputSpeicher)) {
    processCommand(StringInputSpeicher, Serial);
  }

  if (readBTSerialTo(BluetoothInputSpeicher)) {
      Serial.println("Bluetooth Input detected.");  // Debug print
      Serial.print("Bluetooth Input: ");
      Serial.println(BluetoothInputSpeicher);
    processCommand(BluetoothInputSpeicher, ESP_BT);
  }

  if (readSerialRFDTo(RFDStringInputSpeicher)) {
    processCommand(RFDStringInputSpeicher, SerialRFD);
  }

  if (readSensorsAndLog){
  // Log the time that sensor sampling starts
  sampleStartMillis = millis(); 
  // Get data from BME Sensor
  if (StatusBMESensor) {
    bmeTemperature = bme.readTemperature();
    bmePressure = bme.readPressure();
    bmePressure = bmePressure / 100;
    bmeHumidity = bme.readHumidity();
  }
  else {
    bmeTemperature = 0;
    bmePressure = 0;
    bmeHumidity = 0;
    Serial.println("No BME sensor available!!");
    ESP_BT.println("No BME sensor available!!");
    SerialRFD.println("No BME sensor available!!");
    StatusBMESensor = initialize_bme_sensor();
    write_to_file_string += "No BME Sensor available,,,";
  }
  
  if (StatusHDCSensor) {
    hdcHumidity = hdc.readHumidity();
    hdcTemperature = hdc.readTemp();
  }
  else {
    hdcHumidity = 0;
    hdcTemperature = 0;
    Serial.println("No HDC sensor available!!");
  }
  // Get analog voltages from pins
  // flow meter
  float SI_voltage_pin_FLOW = readSIVoltageFromPin(FLOW_ANALOG_PIN, 10, 12, 3.3);
  if (SI_voltage_pin_FLOW >= 3.7) {
    Serial.print("Problem with ADC on Pin FLOW; measured voltage=");
    Serial.println(SI_voltage_pin_FLOW);
    ESP_BT.print("Problem with ADC on Pin FLOW; measured voltage=");
    ESP_BT.println(SI_voltage_pin_FLOW);
    SerialRFD.print("Problem with ADC on Pin FLOW; measured voltage=");
    SerialRFD.println(SI_voltage_pin_FLOW);
  }
  // SO2
  float SI_voltage_pin_SO2 = readSIVoltageFromPin(SO2_ANALOG_PIN, 10, 12, 3.3);
  float actual_SO2_voltage = SI_voltage_pin_SO2 * SO2_VOLTAGE_SCALING;
  if (SI_voltage_pin_SO2 >= 3.7) {
    Serial.print("Problem with ADC on Pin SO2; ADC voltage=");
    Serial.println(SI_voltage_pin_SO2);
    ESP_BT.print("Problem with ADC on Pin SO2; ADC voltage=");
    ESP_BT.println(SI_voltage_pin_SO2);
    SerialRFD.print("Problem with ADC on Pin SO2; ADC voltage=");
    SerialRFD.println(SI_voltage_pin_SO2);
  }
  // Battery
  float SI_voltage_pin_BATT = readSIVoltageFromPin(BATT_ANALOG_PIN, 10, 12, 3.3);
  float actual_batt_voltage = SI_voltage_pin_BATT * BATT_VOLTAGE_SCALING;
  if (SI_voltage_pin_BATT >= 3.7) {
    Serial.print("Problem with ADC on Pin BATT; ADC voltage=");
    Serial.println(SI_voltage_pin_BATT);
    ESP_BT.print("Problem with ADC on Pin BATT; ADC voltage=");
    ESP_BT.println(SI_voltage_pin_BATT);
    SerialRFD.print("Problem with ADC on Pin BATT; ADC voltage=");
    SerialRFD.println(SI_voltage_pin_BATT);
  }
  
  // Get data from CO2 Sensor
  co2 = get_co2();

  write_to_file_string += String(sampleStartMillis);
  write_to_file_string += ",";
    //GPS date-time in ISO-8601 format
  write_to_file_string += String(GPS.year + 2000) + "-" +
                      String(GPS.month < 10 ? "0" : "") + String(GPS.month) + "-" +
                      String(GPS.day < 10 ? "0" : "") + String(GPS.day) + "T" +
                      String(GPS.hour < 10 ? "0" : "") + String(GPS.hour) + ":" +
                      String(GPS.minute < 10 ? "0" : "") + String(GPS.minute) + ":" +
                      String(GPS.seconds < 10 ? "0" : "") + String(GPS.seconds) + "Z";
  write_to_file_string += ",";
    // Assemble latitude, longitude, altitude, and fix quality
  write_to_file_string += String(GPS.latitudeDegrees,6);
  write_to_file_string += ",";
  write_to_file_string += String(GPS.longitudeDegrees,6);
  write_to_file_string += ",";
  write_to_file_string += String(GPS.altitude);
  write_to_file_string += ",";
  write_to_file_string += String(GPS.fixquality);
  write_to_file_string += ",";
  write_to_file_string += String(hdcTemperature);
  write_to_file_string += ",";
  write_to_file_string += String(bmePressure);
  write_to_file_string += ",";
  write_to_file_string += String(hdcHumidity);
  write_to_file_string += ",";
  write_to_file_string += String(SI_voltage_pin_FLOW, 4);
  write_to_file_string += ",";
  write_to_file_string += String(actual_SO2_voltage, 4);
  write_to_file_string += ",";
  write_to_file_string += String(actual_batt_voltage, 4);
  write_to_file_string += ",";
  write_to_file_string += String(co2);
  write_to_file_string += ",";
  write_to_file_string += comment;
  comment = "";
  write_to_file_string += "\n";
  
  // Write data to internal storage
  if (Status_File_System) {
    write_string_to_file(filename, write_to_file_string);
  }
  else {
    Serial.println("Filesystem is not working!!! Data will not be saved on ESP32!");
    ESP_BT.println("Filesystem is not working!!! Data will not be saved on ESP32!");
    SerialRFD.println("Filesystem is not working!!! Data will not be saved on ESP32!");
    Status_File_System = initialize_littlefs_format_file_system();
  }
  // Print data to all serial interfaces
  SerialRFD.print(write_to_file_string);
  Serial.print(write_to_file_string);
  ESP_BT.print(write_to_file_string);
  readSensorsAndLog = false;
}
}
