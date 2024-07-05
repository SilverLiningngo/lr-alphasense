#define FLOW_ANALOG_PIN 36 // old 12
#define SO2_ANALOG_PIN 39 // old 26
#define NC_ANALOG_PIN 34 // old 34 - don't know what this does
#define BATT_ANALOG_PIN 35
#define MOTOR_PWM_PIN 19
// LittleFS initialization
#include <Arduino.h>
//#include <LITTLEFS.h>  //Old LOROL version of littleFS.  New one is included in Arduino ESP32 Core
#include <LittleFS.h>
bool Status_File_System = false;
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
// Serial communication implementation
char StringInputSpeicher[500];
void InitialiseSerial(int BaudRate) {
  Serial.begin(BaudRate); // Start serial monitor at 115200 (ESP32)
  delay(500);
  Serial.println("Serial connection established.");
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
#include "BluetoothSerial.h"
bool BTSwitchedOn = false;
BluetoothSerial ESP_BT; //Object for Bluetooth
char BluetoothInputSpeicher[500];
bool readBTSerialTo(char BTInputBuffer[]) {
  const int BTSERIAL_BUFFER_SIZE = 500;
  static char BTserialBuffer[BTSERIAL_BUFFER_SIZE];
  static byte BTindex;
  while (ESP_BT.available()) {
    char BTc = ESP_BT.read();
    if (BTc == '\n' && BTindex > 0) {
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
// RFD communication implementation
#include <HardwareSerial.h>
HardwareSerial SerialRFD(1); // Hardware serial object = RFD object
char RFDStringInputSpeicher[500];
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
  long long voltage = 0;
  if (anzahl_spannungs_messung >= 300000) {
    anzahl_spannungs_messung = 300000;
  }
  if (anzahl_spannungs_messung <= 0) {
    return 0;
  }
  delay(3);
  for (int i = 0; i < anzahl_spannungs_messung; i++) {
    voltage = voltage + analogRead(volt_pin);
  }
  int total_adc_res = int(pow(2, x_bit_adc));
  si_voltage = float(voltage) / float(anzahl_spannungs_messung);
  si_voltage = (si_voltage * max_voltage) / total_adc_res;
  return si_voltage;
}
// BME implementation
// BME communication via I2C Pins
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // BME object
float Temperature = 0;
float Pressure = 0;
float Humidity = 0;
bool StatusBMESensor = false;
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
// GPS implementation
// Communication via serial connection
#include <HardwareSerial.h>
HardwareSerial GPSSerial(2);
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&GPSSerial); // GPS object
char c;
bool status_GPS_module = false;
int GPS_timeout = 4000;
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
// S300 Sensor
int co2 = 0;
#include "s300i2c.h"
#include <Wire.h>
S300I2C s3(Wire); // S300 object
bool status_s300 = false;
bool initialize_s300() {
  Wire.begin();
  s3.begin(S300I2C_ADDR);
  delay(10000);
  s3.wakeup();
  s3.end_mcdl();
  s3.end_acdl();
  Serial.println("S300 CO2 Initialized");
  return true;
}
int get_co2() {
  co2 = s3.getCO2ppm();
  return co2;
}
int pump_pwm = 0;
String filename = "/datalogger.txt";
void setup() {
  // put your setup code here, to run once:
  InitialiseSerial(115200);
  InitialiseRFDSerial();
  InitialiseBluetooth();
  pinMode(FLOW_ANALOG_PIN, INPUT);
  pinMode(SO2_ANALOG_PIN, INPUT);
  pinMode(NC_ANALOG_PIN, INPUT);
  Status_File_System = initialize_littlefs_format_file_system();
  if (Status_File_System) {
    Serial.println("File system initialized");
  }
  StatusBMESensor = initialize_bme_sensor();
  status_GPS_module = initialize_GPS();
  status_s300 = initialize_s300();
  // PWM Pin implementation (pump)
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  // ledcSetup(0, 200, 8); Old LEDC
  // ledcAttachPin(19, 0); Old LEDC
  ledcAttach(MOTOR_PWM_PIN,200,8);  // Migrated fro 2.x to 3.x ESP core
  Serial.println("Setup finished");
}
void loop() {
  // put your main code here, to run repeatedly:
  String write_to_file_string = "";
  write_to_file_string += String(millis());
  write_to_file_string += ",";
  // Try to read serial input and execute command
  if (readSerialTo(StringInputSpeicher)) {
    Serial.println("Serial input recieved!");
    Serial.println(StringInputSpeicher);
    if ((strcmp(StringInputSpeicher, "help")) == 0) {
      Serial.print("Print Datalogging file to serial terminal: ");
      Serial.println("'print data'");
      Serial.print("Delete Datalogging file: ");
      Serial.println("'delete data'");
      Serial.print("Switch pump on: ");
      Serial.println("'pump_on'");
      Serial.print("Switch pump off: ");
      Serial.println("'pump_off'");
      Serial.print("Switch pump 10/255: ");
      Serial.println("'10'");
      Serial.print("Switch pump 20/255: ");
      Serial.println("'20'");
      Serial.print("Switch pump 30/255: ");
      Serial.println("'30'");
      Serial.println("...");
      Serial.println();
    }
    else if ((strcmp(StringInputSpeicher, "print data")) == 0) {
      read_file_and_print_to_serial(filename);
    }
    else if ((strcmp(StringInputSpeicher, "delete data")) == 0) {
      delete_file(filename);
    }
    else if ((strcmp(StringInputSpeicher, "pump_on")) == 0) {
      digitalWrite(26, HIGH);
      ledcWrite(0, 254);
      pump_pwm = 254;
    }
    else if ((strcmp(StringInputSpeicher, "pump_off")) == 0) {
      digitalWrite(26, LOW);
      ledcWrite(0, 0);
      pump_pwm = 0;
    }
    else if ((strcmp(StringInputSpeicher, "0")) == 0) {
      ledcWrite(0, 0);
      pump_pwm = 0;
    }
    else if ((strcmp(StringInputSpeicher, "10")) == 0) {
      ledcWrite(0, 10);
      pump_pwm = 10;
    }
    else if ((strcmp(StringInputSpeicher, "20")) == 0) {
      ledcWrite(0, 20);
      pump_pwm = 20;
    }
    else if ((strcmp(StringInputSpeicher, "30")) == 0) {
      ledcWrite(0, 30);
      pump_pwm = 30;
    }
    else if ((strcmp(StringInputSpeicher, "40")) == 0) {
      ledcWrite(0, 40);
      pump_pwm = 40;
    }
    else if ((strcmp(StringInputSpeicher, "50")) == 0) {
      ledcWrite(0, 50);
      pump_pwm = 50;
    }
    else if ((strcmp(StringInputSpeicher, "60")) == 0) {
      ledcWrite(0, 60);
      pump_pwm = 60;
    }
    else if ((strcmp(StringInputSpeicher, "70")) == 0) {
      ledcWrite(0, 70);
      pump_pwm = 70;
    }
    else if ((strcmp(StringInputSpeicher, "80")) == 0) {
      ledcWrite(0, 80);
      pump_pwm = 80;
    }
    else if ((strcmp(StringInputSpeicher, "90")) == 0) {
      ledcWrite(0, 90);
      pump_pwm = 90;
    }
    else if ((strcmp(StringInputSpeicher, "100")) == 0) {
      ledcWrite(0, 100);
      pump_pwm = 100;
    }
    else if ((strcmp(StringInputSpeicher, "110")) == 0) {
      ledcWrite(0, 110);
      pump_pwm = 110;
    }
    else if ((strcmp(StringInputSpeicher, "120")) == 0) {
      ledcWrite(0, 120);
      pump_pwm = 120;
    }
    else if ((strcmp(StringInputSpeicher, "130")) == 0) {
      ledcWrite(0, 130);
      pump_pwm = 130;
    }
    else if ((strcmp(StringInputSpeicher, "140")) == 0) {
      ledcWrite(0, 140);
      pump_pwm = 140;
    }
    else if ((strcmp(StringInputSpeicher, "150")) == 0) {
      ledcWrite(0, 150);
      pump_pwm = 150;
    }
    else if ((strcmp(StringInputSpeicher, "160")) == 0) {
      ledcWrite(0, 160);
      pump_pwm = 160;
    }
    else if ((strcmp(StringInputSpeicher, "170")) == 0) {
      ledcWrite(0, 170);
      pump_pwm = 170;
    }
    else if ((strcmp(StringInputSpeicher, "180")) == 0) {
      ledcWrite(0, 180);
      pump_pwm = 180;
    }
    else if ((strcmp(StringInputSpeicher, "190")) == 0) {
      ledcWrite(0, 190);
      pump_pwm = 190;
    }
    else if ((strcmp(StringInputSpeicher, "200")) == 0) {
      ledcWrite(0, 200);
      pump_pwm = 200;
    }
    else if ((strcmp(StringInputSpeicher, "210")) == 0) {
      ledcWrite(0, 210);
      pump_pwm = 210;
    }
    else if ((strcmp(StringInputSpeicher, "220")) == 0) {
      ledcWrite(0, 220);
      pump_pwm = 220;
    }
    else if ((strcmp(StringInputSpeicher, "230")) == 0) {
      ledcWrite(0, 230);
      pump_pwm = 230;
    }
    else if ((strcmp(StringInputSpeicher, "240")) == 0) {
      ledcWrite(0, 240);
      pump_pwm = 240;
    }
    else if ((strcmp(StringInputSpeicher, "250")) == 0) {
      ledcWrite(0, 250);
      pump_pwm = 250;
    }
  }
  if (readBTSerialTo(BluetoothInputSpeicher)) {
    ESP_BT.println("Serial input recieved!");
    ESP_BT.println(BluetoothInputSpeicher);
    if ((strcmp(BluetoothInputSpeicher, "help")) == 0) {
      ESP_BT.print("Print Datalogging file to serial terminal: ");
      ESP_BT.println("'print data'");
      ESP_BT.print("Delete Datalogging file: ");
      ESP_BT.println("'delete data'");
      ESP_BT.print("Switch pump on: ");
      ESP_BT.println("'pump_on'");
      ESP_BT.print("Switch pump off: ");
      ESP_BT.println("'pump_off'");
      ESP_BT.print("Switch pump 10/255: ");
      ESP_BT.println("'10'");
      ESP_BT.print("Switch pump 20/255: ");
      ESP_BT.println("'20'");
      ESP_BT.print("Switch pump 30/255: ");
      ESP_BT.println("'30'");
      ESP_BT.println("...");
      ESP_BT.println();
    }
    else if ((strcmp(BluetoothInputSpeicher, "print data")) == 0) {
      read_file_and_print_to_serial(filename);
    }
    else if ((strcmp(BluetoothInputSpeicher, "delete data")) == 0) {
      delete_file(filename);
    }
    else if ((strcmp(BluetoothInputSpeicher, "pump_on")) == 0) {
      digitalWrite(26, HIGH);
      ledcWrite(0, 254);
      pump_pwm = 254;
    }
    else if ((strcmp(BluetoothInputSpeicher, "pump_off")) == 0) {
      digitalWrite(26, LOW);
      ledcWrite(0, 0);
      pump_pwm = 0;
    }
    else if ((strcmp(BluetoothInputSpeicher, "0")) == 0) {
      ledcWrite(0, 0);
      pump_pwm = 0;
    }
    else if ((strcmp(BluetoothInputSpeicher, "10")) == 0) {
      ledcWrite(0, 10);
      pump_pwm = 10;
    }
    else if ((strcmp(BluetoothInputSpeicher, "20")) == 0) {
      ledcWrite(0, 20);
      pump_pwm = 20;
    }
    else if ((strcmp(BluetoothInputSpeicher, "30")) == 0) {
      ledcWrite(0, 30);
      pump_pwm = 30;
    }
    else if ((strcmp(BluetoothInputSpeicher, "40")) == 0) {
      ledcWrite(0, 40);
      pump_pwm = 40;
    }
    else if ((strcmp(BluetoothInputSpeicher, "50")) == 0) {
      ledcWrite(0, 50);
      pump_pwm = 50;
    }
    else if ((strcmp(BluetoothInputSpeicher, "60")) == 0) {
      ledcWrite(0, 60);
      pump_pwm = 60;
    }
    else if ((strcmp(BluetoothInputSpeicher, "70")) == 0) {
      ledcWrite(0, 70);
      pump_pwm = 70;
    }
    else if ((strcmp(BluetoothInputSpeicher, "80")) == 0) {
      // Try to read Bluetooth serial input and execute command
      ledcWrite(0, 80);
      pump_pwm = 80;
    }
    else if ((strcmp(BluetoothInputSpeicher, "90")) == 0) {
      ledcWrite(0, 90);
      pump_pwm = 90;
    }
    else if ((strcmp(BluetoothInputSpeicher, "100")) == 0) {
      ledcWrite(0, 100);
      pump_pwm = 100;
    }
    else if ((strcmp(BluetoothInputSpeicher, "110")) == 0) {
      ledcWrite(0, 110);
      pump_pwm = 110;
    }
    else if ((strcmp(BluetoothInputSpeicher, "120")) == 0) {
      ledcWrite(0, 120);
      pump_pwm = 120;
    }
    else if ((strcmp(BluetoothInputSpeicher, "130")) == 0) {
      ledcWrite(0, 130);
      pump_pwm = 130;
    }
    else if ((strcmp(BluetoothInputSpeicher, "140")) == 0) {
      ledcWrite(0, 140);
      pump_pwm = 140;
    }
    else if ((strcmp(BluetoothInputSpeicher, "150")) == 0) {
      ledcWrite(0, 150);
      pump_pwm = 150;
    }
    else if ((strcmp(BluetoothInputSpeicher, "160")) == 0) {
      ledcWrite(0, 160);
      pump_pwm = 160;
    }
    else if ((strcmp(BluetoothInputSpeicher, "170")) == 0) {
      ledcWrite(0, 170);
      pump_pwm = 170;
    }
    else if ((strcmp(BluetoothInputSpeicher, "180")) == 0) {
      ledcWrite(0, 180);
      pump_pwm = 180;
    }
    else if ((strcmp(BluetoothInputSpeicher, "190")) == 0) {
      ledcWrite(0, 190);
      pump_pwm = 190;
    }
    else if ((strcmp(BluetoothInputSpeicher, "200")) == 0) {
      ledcWrite(0, 200);
      pump_pwm = 200;
    }
    else if ((strcmp(BluetoothInputSpeicher, "210")) == 0) {
      ledcWrite(0, 210);
      pump_pwm = 210;
    }
    else if ((strcmp(BluetoothInputSpeicher, "220")) == 0) {
      ledcWrite(0, 220);
      pump_pwm = 220;
    }
    else if ((strcmp(BluetoothInputSpeicher, "230")) == 0) {
      ledcWrite(0, 230);
      pump_pwm = 230;
    }
    else if ((strcmp(BluetoothInputSpeicher, "240")) == 0) {
      ledcWrite(0, 240);
      pump_pwm = 240;
    }
    else if ((strcmp(BluetoothInputSpeicher, "250")) == 0) {
      ledcWrite(0, 250);
      pump_pwm = 250;
    }
  }
  // Try to read RFD serial input and execute command
  if (readSerialRFDTo(RFDStringInputSpeicher)) {
    SerialRFD.println("Serial input recieved!");
    SerialRFD.println(RFDStringInputSpeicher);
    if ((strcmp(RFDStringInputSpeicher, "help")) == 0) {
      SerialRFD.print("Print Datalogging file to serial terminal: ");
      SerialRFD.println("'print data'");
      SerialRFD.print("Delete Datalogging file: ");
      SerialRFD.println("'delete data'");
      SerialRFD.print("Switch pump on: ");
      SerialRFD.println("'pump_on'");
      SerialRFD.print("Switch pump off: ");
      SerialRFD.println("'pump_off'");
      SerialRFD.print("Switch pump 10/255: ");
      SerialRFD.println("'10'");
      SerialRFD.print("Switch pump 20/255: ");
      SerialRFD.println("'20'");
      SerialRFD.print("Switch pump 30/255: ");
      SerialRFD.println("'30'");
      SerialRFD.println("...");
      SerialRFD.println();
    }
    else if ((strcmp(RFDStringInputSpeicher, "print data")) == 0) {
      read_file_and_print_to_serial(filename);
    }
    else if ((strcmp(RFDStringInputSpeicher, "delete data")) == 0) {
      delete_file(filename);
    }
    else if ((strcmp(RFDStringInputSpeicher, "pump_on")) == 0) {
      digitalWrite(26, HIGH);
      ledcWrite(0, 254);
      pump_pwm = 254;
    }
    else if ((strcmp(RFDStringInputSpeicher, "pump_off")) == 0) {
      digitalWrite(26, LOW);
      ledcWrite(0, 0);
      pump_pwm = 0;
    }
    else if ((strcmp(RFDStringInputSpeicher, "0")) == 0) {
      ledcWrite(0, 0);
      pump_pwm = 0;
    }
    else if ((strcmp(RFDStringInputSpeicher, "10")) == 0) {
      ledcWrite(0, 10);
      pump_pwm = 10;
    }
    else if ((strcmp(RFDStringInputSpeicher, "20")) == 0) {
      ledcWrite(0, 20);
      pump_pwm = 20;
    }
    else if ((strcmp(RFDStringInputSpeicher, "30")) == 0) {
      ledcWrite(0, 30);
      pump_pwm = 30;
    }
    else if ((strcmp(RFDStringInputSpeicher, "40")) == 0) {
      ledcWrite(0, 40);
      pump_pwm = 40;
    }
    else if ((strcmp(RFDStringInputSpeicher, "50")) == 0) {
      ledcWrite(0, 50);
      pump_pwm = 50;
    }
    else if ((strcmp(RFDStringInputSpeicher, "60")) == 0) {
      ledcWrite(0, 60);
      pump_pwm = 60;
    }
    else if ((strcmp(RFDStringInputSpeicher, "70")) == 0) {
      ledcWrite(0, 70);
      pump_pwm = 70;
    }
    else if ((strcmp(RFDStringInputSpeicher, "80")) == 0) {
      ledcWrite(0, 80);
      pump_pwm = 80;
    }
    else if ((strcmp(RFDStringInputSpeicher, "90")) == 0) {
      ledcWrite(0, 90);
      pump_pwm = 90;
    }
    else if ((strcmp(RFDStringInputSpeicher, "100")) == 0) {
      ledcWrite(0, 100);
      pump_pwm = 100;
    }
    else if ((strcmp(RFDStringInputSpeicher, "110")) == 0) {
      ledcWrite(0, 110);
      pump_pwm = 110;
    }
    else if ((strcmp(RFDStringInputSpeicher, "120")) == 0) {
      ledcWrite(0, 120);
      pump_pwm = 120;
    }
    else if ((strcmp(RFDStringInputSpeicher, "130")) == 0) {
      ledcWrite(0, 130);
      pump_pwm = 130;
    }
    else if ((strcmp(RFDStringInputSpeicher, "140")) == 0) {
      ledcWrite(0, 140);
      pump_pwm = 140;
    }
    else if ((strcmp(RFDStringInputSpeicher, "150")) == 0) {
      ledcWrite(0, 150);
      pump_pwm = 150;
    }
    else if ((strcmp(RFDStringInputSpeicher, "160")) == 0) {
      ledcWrite(0, 160);
      pump_pwm = 160;
    }
    else if ((strcmp(RFDStringInputSpeicher, "170")) == 0) {
      ledcWrite(0, 170);
      pump_pwm = 170;
    }
    else if ((strcmp(RFDStringInputSpeicher, "180")) == 0) {
      ledcWrite(0, 180);
      pump_pwm = 180;
    }
    else if ((strcmp(RFDStringInputSpeicher, "190")) == 0) {
      ledcWrite(0, 190);
      pump_pwm = 190;
    }
    else if ((strcmp(RFDStringInputSpeicher, "200")) == 0) {
      ledcWrite(0, 200);
      pump_pwm = 200;
    }
    else if ((strcmp(RFDStringInputSpeicher, "210")) == 0) {
      ledcWrite(0, 210);
      pump_pwm = 210;
    }
    else if ((strcmp(RFDStringInputSpeicher, "220")) == 0) {
      ledcWrite(0, 220);
      pump_pwm = 220;
    }
    else if ((strcmp(RFDStringInputSpeicher, "230")) == 0) {
      ledcWrite(0, 230);
      pump_pwm = 230;
    }
    else if ((strcmp(RFDStringInputSpeicher, "240")) == 0) {
      ledcWrite(0, 240);
      pump_pwm = 240;
    }
    else if ((strcmp(RFDStringInputSpeicher, "250")) == 0) {
      ledcWrite(0, 250);
      pump_pwm = 250;
    }
  }
  // Get data from BME Sensor
  if (StatusBMESensor) {
    Temperature = bme.readTemperature();
    Pressure = bme.readPressure();
    Pressure = Pressure / 100;
    Humidity = bme.readHumidity();
    write_to_file_string += String(Temperature);
    write_to_file_string += ",";
    write_to_file_string += String(Pressure);
    write_to_file_string += ",";
    write_to_file_string += String(Humidity);
    write_to_file_string += ",";
  }
  else {
    Serial.println("No BME sensor available!!");
    ESP_BT.println("No BME sensor available!!");
    SerialRFD.println("No BME sensor available!!");
    StatusBMESensor = initialize_bme_sensor();
    write_to_file_string += "No BME Sensor available,,,";
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
  if (SI_voltage_pin_SO2 >= 3.7) {
    Serial.print("Problem with ADC on Pin SO2; measured voltage=");
    Serial.println(SI_voltage_pin_SO2);
    ESP_BT.print("Problem with ADC on Pin SO2; measured voltage=");
    ESP_BT.println(SI_voltage_pin_SO2);
    SerialRFD.print("Problem with ADC on Pin SO2; measured voltage=");
    SerialRFD.println(SI_voltage_pin_SO2);
  }
  // NC
  float SI_voltage_pin_NC = readSIVoltageFromPin(NC_ANALOG_PIN, 10, 12, 3.3);
  if (SI_voltage_pin_NC >= 3.7) {
    Serial.print("Problem with ADC on Pin NC; measured voltage=");
    Serial.println(SI_voltage_pin_NC);
    ESP_BT.print("Problem with ADC on Pin NC; measured voltage=");
    ESP_BT.println(SI_voltage_pin_NC);
    SerialRFD.print("Problem with ADC on Pin NC; measured voltage=");
    SerialRFD.println(SI_voltage_pin_NC);
  }
  write_to_file_string += String(SI_voltage_pin_FLOW, 4);
  write_to_file_string += ",";
  write_to_file_string += String(SI_voltage_pin_SO2, 4);
  write_to_file_string += ",";
  write_to_file_string += String(SI_voltage_pin_NC, 4);
  write_to_file_string += ",";
  // Get data from CO2 Sensor
  co2 = get_co2();
  write_to_file_string += String(co2);
  write_to_file_string += ",";
  // Get data from GPS Module
  clearGPS();
  int start_millis = millis();
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
    status_GPS_module = true;
    if (((millis() - start_millis) >= GPS_timeout) or (millis() < start_millis)) {
      status_GPS_module = false;
      break;
    }
  }
  if (status_GPS_module) {
    GPS.parse(GPS.lastNMEA());
    String last_NMEA = GPS.lastNMEA();
    write_to_file_string += last_NMEA;
  }
  else {
    status_GPS_module = initialize_GPS();
    Serial.println("GPS not working!");
    ESP_BT.println("GPS not working!");
    SerialRFD.println("GPS not working!");
    write_to_file_string += "GPS not working!\n";
  }
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
  delay(1000);
}
