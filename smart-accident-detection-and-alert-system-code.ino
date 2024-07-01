// Define constants for the sensors and modem
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024

#define EMERGENCY_SWITCH 23  // Define the pin number for the emergency switch
#define VIBRATION_SENSOR_PIN 13

//Libraries Required
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <StreamDebugger.h>

const char* simPIN = "";  // SIM card PIN (empty if PIN is not defined)
#define SMS_TARGET "+9779841000000"

// Sensor and GPS configuration
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
TinyGPSPlus gps;
StreamDebugger debugger(Serial1, Serial);
TinyGsm modem(debugger);

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address of LCD 

// IP5306 Power management IC settings
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

// Modem and I2C configuration
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

volatile bool cancelSMS = false;  // Flag to control SMS cancellation
bool smsSent = false;
float tiltThreshold = 0.5;
int16_t AcX, AcY, AcZ;

#define MOVING_AVERAGE_SIZE 10 //buffer size

int16_t AcX_buffer[MOVING_AVERAGE_SIZE] = { 0 };
int16_t AcY_buffer[MOVING_AVERAGE_SIZE] = { 0 };
int16_t AcZ_buffer[MOVING_AVERAGE_SIZE] = { 0 };
int bufferIndex = 0;

//adds new reading to current position of array and moves index to next position
void updateBuffer(int16_t* buffer, int16_t newValue) {  
  buffer[bufferIndex] = newValue;
  bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_SIZE;
}

int16_t calculateAverage(int16_t* buffer) {  
  int32_t sum = 0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / MOVING_AVERAGE_SIZE;
}

void setup() {
  lcd.init();  // Initializes the LCD
  lcd.backlight();
  initializeSerial();
  initializePowerManagement();
  initializeModem();
  initializeSensors();
  lcd.clear();
}

void loop() {
  readSensors();
  delay(100);
}

void initializeSerial() {
  Serial.begin(9600);
  Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  Serial.println("Serial initialized.");
  lcd.setCursor(0, 0);
  lcd.print("Serial Init Ready");
}

void initializePowerManagement() {
  Wire.begin(I2C_SDA, I2C_SCL);
  bool isOk = setPowerBoostKeepOn(1);
  Serial.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
  lcd.setCursor(0, 1);
  lcd.print("Power IC: " + String(isOk ? "Enabled  " : "Failed   "));
}

bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  Wire.write(en ? 0x37 : 0x35);
  return Wire.endTransmission() == 0;
}

void initializeModem() {
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  Serial.println("Initializing modem..");
  lcd.setCursor(0, 2);
  lcd.print("Modem Initializing..");
  modem.restart();
  if (strlen(simPIN) && modem.getSimStatus() != 3) {
    modem.simUnlock(simPIN);
  }
  if (modem.waitForNetwork() && modem.isNetworkConnected()) {
    Serial.println("Network connected");
    lcd.setCursor(0, 2);
    lcd.print("Network: Connected     ");
  } else {
    Serial.println("Not connected to network");
    lcd.setCursor(0, 2);
    lcd.print("Network Disconnected");
  }
}

void initializeSensors() {
  pinMode(EMERGENCY_SWITCH, INPUT_PULLUP); // when not pressed, reads high and when pressed, reads low
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_SWITCH), emergencyPressed, FALLING);  // switch transition from high to low
  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // MPU-6050 wakeup command
  Wire.write(0);     // Wakeup MPU-6050
  Wire.endTransmission(true);
  Serial.println("Sensors initialized.");
  lcd.setCursor(0, 3);
  lcd.print("Sensors Init Ready");
}

void readSensors() {
  processGPSData();

  bool isVibrated = analogRead(VIBRATION_SENSOR_PIN) < 500; //Active-low: less value results in more vibration
  Serial.print("Vibration Sensor");
  Serial.println(analogRead(VIBRATION_SENSOR_PIN));

  // Prioritize tilt detection, then check for vibration average less than 2
  if (checkForTilt()) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Tilt Detected!");
    handleAccident(gps.location.lat(), gps.location.lng());
    return;
  }
  if (isVibrated) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Vibration Detected!");
    handleAccident(gps.location.lat(), gps.location.lng());
    return;
  }
}

bool checkForTilt() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // request sent for acceleration data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
//reads data in two bytes per axis
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Update buffers with new readings
  updateBuffer(AcX_buffer, AcX);
  updateBuffer(AcY_buffer, AcY);
  updateBuffer(AcZ_buffer, AcZ);

  // Calculate moving average for each axis
  AcX = calculateAverage(AcX_buffer);
  AcY = calculateAverage(AcY_buffer);
  AcZ = calculateAverage(AcZ_buffer);

  // Compute tilt angles in degrees from radian
  float xAng = atan2(AcX, sqrt(AcY * AcY + AcZ * AcZ)) * 180.0 / PI;
  float yAng = atan2(AcY, sqrt(AcX * AcX + AcZ * AcZ)) * 180.0 / PI;

  Serial.print("Tilt Angle X: ");
  Serial.print(abs(xAng));
  Serial.print("°, Y: ");
  Serial.print(abs(yAng));
  Serial.println("°");

  // Defining accident condition here, i.e. tilt angle threshold of 50 degrees
  if (abs(xAng) > 50 || abs(yAng) > 50) {
    return true;
  }
  return false;
}

void processGPSData() {
  lcd.clear();
  while (Serial.available() > 0) { //reads data from serial port
    char c = Serial.read();
    gps.encode(c);
  }
  if (gps.location.isValid()) {
    lcd.setCursor(0, 1);
    lcd.print("GPS: " + String(gps.location.lat(), 6) + ", ");
    lcd.setCursor(0, 2);
    lcd.print(String(gps.location.lng(), 6));
  } else {
    lcd.setCursor(0, 1);
    lcd.print("GPS: Searching...       ");
  }
}

void handleAccident(float lat, float lng) {
  cancelSMS = false;   // Reset cancellation flag
  int countdown = 10;  // Countdown timer in seconds

  for (int i = countdown; i > 0; i--) {
    if (cancelSMS) {
      lcd.setCursor(0, 3);
      lcd.print("SMS Cancelled          ");
      return;  // Exit if SMS is cancelled
    }
    lcd.setCursor(0, 3);
    lcd.print("Sending SMS in " + String(i) + " sec ");
    delay(1000);  // Wait for 1 second
  }

  sendAccidentSMS(lat, lng);  // Call the SMS sending function
}

void sendAccidentSMS(float lat, float lng) {
  if (!smsSent) { //checks if message has already been sent
    String message;
    if (gps.location.isValid()) {
      message = "Accident at: http://maps.google.com/maps?q=" + String(lat, 6) + "," + String(lng, 6) + " Vehicle Number Ba Pra 3 6439";
    } else {
      message = "Accident detected, location unavailable. Vehicle Number Ba Pra 3 6439";
    }

    if (modem.sendSMS(SMS_TARGET, message)) {
      Serial.println("SMS sent: " + message);
      lcd.setCursor(0, 3);
      lcd.print("SMS Sent               ");
      smsSent = true;
    } else {
      Serial.println("Failed to send SMS: " + message);
      lcd.setCursor(0, 3);
      lcd.print("SMS Failed             ");
    }
  }
}

void emergencyPressed() {
  cancelSMS = true;  // Set the flag to cancel SMS when the interrupt occurs
}