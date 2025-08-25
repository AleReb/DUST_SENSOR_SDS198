#include <HardwareSerial.h>

// For the ESP32, it's better to use a hardware serial port for reliability.
// We can map the pins of a hardware serial port (e.g., Serial1) to any GPIO.
// Here, we map it to the pins you requested: RX=18, TX=19.
HardwareSerial sdsSerial(1);

// --- Protocol Constants ---
const byte HEADER = 0xAA;
const byte TAIL   = 0xAB;
// Per the datasheet, SDS198 replies with 0xCF. SDS011 uses 0xC0.
const byte CMD_SDS198 = 0xCF;
const byte CMD_SDS011 = 0xC0;


void setup() {
  // Start the main serial port for printing output to the Serial Monitor
  Serial.begin(115200);
  // Wait for the serial port to be ready
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("ESP32 SDS198 Sensor Reader");

  // Start the serial port connected to the SDS sensor.
  // Baud rate: 9600, Configuration: 8 data bits, no parity, 1 stop bit (SERIAL_8N1)
  // RX Pin: 18, TX Pin: 19
  sdsSerial.begin(9600, SERIAL_8N1, 18, 19);
}

void loop() {
  byte frame[10];

  // Try to read a data frame from the sensor
  if (readFrame(frame)) {
    
    // --- Frame Parsing ---
    // Check if the frame is from an SDS198 (PM100/TSP)
    if (frame[1] == CMD_SDS198) {
      int pm100 = parseSds198(frame);
      if (pm100 >= 0) {
        Serial.print("PM100 (TSP): ");
        Serial.print(pm100);
        Serial.println(" µg/m³");
      } else {
        Serial.print("[WARN] SDS198 checksum error. Raw: ");
        printHex(frame, 10);
      }
    } 
    // As a fallback, check if it's an SDS011-style frame
    else if (frame[1] == CMD_SDS011) {
      float pm25, pm10;
      if (parseSds011(frame, pm25, pm10)) {
        Serial.print("SDS011-like -> ");
        Serial.print("PM2.5: ");
        Serial.print(pm25, 1);
        Serial.print(" µg/m³, PM10: ");
        Serial.print(pm10, 1);
        Serial.println(" µg/m³");
      } else {
        Serial.print("[WARN] SDS011 checksum error. Raw: ");
        printHex(frame, 10);
      }
    }
    // If the command byte is unknown
    else {
      Serial.print("[WARN] Unexpected frame type. Raw: ");
      printHex(frame, 10);
    }
  }
  
  // Wait for 1 second before the next reading
  delay(1000);
}

/**
 * @brief Reads a 10-byte data frame from the sensor.
 * @param buffer A byte array of size 10 to store the frame.
 * @return true if a valid-looking frame was read, false otherwise.
 */
bool readFrame(byte* buffer) {
  if (sdsSerial.available() < 10) {
    return false; // Not enough data to form a complete frame
  }

  // 1. Look for the header byte
  if (sdsSerial.read() != HEADER) {
    return false;
  }
  
  buffer[0] = HEADER;

  // 2. Read the remaining 9 bytes of the frame
  size_t bytesRead = sdsSerial.readBytes(buffer + 1, 9);
  if (bytesRead < 9) {
    return false; // Incomplete frame
  }

  // 3. Check for the tail byte
  if (buffer[9] != TAIL) {
    return false;
  }

  return true; // Frame is structurally valid
}

/**
 * @brief Parses a frame from an SDS198 sensor.
 * Frame: [AA CF D1 D2 D3 D4 D5 D6 CHK AB]
 * @param frame The 10-byte data frame.
 * @return The PM100 (TSP) value, or -1 if checksum fails.
 */
int parseSds198(byte* frame) {
  // Checksum is the sum of data bytes [2] through [7]
  byte checksum = 0;
  for (int i = 2; i < 8; i++) {
    checksum += frame[i];
  }

  // Compare calculated checksum with the one in the frame
  if (checksum != frame[8]) {
    return -1; // Checksum error
  }

  // PM100 = High byte * 256 + Low byte
  int pm100 = (frame[5] << 8) | frame[4];
  return pm100;
}

/**
 * @brief Parses an SDS011-style data frame.
 * Frame: [AA C0 PM2.5_L PM2.5_H PM10_L PM10_H ID1 ID2 CHK AB]
 * @param frame The 10-byte data frame.
 * @param pm25 Reference to store the PM2.5 value.
 * @param pm10 Reference to store the PM10 value.
 * @return true if checksum is valid, false otherwise.
 */
bool parseSds011(byte* frame, float &pm25, float &pm10) {
  byte checksum = 0;
  for (int i = 2; i < 8; i++) {
    checksum += frame[i];
  }

  if (checksum != frame[8]) {
    return false; // Checksum error
  }

  pm25 = (((frame[3] << 8) | frame[2])) / 10.0;
  pm10 = (((frame[5] << 8) | frame[4])) / 10.0;
  return true;
}

/**
 * @brief Prints a byte array as a hex string to the Serial monitor.
 * @param data Pointer to the byte array.
 * @param len The length of the array.
 */
void printHex(byte* data, int len) {
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
