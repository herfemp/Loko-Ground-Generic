/*
 * Loko Ground Station Firmware - Arduino Version
 * Converted from MicroPython to Arduino C++ for ESP32-PICO-D4
 * 
 * Features:
 * - LoRa communication via UART
 * - BLE connectivity (Nordic UART Service)
 * - Settings management (stored in SPIFFS)
 * - Battery monitoring
 * - Data logging
 * - Command line interface
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <mbedtls/aes.h>
#include <mbedtls/base64.h>  // Use mbedtls base64

// Pin Definitions (for real hardware)
//#define VBAT_IN_PIN 39
//#define BUTTON_PIN 35
//#define POWER_CTRL_PIN 12
//#define LED_BLUE_PIN 21
#define LED_RED_PIN 18
//#define LED_GREEN_PIN 19
#define LORA_RX_PIN 33  // UART2 RX
#define LORA_TX_PIN 32  // UART2 TX

// Configuration
#define USE_COMMAND_LINE_PARSER true
#define MAX_LOG_ENTRIES 100
#define LOG_FILENAME "/lora_log.txt"
#define SETTINGS_FILENAME "/settings.json"

// BLE UUIDs (Nordic UART Service)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Global objects
HardwareSerial LoRaSerial(2);
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Settings structure
struct Settings {
    uint32_t id2;
    uint32_t freq;
    char p2p_key[65]; // 64 chars + null terminator
};

Settings settings = {0, 868000000, ""};

// Log entry structure
struct LogEntry {
    String timestamp;
    String data;
};

// Global variables
std::vector<LogEntry> logEntries;
int btCounter = 0;
bool exitRequest = false;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;

// Forward declarations
void loadSettings();
void saveSettings();
void addLogEntry(const String& data);
void clearLog();
String exportLogs();
void loraSet(uint32_t freq_hz);
void loraDataReceive();
String parseLoraModuleMessage(const String& message);
bool isHexAsciiConvertible(const String& hexString);
void parseLokoBinPacket(const String& binData, uint8_t* key, JsonDocument& result);
void parseLokoStringPacket(const String& str, uint8_t* key, JsonDocument& result);
float batteryLevel();
void processCommand(const String& cmdLine);
void setupBLE();
void sendBLE(const String& data);
float binUnpackVbat(uint8_t vb);
float binUnpackLatLon24(const uint8_t* packed);
float binUnpackLatLon32(const uint8_t* packed);

// ============================================================================
// BLE Callbacks
// ============================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
       // digitalWrite(LED_BLUE_PIN, LOW); // LED on
        Serial.println("BLE Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Disconnected");
        delay(500);
        pServer->startAdvertising(); // Restart advertising
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.print("BLE Rx: ");
            Serial.println(rxValue.c_str());
        }
    }
};

// ============================================================================
// Settings Management
// ============================================================================
void loadSettings() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }

    File file = SPIFFS.open(SETTINGS_FILENAME, "r");
    if (!file) {
        Serial.println("Settings file not found, using defaults");
        strcpy(settings.p2p_key, "0000000000000000000000000000000000000000000000000000000000000000");
        saveSettings();
        return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.println("Failed to parse settings");
        return;
    }

    settings.id2 = doc["id2"] | 0;
    settings.freq = doc["freq"] | 868000000;
    
    // Handle frequency conversion from MHz to Hz if needed
    if (settings.freq < 1000) {
        settings.freq = settings.freq * 1000000;
        saveSettings();
    }

    const char* key = doc["p2p_key"] | "0000000000000000000000000000000000000000000000000000000000000000";
    strncpy(settings.p2p_key, key, 64);
    settings.p2p_key[64] = '\0';

    Serial.printf("Loaded settings - ID2: %d, Freq: %d Hz, Key: %s\n", 
                  settings.id2, settings.freq, settings.p2p_key);
}

void saveSettings() {
    StaticJsonDocument<512> doc;
    doc["id2"] = settings.id2;
    doc["freq"] = settings.freq;
    doc["p2p_key"] = settings.p2p_key;

    File file = SPIFFS.open(SETTINGS_FILENAME, "w");
    if (!file) {
        Serial.println("Failed to open settings file for writing");
        return;
    }

    serializeJson(doc, file);
    file.close();
    Serial.println("Settings saved");
}

// ============================================================================
// Log Management
// ============================================================================
void loadLogs() {
    File file = SPIFFS.open(LOG_FILENAME, "r");
    if (!file) {
        Serial.println("No existing log file");
        return;
    }

    logEntries.clear();
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            int endBracket = line.indexOf(']');
            if (endBracket > 0) {
                LogEntry entry;
                entry.timestamp = line.substring(1, endBracket);
                entry.data = line.substring(endBracket + 2);
                if (logEntries.size() < MAX_LOG_ENTRIES) {
                    logEntries.push_back(entry);
                }
            }
        }
    }
    file.close();
}

void addLogEntry(const String& data) {
    LogEntry entry;
    
    // Create timestamp
    unsigned long now = millis();
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "LOG-%lu", now);
    entry.timestamp = String(timeStr);
    entry.data = data;

    // Manage max entries
    if (logEntries.size() >= MAX_LOG_ENTRIES) {
        logEntries.erase(logEntries.begin());
    }
    logEntries.push_back(entry);

    // Append to file
    File file = SPIFFS.open(LOG_FILENAME, "a");
    if (file) {
        file.printf("[%s] %s\n", entry.timestamp.c_str(), entry.data.c_str());
        file.close();
    }
}

void clearLog() {
    logEntries.clear();
    SPIFFS.remove(LOG_FILENAME);
    Serial.println("Log cleared");
}

String exportLogs() {
    String output = "";
    for (const auto& entry : logEntries) {
        output += "[" + entry.timestamp + "] " + entry.data + "\n";
    }
    return output;
}

// ============================================================================
// Battery Monitoring
// ============================================================================
//float batteryLevel() {
//    analogReadResolution(12);
//    analogSetAttenuation(ADC_11db);
//    
//    int adcReading = analogRead(VBAT_IN_PIN);
//    
//    float maxAdcValue = 2455.0;
//    float maxBatteryVoltage = 2.1;
//    float adcBatteryVoltage = 2.0 * (adcReading * maxBatteryVoltage / maxAdcValue);
//    
//    return adcBatteryVoltage;
//}

// ============================================================================
// LoRa Communication
// ============================================================================
void loraSet(uint32_t freq_hz) {
    uint32_t freq_mhz = freq_hz / 1000000;
    
    LoRaSerial.println("AT+MODE=TEST");
    delay(1000);
    while (LoRaSerial.available()) {
        Serial.print("Lora Resp: ");
        Serial.println(LoRaSerial.readString());
    }
    
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+TEST=RFCFG,%d,SF12,125,12,15,14,ON,OFF,OFF", freq_mhz);
    LoRaSerial.println(cmd);
    delay(1000);
    while (LoRaSerial.available()) {
        Serial.print("Lora Resp: ");
        Serial.println(LoRaSerial.readString());
    }
}

void loraDataReceive() {
    LoRaSerial.println("AT+TEST=RXLRPKT");
    delay(500);
    while (LoRaSerial.available()) {
        Serial.print("Lora RX: ");
        Serial.println(LoRaSerial.readString());
    }
}

String parseLoraModuleMessage(const String& message) {
    // Expected: '+TEST: LEN:31, RSSI:-35, SNR:12\r\n+TEST: RX "30302C..."'
    int rxPos = message.indexOf("RX \"");
    if (rxPos == -1) return "";
    
    int startPos = rxPos + 4;
    int endPos = message.indexOf("\"", startPos);
    if (endPos == -1) return "";
    
    return message.substring(startPos, endPos);
}

// ============================================================================
// Hex/Binary utilities
// ============================================================================
bool isHexAsciiConvertible(const String& hexString) {
    if (hexString.length() % 2 != 0) return false;
    
    for (size_t i = 0; i < hexString.length(); i++) {
        char c = hexString[i];
        if (!isxdigit(c)) return false;
    }
    
    // Check if decoded bytes are printable ASCII
    for (size_t i = 0; i < hexString.length(); i += 2) {
        String byteStr = hexString.substring(i, i + 2);
        int byteVal = strtol(byteStr.c_str(), nullptr, 16);
        if (byteVal < 32 || byteVal > 126) return false;
    }
    
    return true;
}

void hexStringToBytes(const String& hexStr, uint8_t* output, size_t maxLen) {
    size_t len = hexStr.length() / 2;
    if (len > maxLen) len = maxLen;
    
    for (size_t i = 0; i < len; i++) {
        String byteStr = hexStr.substring(i * 2, i * 2 + 2);
        output[i] = (uint8_t)strtol(byteStr.c_str(), nullptr, 16);
    }
}

String hexToAscii(const String& hexStr) {
    String result = "";
    for (size_t i = 0; i < hexStr.length(); i += 2) {
        String byteStr = hexStr.substring(i, i + 2);
        char c = (char)strtol(byteStr.c_str(), nullptr, 16);
        result += c;
    }
    return result;
}

// Base64 decode helper using mbedtls
size_t base64Decode(const String& input, uint8_t* output, size_t outputLen) {
    // Remove any whitespace or padding issues
    String cleanInput = input;
    cleanInput.trim();
    
    // Use mbedtls base64 decode
    size_t outLen = 0;
    int ret = mbedtls_base64_decode(output, outputLen, &outLen,
                                     (const unsigned char*)cleanInput.c_str(),
                                     cleanInput.length());
    
    if (ret == 0) {
        return outLen;  // Success
    }
    
    return 0;  // Failed
}

// ============================================================================
// Binary unpacking utilities
// ============================================================================
float binUnpackVbat(uint8_t vb) {
    return (vb + 27) * 0.1;
}

float binUnpackLatLon24(const uint8_t* packed) {
    int32_t latLonScaled = (packed[0] << 16) | (packed[1] << 8) | packed[2];
    
    // Handle sign extension for 24-bit signed
    if (latLonScaled & 0x800000) {
        latLonScaled -= 0x1000000;
    }
    
    return latLonScaled / 10000.0;
}

float binUnpackLatLon32(const uint8_t* packed) {
    int32_t latLonScaled = (packed[0] << 24) | (packed[1] << 16) | 
                           (packed[2] << 8) | packed[3];
    
    return latLonScaled / 1000000.0;
}

// ============================================================================
// Packet Parsing
// ============================================================================
void parseLokoStringPacket(const String& str, uint8_t* key, JsonDocument& result) {
    // Split by comma
    int idx = 0;
    String values[10];
    int lastIdx = 0;
    
    for (int i = 0; i <= str.length(); i++) {
        if (i == str.length() || str[i] == ',') {
            values[idx++] = str.substring(lastIdx, i);
            lastIdx = i + 1;
            if (idx >= 10) break;
        }
    }
    
    if (idx == 5) {
        // Format: '123,321,40.376123,49.850848,3420'
        result["id1"] = values[0].toInt();
        result["id2"] = values[1].toInt();
        result["lat"] = values[2];
        result["lon"] = values[3];
        result["vbat"] = values[4].toInt();
    }
    else if (idx == 7) {
        // Format: '00,000,54.685349,25.282091,117,0,6432'
        result["id1"] = values[0].toInt();
        result["id2"] = values[1].toInt();
        result["lat"] = values[2];
        result["lon"] = values[3];
        result["alt"] = values[4].toInt();
        result["mps"] = values[5].toInt();
        result["vbat"] = values[6].toInt();
    }
    else if (idx == 3) {
        // Format: '00,000,KsC72EMf5cAYJU8eATDTMg==' (encrypted)
        result["id1"] = values[0].toInt();
        result["id2"] = values[1].toInt();
        
        // Decode base64
        String base64Str = values[2];
        uint8_t encryptedBytes[64]; // Should be enough for 16 bytes encrypted + base64 overhead
        size_t decodedLen = base64Decode(base64Str, encryptedBytes, sizeof(encryptedBytes));
        
        if (decodedLen >= 16) {
            // Decrypt with AES ECB
            mbedtls_aes_context aes;
            mbedtls_aes_init(&aes);
            mbedtls_aes_setkey_dec(&aes, key, 256);
            
            uint8_t decryptedBytes[16];
            mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, encryptedBytes, decryptedBytes);
            mbedtls_aes_free(&aes);
            
            // Verify checksum
            uint8_t checksum = 0;
            for (int i = 0; i < 15; i++) {
                checksum += decryptedBytes[i];
            }
            checksum = checksum % 256;
            
            if (checksum == decryptedBytes[15]) {
                float lat, lon;
                uint16_t vbat_mv, alt_meters, speed_mps;
                uint8_t reserved1, integrity;
                
                memcpy(&lat, &decryptedBytes[0], 4);
                memcpy(&lon, &decryptedBytes[4], 4);
                memcpy(&vbat_mv, &decryptedBytes[8], 2);
                memcpy(&alt_meters, &decryptedBytes[10], 2);
                memcpy(&speed_mps, &decryptedBytes[12], 2);
                
                result["lat"] = lat;
                result["lon"] = lon;
                result["vbat"] = vbat_mv;
                result["alt"] = alt_meters;
                result["mps"] = speed_mps;
            } else {
                Serial.println("Can't decrypt, possible wrong key");
            }
        } else {
            Serial.println("Base64 decode failed");
        }
    }
}

void parseLokoBinPacket(const String& binData, uint8_t* key, JsonDocument& result) {
    uint8_t data[256];
    hexStringToBytes(binData, data, sizeof(data));
    size_t dataLen = binData.length() / 2;
    
    uint32_t id1 = 0, id2 = 0;
    float lat = 0.0, lon = 0.0;
    uint16_t vbat_mv = 0;
    uint16_t alt_meters = 0;
    uint8_t speed_mps = 0;
    uint8_t packet_version = 0;
    
    if (dataLen == 15) {
        memcpy(&id1, &data[0], 4);
        memcpy(&id2, &data[4], 4);
        uint8_t vb_version = data[8];
        packet_version = (vb_version >> 4) & 0x0F;
        vbat_mv = binUnpackVbat(vb_version & 0x0F);
        lat = binUnpackLatLon24(&data[9]);
        lon = binUnpackLatLon24(&data[12]);
    }
    else if (dataLen == 17) {
        memcpy(&id1, &data[0], 4);
        memcpy(&id2, &data[4], 4);
        uint8_t vb_version = data[8];
        packet_version = (vb_version >> 4) & 0x0F;
        vbat_mv = binUnpackVbat(vb_version & 0x0F);
        lat = binUnpackLatLon32(&data[9]);
        lon = binUnpackLatLon32(&data[13]);
    }
    else if (dataLen == 18) {
        memcpy(&id1, &data[0], 4);
        memcpy(&id2, &data[4], 4);
        uint8_t vb_version = data[8];
        packet_version = (vb_version >> 4) & 0x0F;
        vbat_mv = binUnpackVbat(vb_version & 0x0F);
        lat = binUnpackLatLon24(&data[9]);
        lon = binUnpackLatLon24(&data[12]);
        speed_mps = data[15];
        memcpy(&alt_meters, &data[16], 2);
    }
    else if (dataLen == 20) {
        memcpy(&id1, &data[0], 4);
        memcpy(&id2, &data[4], 4);
        uint8_t vb_version = data[8];
        packet_version = (vb_version >> 4) & 0x0F;
        vbat_mv = binUnpackVbat(vb_version & 0x0F);
        lat = binUnpackLatLon32(&data[9]);
        lon = binUnpackLatLon32(&data[13]);
        speed_mps = data[17];
        memcpy(&alt_meters, &data[18], 2);
    }
    else if (dataLen == 25) {
        // Encrypted packet
        memcpy(&id1, &data[0], 4);
        memcpy(&id2, &data[4], 4);
        uint8_t vb_version = data[8];
        packet_version = (vb_version >> 4) & 0x0F;
        
        // Decrypt AES payload
        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        mbedtls_aes_setkey_dec(&aes, key, 256);
        
        uint8_t decryptedBytes[16];
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, &data[9], decryptedBytes);
        mbedtls_aes_free(&aes);
        
        // Verify checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 15; i++) {
            checksum += decryptedBytes[i];
        }
        checksum = checksum % 256;
        
        if (checksum == decryptedBytes[15]) {
            if (vb_version == 2) {
                vbat_mv = binUnpackVbat(decryptedBytes[0] & 0x0F);
                lat = binUnpackLatLon24(&decryptedBytes[1]);
                lon = binUnpackLatLon24(&decryptedBytes[4]);
                speed_mps = decryptedBytes[7];
                memcpy(&alt_meters, &decryptedBytes[8], 2);
            }
            else if (vb_version == 5) {
                vbat_mv = binUnpackVbat(decryptedBytes[0] & 0x0F);
                lat = binUnpackLatLon32(&decryptedBytes[1]);
                lon = binUnpackLatLon32(&decryptedBytes[5]);
                speed_mps = decryptedBytes[9];
                memcpy(&alt_meters, &decryptedBytes[10], 2);
            }
        } else {
            Serial.println("Can't decrypt, possible wrong key");
        }
    }
    
    result["id1"] = id1;
    result["id2"] = id2;
    result["lat"] = lat;
    result["lon"] = lon;
    result["vbat"] = vbat_mv;
    result["alt"] = alt_meters;
    result["mps"] = speed_mps;
}

// ============================================================================
// BLE Setup
// ============================================================================
void setupBLE() {
    BLEDevice::init("Loko");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            BLECharacteristic::PROPERTY_NOTIFY
                        );
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                                CHARACTERISTIC_UUID_RX,
                                                BLECharacteristic::PROPERTY_WRITE
                                            );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE advertising started");
}

void sendBLE(const String& data) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue(data.c_str());
        pTxCharacteristic->notify();
    }
}

// ============================================================================
// Command Processing
// ============================================================================
void processCommand(const String& cmdLine) {
    // Create a mutable copy
    String cmd_copy = cmdLine;
    cmd_copy.trim();
    if (cmd_copy.length() == 0) return;
    
    // Parse command and arguments
    int spaceIdx = cmd_copy.indexOf(' ');
    String cmd = (spaceIdx == -1) ? cmd_copy : cmd_copy.substring(0, spaceIdx);
    String args = (spaceIdx == -1) ? "" : cmd_copy.substring(spaceIdx + 1);
    
    cmd.toLowerCase();
    
    if (cmd == "help") {
        Serial.println("Available commands:");
        Serial.println("  set gid2 VALUE - Set device ID");
        Serial.println("  set gfreq VALUE - Set frequency in Hz");
        Serial.println("  set gp2p_key VALUE - Set encryption key (64 hex chars)");
        Serial.println("  info - Print current settings");
        Serial.println("  log [N] - Show log entries (optionally last N)");
        Serial.println("  clearlog - Clear all log entries");
        Serial.println("  savelog - Force save logs to flash");
        Serial.println("  mem - Show memory usage");
        Serial.println("  exit - Exit application");
        Serial.println("OK");
    }
    else if (cmd == "set") {
        int spaceIdx2 = args.indexOf(' ');
        if (spaceIdx2 == -1) {
            Serial.println("Error: Expected parameter and value");
            return;
        }
        
        String param = args.substring(0, spaceIdx2);
        String value = args.substring(spaceIdx2 + 1);
        
        if (param == "gid2") {
            settings.id2 = value.toInt();
            saveSettings();
            Serial.println("OK");
        }
        else if (param == "gfreq") {
            uint32_t freq = value.toInt();
            if (freq < 100000000 || freq > 1000000000) {
                Serial.println("Error: Invalid frequency");
                return;
            }
            settings.freq = freq;
            saveSettings();
            Serial.println("OK");
        }
        else if (param == "gp2p_key") {
            if (value.length() != 64) {
                Serial.println("Error: Key must be 64 hex characters");
                return;
            }
            strncpy(settings.p2p_key, value.c_str(), 64);
            settings.p2p_key[64] = '\0';
            saveSettings();
            Serial.println("OK");
        }
        else {
            Serial.println("Error: Unknown parameter");
        }
    }
    else if (cmd == "info") {
        Serial.println("Settings:");
        Serial.printf("  Device ID (id2): %d\n", settings.id2);
        Serial.printf("  Frequency: %d Hz\n", settings.freq);
        Serial.printf("  P2P Key: %s\n", settings.p2p_key);
        Serial.println("OK");
    }
    else if (cmd == "log") {
        int numEntries = (args.length() > 0) ? args.toInt() : logEntries.size();
        if (numEntries > logEntries.size()) numEntries = logEntries.size();
        
        Serial.println("--- Log Entries ---");
        int startIdx = logEntries.size() - numEntries;
        for (int i = startIdx; i < logEntries.size(); i++) {
            Serial.printf("[%s] %s\n", logEntries[i].timestamp.c_str(), 
                         logEntries[i].data.c_str());
        }
        Serial.println("------------------");
        Serial.println("OK");
    }
    else if (cmd == "clearlog") {
        clearLog();
        Serial.println("OK");
    }
    else if (cmd == "savelog") {
        File file = SPIFFS.open(LOG_FILENAME, "w");
        if (file) {
            file.print(exportLogs());
            file.close();
            Serial.println("Logs saved");
        } else {
            Serial.println("Error: Failed to save logs");
        }
        Serial.println("OK");
    }
    else if (cmd == "mem") {
        Serial.println("Memory Usage:");
        Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("  Total heap: %d bytes\n", ESP.getHeapSize());
        Serial.printf("  Used heap: %d bytes\n", ESP.getHeapSize() - ESP.getFreeHeap());
        Serial.printf("  Percent used: %.1f%%\n", 
                     100.0 * (ESP.getHeapSize() - ESP.getFreeHeap()) / ESP.getHeapSize());
        Serial.println("OK");
    }
    else if (cmd == "exit") {
        Serial.println("OK");
        exitRequest = true;
    }
    else {
        Serial.println("Error: Unknown command");
    }
}

// ============================================================================
// Button Handler (ISR)
// ============================================================================
void IRAM_ATTR buttonISR() {
    buttonPressed = true;
    buttonPressTime = millis();
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\nLoko Ground Station - Arduino Version");
    
    // Initialize pins
    //pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    //pinMode(LED_GREEN_PIN, OUTPUT);
   // pinMode(POWER_CTRL_PIN, OUTPUT);
   // pinMode(BUTTON_PIN, INPUT);
    
    // LED startup sequence
    //digitalWrite(LED_BLUE_PIN, LOW);
    delay(500);
    //digitalWrite(LED_BLUE_PIN, HIGH);
    digitalWrite(LED_RED_PIN, LOW);
    delay(500);
    digitalWrite(LED_RED_PIN, HIGH);
    //digitalWrite(LED_GREEN_PIN, LOW);
    delay(500);
   // digitalWrite(LED_GREEN_PIN, HIGH);
    
    // Power control
    //digitalWrite(POWER_CTRL_PIN, HIGH);
   // digitalWrite(LED_GREEN_PIN, LOW);
    delay(500);
    //digitalWrite(LED_GREEN_PIN, HIGH);
    
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
    }
    
    // Load settings and logs
    loadSettings();
    loadLogs();
    
    // Initialize LoRa UART
    LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    LoRaSerial.setRxBufferSize(1024);
    
    // Setup BLE
    setupBLE();
    
    // Configure LoRa
    loraSet(settings.freq);
    loraDataReceive();
    
    // Setup button interrupt
   // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    
    // Check battery
    float vbat = 4.1;;
    Serial.printf("Battery voltage: %.2f V\n", vbat);
    
    Serial.println("Setup complete. Ready.");
    if (USE_COMMAND_LINE_PARSER) {
        Serial.println("Type 'help' for commands");
        Serial.print("> ");
    }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // Handle button press
//    if (buttonPressed) {
//        buttonPressed = false;
//        if (digitalRead(BUTTON_PIN) == LOW) {
//            delay(2000);
//            if (digitalRead(BUTTON_PIN) == LOW) {
//                digitalWrite(LED_RED_PIN, !digitalRead(LED_GREEN_PIN));
//                digitalWrite(POWER_CTRL_PIN, !digitalRead(POWER_CTRL_PIN));
//                Serial.printf("Power value: %d\n", digitalRead(POWER_CTRL_PIN));
//            } else {
//                Serial.println("Button released");
//            }
//        }
//    }
    
    // Check battery level
    float vbat = 4.1;
    if (vbat < 3.3) {
        Serial.println("Battery too low. Entering deep sleep.");
        delay(100);
       // digitalWrite(POWER_CTRL_PIN, LOW);
       // esp_deep_sleep_start();
    }
    
    // Send battery level via BLE periodically
    if (btCounter < 301 && deviceConnected) {
        float batteryPercent = (vbat - 3.3) * 100.0 / 0.9;
        String batteryStr = String(batteryPercent, 2);
        sendBLE(batteryStr);
    }
    btCounter++;
    if (btCounter > 300) {
        btCounter = 0;
    }
    
    // Handle BLE connection state changes
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        oldDeviceConnected = deviceConnected;
        
        // Blink LED when disconnected
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 300) {
           // digitalWrite(LED_BLUE_PIN, !digitalRead(LED_BLUE_PIN));
            lastBlink = millis();
        }
    }
    
    // Process LoRa data
    if (LoRaSerial.available()) {
        String loraData = LoRaSerial.readString();
        Serial.print("LoraRx: ");
        Serial.println(loraData);
        
        String lokoPayload = parseLoraModuleMessage(loraData);
        if (lokoPayload.length() > 0) {
            StaticJsonDocument<512> lokoData;
            String lokoString = "";
            
            // Get encryption key
            uint8_t key[32];
            hexStringToBytes(settings.p2p_key, key, 32);
            
            if (isHexAsciiConvertible(lokoPayload)) {
                lokoString = hexToAscii(lokoPayload);
                Serial.print("LokoMessage: ");
                Serial.println(lokoString);
                parseLokoStringPacket(lokoString, key, lokoData);
            } else {
                parseLokoBinPacket(lokoPayload, key, lokoData);
                
                // Reconstruct string representation
                char buf[256];
                snprintf(buf, sizeof(buf), "%d,%d,%.6f,%.6f,%d,%d,%d",
                        (int)lokoData["id1"], (int)lokoData["id2"],
                        (float)lokoData["lat"], (float)lokoData["lon"],
                        (int)lokoData["vbat"], (int)lokoData["alt"], 
                        (int)lokoData["mps"]);
                lokoString = String(buf);
                Serial.print("LokoMessage: ");
                Serial.println(lokoString);
            }
            
            if (!lokoData.isNull() && lokoData.containsKey("id1")) {
                // Create log entry
                if (USE_COMMAND_LINE_PARSER) {
                    char logBuf[256];
                    snprintf(logBuf, sizeof(logBuf), 
                            "ID1=%d, ID2=%d, LAT=%.6f, LON=%.6f, VBAT=%d",
                            (int)lokoData["id1"], (int)lokoData["id2"],
                            (float)lokoData["lat"], (float)lokoData["lon"],
                            (int)lokoData["vbat"]);
                    
                    if (lokoData.containsKey("alt") && lokoData.containsKey("mps")) {
                        char altMps[64];
                        snprintf(altMps, sizeof(altMps), ", ALT=%d, MPS=%d",
                                (int)lokoData["alt"], (int)lokoData["mps"]);
                        strcat(logBuf, altMps);
                    }
                    
                    addLogEntry(String(logBuf));
                }
                
                // Check ID and send via BLE
                if ((int)lokoData["id2"] == settings.id2) {
                    if (deviceConnected) {
                        sendBLE(lokoString);
                    } else {
                        Serial.println("BLE not connected");
                    }
                } else {
                    Serial.printf("DEBUG: Received unexpected ID2=%d, Expected=%d\n",
                                (int)lokoData["id2"], settings.id2);
                }
            }
        }
    }
    
    // Handle serial commands
    if (USE_COMMAND_LINE_PARSER && Serial.available()) {
        String cmdLine = Serial.readStringUntil('\n');
        processCommand(cmdLine);
        if (!exitRequest) {
            Serial.print("> ");
        }
    }
    
    // Exit if requested
    if (exitRequest) {
        Serial.println("Exiting...");
        delay(1000);
        ESP.restart();
    }
    
    delay(100);
}
