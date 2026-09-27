//============================================================================
// TBTracker-esp32 - roel@kroes.com
// 
//  FIRST THING YOU NEED TO DO IS ADJUST THE SETTINGS IN Settings.h
//  
//  Have FUN!
//============================================================================
#include "Settings.h"
#include <RadioLib.h>
#include <SPI.h>
#include "esp32-hal-cpu.h"
#include "horus_l2.h"
#include "HorusBinaryV3.h"
#include <esp_camera.h>
#include <Wire.h>
#include <XPowersLib.h>
#if defined(USE_OLED)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

// Macro for clamping Horus Binary V3 parameters
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
XPowersLibInterface *PMU = nullptr;
#if defined(USE_OLED)
Adafruit_SSD1306 StatusDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool StatusDisplayReady = false;
unsigned long PreviousDisplayUpdate = 0;
#endif


void setupTBeamPower()
{
  Serial.println("Initialising T-Beam AXP2101 PMU...");

  PMU = new XPowersAXP2101(Wire, 21, 22);

  if (!PMU->init()) {
    Serial.println("ERROR: AXP2101 PMU not found!");
    Serial.println("Expected AXP2101 at address 0x34 on SDA=21/SCL=22.");
    delete PMU;
    PMU = nullptr;

    // Stop here, but only after Serial has been started so the failure is visible.
    while (true) {
      delay(1000);
    }
  }

  Serial.println("AXP2101 detected.");

  // T-Beam V1.2 AXP2101: ALDO2 powers the LoRa radio.
  PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
  PMU->enablePowerOutput(XPOWERS_ALDO2);

  // T-Beam V1.2 AXP2101: ALDO3 powers the GPS receiver.
  PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
  PMU->enablePowerOutput(XPOWERS_ALDO3);

  // DC1 supplies the ESP32 on this revision. Do not reconfigure it.
  PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

  delay(500);

  Serial.println("LoRa and GPS power enabled.");
}

//============================================================================
// DATA STRUCTS
// 
// Normally no change necessary
//============================================================================
// Struct to hold GPS data
struct TGPS {
  int Hours, Minutes, Seconds, Day, Month, Year;
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  unsigned int Heading;
  bool validPosition = false;
  bool validTime = false;
  bool validHDOP = false;
  float HDOP = 0;
  float EstimatedAccuracy = 0;
  float Speed;
} UGPS;

#if defined(USE_OLED)
//============================================================================
// Initialize the optional T-Beam OLED. A missing display is non-fatal.
//============================================================================
void setupStatusDisplay()
{
  // Wire was already initialized by XPowersLib. Do not restart the shared bus.
  StatusDisplayReady = StatusDisplay.begin(
    SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false);

  if (!StatusDisplayReady) {
    toSerialConsole("OLED display not found; continuing without it.\n");
    return;
  }

  StatusDisplay.clearDisplay();
  StatusDisplay.setTextColor(SSD1306_WHITE);
  StatusDisplay.setTextSize(1);
  StatusDisplay.setCursor(0, 0);
  StatusDisplay.println(F("TBTRACKER STARTING"));
  StatusDisplay.println();
  StatusDisplay.println(F("PMU: AXP2101 OK"));
  StatusDisplay.println(F("GPS + RADIO: ON"));
  StatusDisplay.display();
  toSerialConsole("OLED status display initiated okay.\n");
}

//============================================================================
// Refresh the status screen at most once per second.
//============================================================================
void updateStatusDisplay(bool forceUpdate = false)
{
  if (!StatusDisplayReady) return;

  unsigned long now = millis();
  if (!forceUpdate && (now - PreviousDisplayUpdate < 1000)) return;
  PreviousDisplayUpdate = now;

  StatusDisplay.clearDisplay();
  StatusDisplay.setTextColor(SSD1306_WHITE);
  StatusDisplay.setTextSize(1);

  StatusDisplay.setCursor(0, 0);
  StatusDisplay.print(F("TBTRACKER  RUNNING"));
  StatusDisplay.drawFastHLine(0, 9, OLED_WIDTH, SSD1306_WHITE);

  StatusDisplay.setCursor(0, 13);
  StatusDisplay.print(F("UTC  "));
  if (UGPS.validTime) {
    if (UGPS.Hours < 10) StatusDisplay.print('0');
    StatusDisplay.print(UGPS.Hours);
    StatusDisplay.print(':');
    if (UGPS.Minutes < 10) StatusDisplay.print('0');
    StatusDisplay.print(UGPS.Minutes);
    StatusDisplay.print(':');
    if (UGPS.Seconds < 10) StatusDisplay.print('0');
    StatusDisplay.print(UGPS.Seconds);
  } else {
    StatusDisplay.print(F("--:--:--"));
  }

  StatusDisplay.setCursor(0, 25);
  StatusDisplay.print(F("GPS FIX: "));
  StatusDisplay.print(UGPS.validPosition ? F("YES") : F("NO"));

  StatusDisplay.setCursor(0, 37);
  StatusDisplay.print(F("SATS "));
  StatusDisplay.print(UGPS.Satellites);
  StatusDisplay.print(F("  HDOP "));
  if (UGPS.validHDOP) {
    StatusDisplay.print(UGPS.HDOP, 1);
  } else {
    StatusDisplay.print(F("--"));
  }

  StatusDisplay.setCursor(0, 49);
  StatusDisplay.print(F("EST ACC: "));
  if (UGPS.validPosition && UGPS.validHDOP) {
    StatusDisplay.print(F("+/-"));
    StatusDisplay.print(UGPS.EstimatedAccuracy, 1);
    StatusDisplay.print(F("m"));
  } else {
    StatusDisplay.print(F("--"));
  }

  StatusDisplay.display();
}
#endif

// Struct to hold LoRA settings
struct TLoRaSettings {
  float Frequency = LORA_FREQUENCY;
  float Bandwidth = LORA_BANDWIDTH;
  uint8_t SpreadFactor = LORA_SPREADFACTOR;
  uint8_t CodeRate = LORA_CODERATE;
  uint8_t SyncWord = LORA_SYNCWORD;
  uint8_t Power = LORA_POWER;
  uint8_t CurrentLimit = LORA_CURRENTLIMIT;
  uint16_t PreambleLength = LORA_PREAMBLELENGTH;
  uint8_t Gain = LORA_GAIN;
  size_t implicitHeader = 255;
} LoRaSettings;

// Struct to hold RTTY settings
struct TRTTYSettings {
  float Frequency = RTTY_FREQUENCY;  // Base frequency
  uint32_t Shift = RTTY_SHIFT;       // RTTY shift
  uint16_t Baud = RTTY_BAUD;         // Baud rate
  uint8_t Encoding = RTTY_ASCII;     // Encoding (ASCII = 7 bits)
  uint8_t StopBits = RTTY_STOPBITS;  // Number of stopbits
} RTTYSettings;

// Horus Binary Packet Structure - Version 1
struct HorusBinaryPacketV1 {
  uint8_t PayloadID;    // Refer to  https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
  uint16_t Counter;     // Packet counter
  uint8_t Hours;        // Hours Zulu
  uint8_t Minutes;      // Minutes
  uint8_t Seconds;      // Seconds
  float Latitude;       // Latitude in format xx.yyyy
  float Longitude;      // Longitude in format xx.yyyy
  uint16_t Altitude;    // Altitude in meters
  uint8_t Speed;        // Speed in kmh
  uint8_t Sats;         // Number of satellites visible
  int8_t Temp;          // Twos Complement Temp value.
  uint8_t BattVoltage;  // 0 = 0.5v, 255 = 5.0V, linear steps in-between.
  uint16_t Checksum;    // CRC16-CCITT Checksum.
} __attribute__((packed));

// Horus v2 Mode 1 (32-byte) Binary Packet
struct HorusBinaryPacketV2 {
  uint16_t PayloadID;   // Refer to  https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
  uint16_t Counter;     // Packet counter
  uint8_t Hours;        // Hours Zulu
  uint8_t Minutes;      // Minutes
  uint8_t Seconds;      // Seconds
  float Latitude;       // Latitude in format xx.yyyy
  float Longitude;      // Longitude in format xx.yyyy
  uint16_t Altitude;    // Altitude in meters
  uint8_t Speed;        // Speed in kmh
  uint8_t Sats;         // Number of satellites visible
  int8_t Temp;          // Twos Complement Temp value.
  uint8_t BattVoltage;  // 0 = 0.5v, 255 = 2.0V, linear steps in-between.
  // The following 9 bytes (up to the CRC) are user-customizable. The following just
  // provides an example of how they could be used.
  // Refer here for details: https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet
  int16_t dummy1;     // Interpreted as Ascent rate divided by 100 for the Payload ID: 4FSKTEST-V2
  int16_t dummy2;     // External temperature divided by 10 for the Payload ID: 4FSKTEST-V2
  uint8_t dummy3;     // External humidity for the Payload ID: 4FSKTEST-V2
  uint16_t dummy4;    // External pressure divided by 10 for the Payload ID:  4FSKTEST-V2
  uint16_t unused;    // 2 bytes which are not interpreted
  uint16_t Checksum;  // CRC16-CCITT Checksum.

} __attribute__((packed));

//============================================================================
// GLOBALS
//  
// Normally no change necessary
//============================================================================
#define HORUS_UNCODED_BUFFER_SIZE 512
#define HORUS_CODED_BUFFER_SIZE 512
uint8_t rawbuffer[HORUS_UNCODED_BUFFER_SIZE];    // Buffer to temporarily store a raw binary packet.
uint8_t codedbuffer[HORUS_CODED_BUFFER_SIZE];  // Buffer to store an encoded binary packet
char debugbuffer[512];     // Buffer to store debug strings
HardwareSerial SerialGPS(1);
char Sentence[SENTENCE_LENGTH];
long RTTYCounter = 1;
long LoRaCounter = 1;
long horusCounterV1 = 1;
long horusCounterV2 = 1;
int horusCounterV3 = 1;
unsigned long previousTX_LoRa = 0;
unsigned long previousTX_RTTY = 0;
unsigned long previousTX_HorusV1 = 0;
unsigned long previousTX_HorusV2 = 0;
unsigned long previousTX_HorusV3 = 0;
unsigned long previousTX_LoRa_APRS = 0;
unsigned long previousTX_APRS_AFSK = 0;
unsigned long previousTX_SSDVHIGHRES = 0;
unsigned long previousTX_SSDVLOWRES = 0;
volatile bool receivedFlag = false;

//============================================================================
// Generate a Horus Binary v1 packet, and populate it with data.
//============================================================================
int build_horus_binary_packet_v1(uint8_t *buffer) {
  struct HorusBinaryPacketV1 BinaryPacket;

  BinaryPacket.PayloadID = PAYLOAD_ID_V1;
  BinaryPacket.Counter = horusCounterV1++;
  BinaryPacket.Hours = UGPS.Hours;
  BinaryPacket.Minutes = UGPS.Minutes;
  BinaryPacket.Seconds = UGPS.Seconds;
  BinaryPacket.Latitude = UGPS.Latitude;
  BinaryPacket.Longitude = UGPS.Longitude;
  BinaryPacket.Altitude = UGPS.Altitude;
  BinaryPacket.Speed = 0;
  BinaryPacket.BattVoltage = round((ReadVCC() / 5.0) * 256.0);
  BinaryPacket.Sats = UGPS.Satellites;
  BinaryPacket.Temp = (int8_t)round(ReadTemp());
  BinaryPacket.Checksum = (uint16_t)crc16((unsigned char *)&BinaryPacket, sizeof(BinaryPacket) - 2);

  memcpy(buffer, &BinaryPacket, sizeof(BinaryPacket));
  return sizeof(struct HorusBinaryPacketV1);
}

//============================================================================
// Generate a Horus Binary v2 packet, and populate it with data.
//============================================================================
int build_horus_binary_packet_v2(uint8_t *buffer) {
  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = PAYLOAD_ID_V2;
  BinaryPacketV2.Counter = horusCounterV2++;
  BinaryPacketV2.Hours = UGPS.Hours;
  BinaryPacketV2.Minutes = UGPS.Minutes;
  BinaryPacketV2.Seconds = UGPS.Seconds;
  BinaryPacketV2.Latitude = UGPS.Latitude;
  BinaryPacketV2.Longitude = UGPS.Longitude;
  BinaryPacketV2.Altitude = UGPS.Altitude;
  BinaryPacketV2.Speed = 0;
  BinaryPacketV2.BattVoltage = round((ReadVCC() / 5.0) * 256.0);
  BinaryPacketV2.Sats = UGPS.Satellites;
  BinaryPacketV2.Temp = (int8_t)round(ReadTemp());
  // Custom section. This is an example only, and the 9 bytes in this section can be used in other
  // ways. Refer here for details: https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet
  BinaryPacketV2.dummy1 = 0;                           // int16_t Interpreted as Ascent rate divided by 100 for 4FSKTEST-V2. This value would display as 2.0 on HabHub
  BinaryPacketV2.dummy2 = round(ReadTemp()) * 10;      // int16_t External temperature divided by 10 for 4FSKTEST-V2. This value would display as -2.0 on HabHub
  BinaryPacketV2.dummy3 = round(ReadHumidity());       // uint8_t External humidity for 4FSKTEST-V2. This value would display as 51 on HabHub
  BinaryPacketV2.dummy4 = round(ReadPressure()) * 10;  // uint16_t External pressure divided by 10 for 4FSKTEST-V2. This value would display as 2.1 on HabHub
  BinaryPacketV2.unused = 0;                           // Two unused filler bytes
  BinaryPacketV2.Checksum = (uint16_t)crc16((unsigned char *)&BinaryPacketV2, sizeof(BinaryPacketV2) - 2);

  memcpy(buffer, &BinaryPacketV2, sizeof(BinaryPacketV2));
  return sizeof(struct HorusBinaryPacketV2);
}

//============================================================================
// Generate a Horus Binary v3 packet, and populate it with data.
//============================================================================
int build_horus_binary_packet_v3(uint8_t * uncoded_buffer)
{
  // Horus v3 packets are encoded using ASN.1, and are encapsulated in packets
  // of sizes 32, 48, 64, 96 or 128 bytes (before coding)
  // The CRC16 for these packets is located at the *start* of the packet, still little-endian encoded

  // Erase the uncoded buffer
  // This has the effect of padding out the unused bytes in the packet with zeros
  memset(uncoded_buffer, 0, HORUS_UNCODED_BUFFER_SIZE);

  horusTelemetry asnMessage = 
  {
    .payloadCallsign  = HORUS_V3_CALLSIGN,
    .sequenceNumber = horusCounterV3++,
    .timeOfDaySeconds  = CLAMP(UGPS.Hours*3600 + UGPS.Minutes*60 + UGPS.Seconds,-1,86400),
    .latitude = CLAMP((int)(UGPS.Latitude*100000),-9000000, 9000000),
    .longitude = CLAMP((int)(UGPS.Longitude*100000),-18000000, 18000000),
    .altitudeMeters = CLAMP(UGPS.Altitude,-1000, 50000),
    // Example of a custom parameter
    .extraSensors = {
          .nCount=1, // Number of custom fields.
          .arr = {
            // Example of an array of integers 
             {
                 .name = "rf",
                 .values = {
                     .kind = horusStr_PRESENT,
                     .u = {
#if defined(USE_SX127X)
                         .horusStr = "sx127x"
#endif
#if defined(USE_LLCC68)
                         .horusStr = "llcc68"
#endif
#if defined(USE_SX1268)
                         .horusStr = "sx1268"
#endif
#if defined(USE_SX1262)
                         .horusStr = "sx1262"
#endif
#if defined(USE_RF69)
                         .horusStr = "rf69"
#endif


                     }
                 },
                  .exist = {
                     .name = true,
                     .values = true,
                 },
             },
          },
        },



    .velocityHorizontalKilometersPerHour = CLAMP(UGPS.Speed,0, 511),
    .gnssSatellitesVisible = CLAMP(UGPS.Satellites,0, 31),
    // .ascentRateCentimetersPerSecond = vVCalc * 100, // m/s -> cm/s
    .pressurehPa_x10 = CLAMP((uint16_t)round(ReadPressure()) * 10,0,12000),
    .temperatureCelsius_x10 = 
    {
        .internal = CLAMP((int16_t)(round(ReadTemp()) * 10),-1023, 1023),
        .external = CLAMP((int16_t)(round(ReadTemp()) * 10),-1023, 1023),
        .exist = 
        {
             .internal = false,
             .external = true,
             .custom1 = false,
             .custom2 = false
        }
    },
    .humidityPercentage = CLAMP((uint8_t)round(ReadHumidity()),0,100),
    .milliVolts = 
    {
      .battery =  CLAMP(ReadVCC()*1000,0,16383),   
      .exist = 
       {
         .battery = true,
         .solar = false,
         .custom1 = false,
         .custom2 = false
       }
     },



     // We need to explicitly specify which optional fields we want to include in the packet
     .exist = 
     {
#if defined(HORUS_V3_CUSTOM_FIELDS)      
       .extraSensors = true,
#else       
       .extraSensors = false,
#endif
       .velocityHorizontalKilometersPerHour = true,
       .gnssSatellitesVisible = true,
       .ascentRateCentimetersPerSecond = false,
       .pressurehPa_x10 = true,
       .temperatureCelsius_x10 = true,
       .humidityPercentage = true,
       .milliVolts = true
      }
    };
    // It is possible to conditionally disable some of the fields if we have no valid data source for them

    // Example: Don't send pressure data if it's just 0. This is either a failed sensor or no data
    // if (pressureValue == 0){
    //   asnMessage.exist.pressurehPa_x10 = false;
    // }
        // The encoder needs a data structure for the serialization
    // Again - how much memory is allocated here?
    BitStream encodedMessage;

    // The Encoder may fail and update an error code
    int errCode;

    // Initialization associates the buffer to the bit stream
    // We want to write the uncoded message starting at 2 bytes into the message.
    BitStream_Init (&encodedMessage,
                    (unsigned char*)(uncoded_buffer+2),
                    HORUS_UNCODED_BUFFER_SIZE-1);

    // Encode the message using uPER encoding rule

    // We patch in assert functionality in assert_override.h
    // Before running encode we set assert_value = 0
    // Then check the value in assert_value
    assert_value = 0;

    if (!horusTelemetry_Encode(&asnMessage,
                        &encodedMessage,
                        &errCode,
                        true) || assert_value != 0)
    {  
        // Not at this error helps that much in a flight, but it helps
        // us when debugging!   
        if(errCode > 0)
        {
            toSerialConsole("[error]: HORUS v3 Encoding Failed: ");
            toSerialConsole(errCode);toSerialConsole("\n");
        }
        if(assert_value != 0)
        {
          toSerialConsole("[error]: HORUS v3 Assert Failure, maybe hit buffer size limit");toSerialConsole("\n");
        }
        // Need to check what happens here.
        return 0;
    }
    else 
    {
        // Encoding was successful!
        // Now we need to figure out the required frame size, and add the CRC.
        int encodedSize = BitStream_GetLength(&encodedMessage);

        // Determine the required frame size.
        // Probably should do this from a list of valid sizes in a neater manner
        int frameSize = 128;
        if (encodedSize <= 30){
          frameSize = 32;
        } else if (encodedSize <= 46){
          frameSize = 48;
        } else if (encodedSize <= 62){
          frameSize = 64;
        } else if (encodedSize <= 94){
          frameSize = 96;
        } else if (encodedSize <= 126){
          frameSize = 128;
        }

        // Calculate CRC16 over the frame, starting at byte 2
        uint16_t packetCrc = (uint16_t)crc16((unsigned char *)(uncoded_buffer + 2),
                                     frameSize - 2);
        // Write CRC into bytes 0–1 of the packet
        memcpy(uncoded_buffer, &packetCrc, sizeof(packetCrc));  // little‑endian on STM32

        
        toSerialConsole("[info]: HORUS v3 ASN1: ");
        toSerialConsole(encodedSize);
        toSerialConsole(" Frame: ");
        toSerialConsole(frameSize);
        toSerialConsole("\n");
        
        return frameSize;
    }
    return 0;
}

//============================================================================
// Do the setup of the program
//============================================================================
void setup() {
  // Set CPU speed to 40MHz to spare energy
  // setCpuFrequencyMhz(40);

  // Setup Serial for debugging
#if defined(ALLOWDEBUG)
  Serial.begin(115200);
  unsigned long Serialstart = millis();
  // The timeout loop
  while (!Serial && (millis() - Serialstart < 500)) 
  {
    delay(10);
  }
  if (Serial) 
  {
     toSerialConsole("Serial console initiated okay.\n");
  }
#endif

  // The PMU diagnostic must run after Serial.begin(). Otherwise a failed PMU
  // probe enters its stop loop before any useful message can be displayed.
  setupTBeamPower();

// Disable the Bluetooth stack if it is on the board
#if defined(CONFIG_BT_ENABLED)
  btStop();
#else
    toSerialConsole("No Bluetooth radio detected.\n");
#endif

  // Write the version information
  write_version_info();

#if defined(USE_OLED)
  setupStatusDisplay();
#endif
  
  // Start the SPI interface
  SPI.begin(SCK, MISO, MOSI, CS);

  // Setup the GPS
  // HardwareSerial arguments are RX pin first, then TX pin.
  SerialGPS.begin(GPSBaud, SERIAL_8N1, Rx, Tx);
  
  // Test communication with the GPS for 2 seconds
  GPSCommTest(2000);

  // Setup the Radio
  ResetRadio();
  
#if defined(USE_BME280)
  // Initialize the BME280 sensor if available
  setup_bme280();
#endif

#if defined(USE_SSDV)
  // Initialize the cameraand SD card
  setupSSDV();
#endif

  // If defined in settings.h, the software will go into calibration mode
#if defined(CALIBRATE_RADIO)
  FreqCalibration(CAL_FREQUENCY + CAL_OFFSET_FREQUENCY);
#endif

  // Set the Tracker in receiving mode
  // Can only be done if LoRa is enabled
  if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
}

//============================================================================
// Print any sensordata to the Serial console
//============================================================================
void printSensorData() 
{
  toSerialConsole("============\n");
  printGPSData();
  printbme280Data();
  printVoltageInfo();
  toSerialConsole("============\n\n");
}

//============================================================================
// Program loop
//============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Get data from the GPS
  smartDelay(1000);
  CheckGPS();
#if defined(USE_OLED)
  updateStatusDisplay();
#endif
  printSensorData();

  // Process any received LoRa packets
  if (LORA_ENABLED && RECEIVING_ENABLED && receivedFlag) {
    ProcessRXPacket();
  }

  // SSDV enabled
  #if defined(USE_SSDV)

  // Take a lowRes picture and send it over LoRa
  if ((SSDV_LOWRES) && (currentMillis - previousTX_SSDVLOWRES >= ((unsigned long)SSDVLOWRES_LOOPTIME * (unsigned long)1000))) {
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    TakeandSendLowResPhoto(); 
    previousTX_SSDVLOWRES = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Take HighRes picture and save to SD
  if ((SSDV_HIGHRES) && (currentMillis - previousTX_SSDVHIGHRES >= ((unsigned long)SSDVHIGHRES_LOOPTIME * (unsigned long)1000))) {
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    SaveHighResPhoto();  
    previousTX_SSDVHIGHRES = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }
  #endif // SSDV
  
  // Send RTTY
  if ((RTTY_ENABLED) && (currentMillis - previousTX_RTTY >= ((unsigned long)RTTY_LOOPTIME * (unsigned long)1000))) {
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    for (int i = 1; i <= RTTY_REPEATS; i++) {
      CreateTXLine(RTTY_PAYLOAD_ID, RTTYCounter++, RTTY_PREFIX);
      sendRTTY(Sentence);
    }
    previousTX_RTTY = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Send LoRa
  if ((LORA_ENABLED) && (currentMillis - previousTX_LoRa >= ((unsigned long)LORA_LOOPTIME * (unsigned long)1000))) {
    delay(1000);
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    for (int i = 1; i <= LORA_REPEATS; i++) {
      CreateTXLine(LORA_PAYLOAD_ID, LoRaCounter++, LORA_PREFIX);
      sendLoRa(Sentence, LORA_MODE);
    }
    previousTX_LoRa = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Send HORUS V1
  if ((HORUS_V1_ENABLED) && (currentMillis - previousTX_HorusV1 >= ((unsigned long)HORUS_LOOPTIME * (unsigned long)1000))) {
    delay(1000);
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    sendHorusV1();
    previousTX_HorusV1 = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Send HORUS V2
  if ((HORUS_V2_ENABLED) && (currentMillis - previousTX_HorusV2 >= ((unsigned long)HORUS_LOOPTIME * (unsigned long)1000))) {
    delay(1000);
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    sendHorusV2();
    previousTX_HorusV2 = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Send HORUS V3
  if ((HORUS_V3_ENABLED) && (currentMillis - previousTX_HorusV3 >= ((unsigned long)HORUS_LOOPTIME * (unsigned long)1000))) {
    delay(1000);
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    sendHorusV3();
    previousTX_HorusV3 = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Send LORA-APRS
  if ((LORA_APRS_ENABLED) && (currentMillis - previousTX_LoRa_APRS >= ((unsigned long)LORA_APRS_LOOPTIME * (unsigned long)1000))) {
    delay(1000);
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    sendLoRaAprs();
    previousTX_LoRa_APRS = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }

  // Send AFSK-APRS
  if ((APRS_AFSK_ENABLED) && (currentMillis - previousTX_APRS_AFSK >= ((unsigned long)APRS_AFSK_LOOPTIME * (unsigned long)1000))) {
    delay(1000);
    if (LORA_ENABLED && RECEIVING_ENABLED) { unsetFlag(); }
    SendAPRS();
    previousTX_APRS_AFSK = millis();
    // Set the Tracker in receiving mode
    if (LORA_ENABLED && RECEIVING_ENABLED) { StartReceiveLoRaPacket(); }
  }
}
