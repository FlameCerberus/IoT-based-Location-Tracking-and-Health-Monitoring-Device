
#include <TinyGPS++.h>
#include "ThingSpeak.h"
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <ESP_LM35.h>
#include <SIM800L.h>
#include "SoftwareSerial.h"

#define SIM800_RX_PIN 1
#define SIM800_TX_PIN 3
#define SIM800_RST_PIN 5

Sim800l *sim800l;
#define LED 2
float latitude;
float longitude;
float temps[14];
float sumTemp;
float avgTemp;

#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);
TinyGPSPlus gps;

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; // infrared LED sensor data
uint16_t redBuffer[100]; // red LED sensor data
#else
uint32_t irBuffer[100]; // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
#endif

int32_t bufferLength;    // data length
int32_t spo2;            // SPO2 value
int8_t validSPO2;        // indicator to show if the SPO2 calculation is valid
int32_t heartRate;       // heart rate value
int8_t validHeartRate;   // indicator to show if the heart rate calculation is valid

byte pulseLED = 11;  // Must be on PWM pin
byte readLED = 13;   // Blinks with each data read

ESP_LM35 temp(36);

void setupGSM()
{
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial)
    ; // wait for the serial port to connect

  // Initialize the SIM800L module
  sim800l->setPowerMode(NORMAL);
}

void ThingSpeakUpload()
{
  sim800l->setupGPRS("apn", "user", "password");
  // Connect to the network
  if (sim800l->connectGPRS())
  {
    Serial.println("Connected to the network!");
    // Send data to ThingSpeak via GSM Network
    if (sim800l->doPost("api.thingspeak.com", NULL, "api_key=OEZAC2INZJ8JLDT0&field1=latitude&field2=longitude&field3=avgTemp&field4=heartRate&field5=spo2", "Content-Type: applications /wx-ww-urlencoded", NULL, NULL))
    {

      Serial.println("Data sent successfully!");
    }
    else
    {
      Serial.println("Error sending data");
    }
  }
  else
  {
    Serial.println("Error connecting to the network");
  }

  delay(60000); // delay for 1 minute
}

void print_speedGPS()
{
  if (gps.location.isValid() == 1)
  {
    Serial.print("Latitude : ");
    Serial.println(gps.location.lat(), 8);
    Serial.print("Longitude : ");
    Serial.println(gps.location.lng(), 8);
    Serial.print("Satellites : ");
    Serial.println(gps.satellites.value());
  }
  else
  {
    Serial.print("No Data");
  }
}

void loopGPS()
{
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  // If newData is true
  if (newData == true)
  {
    newData = false;
    print_speedGPS();
    Serial.println(gps.satellites.value());
    longitude = gps.location.lng();
    latitude = gps.location.lat();
  }
  else
    Serial.print("No Data\n");
}

void loopTempSensor()
{
  sumTemp = 0;
  for (unsigned long start = millis(); millis() - start < 15000;)
  {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    sumTemp += temp.tempC();
    Serial.print("Temp: ");
    Serial.print(temp.tempC());
    Serial.println("");
    Serial.print("Sum Temp: ");
    Serial.print(sumTemp);
    delay(900);
  }
  avgTemp = sumTemp / 15;
  Serial.print("Avg temp: ");
  Serial.print(avgTemp);
}

void setupPulse()
{
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1)
      ;
  }
  byte ledBrightness = 30;    // Options: 0=Off to 255=50mA
  byte sampleAverage = 1;     // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;           // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 3200;     // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;       // Options: 69, 118, 215, 411
  int adcRange = 4096;        // Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
}

void loopPulse()
{
  bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps
  // read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check();                  // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to the next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // calculate heart rate and SpO2 after the first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  for (unsigned long start = millis(); millis() - start < 15000;)
  {
    // Dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // Take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) // Do we have new data?
        particleSensor.check();                    // Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); // Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // We're finished with this sample so move to the next sample

      // Send samples and calculation result to the terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    // After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void setup()
{
  Serial.begin(115200);
  setupGSM();
  pinMode(LED, OUTPUT); // Setup temp sensor
  setupPulse();
}

void loop()
{
  loopGPS();
  loopTempSensor();
  loopPulse();
  ThingSpeakUpload();
}
