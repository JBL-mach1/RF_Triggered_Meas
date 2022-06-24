/*
   This code uses rf95_reliable_datagram_server from the Radiohead library as the
   framework for recieving/sending data packets using the Semtech SX1272 LoRa radio chip.

   Board: Adafruit Feather 32u4 w/ LoRa Radio Module
   https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/overview

   This project is completed to fulfill the needs of a graduate research project.

   James Lesser
   Nov. 23, 2021
*/
//------------------------------------------------------------------------------
// #include
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <string.h>
//------------------------------------------------------------------------------
// Uncomment to enable Serial commands.
// #define DEBUG
//------------------------------------------------------------------------------
// Radio Configuration

// Define LoRa radio addresses.
#define CONTROLLER_ADDRESS 0  // Destination.
#define ACCEL_NUM 5 // DEVICE SPECIFIC!

// Pins.
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Radio message arrays
volatile uint8_t data[32]; // Array to hold outgoing messages.
volatile uint8_t buf[2];   // Array to hold received messages: "1", "2", or "3".

// Prepare to receive message.
uint8_t len = sizeof(buf);
uint8_t from;

// Singleton instance of the radio driver.
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above.
RHReliableDatagram manager(driver, ACCEL_NUM);

// Define radio frequency, must match.
#define RF95_FREQ 905.0

//------------------------------------------------------------------------------
// Error LED.
const int8_t ERROR_LED_PIN = 13;

// Battery pin.
#define VBATPIN 9

// Define i2c ddresses for communicating with Datalogger.
#define RADIO_I2C_ADDRESS 0x8
#define SD_I2C_ADDRESS 0x9

// Indicate when an i2c message is received.
volatile bool rcvIndicator = false;

// Messages.
uint8_t rcvMes[] = "Received";
uint8_t wireError[] = "i2c Error";
uint8_t rBat[4];

// Timing variables.
uint32_t startMillis;
uint32_t waitToSend = ACCEL_NUM * 125;
uint32_t waitToLog = 5000;
//==============================================================================
/* Should only execute if initialization fails, watch for this when uploading.
   Won't do us much good once the sensors are placed in the snow. */

void fatalBlink() {

  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}
//==============================================================================
/* Keep track of time while waiting for an i2c response. Copy "Wire Error" to data
   and break out of while loop if more than the specified time has elapsed since the call
   to Wire.endTransmission. receiveEvent sets rcvIndicator == true, the while condition
   evaluates false, the program advances and rcvIndicator is reset for the next time
   this function is called. */

void waitForWire(float timeOut) {

  uint32_t startMillis = millis();
  while (rcvIndicator == false) {
    if (millis() - startMillis >= timeOut) {
      // Too much time has passed, copy "Wire Error" to data.
      strcpy(data, wireError);
      break;
    }
  }
  // Reset rcvIndicator
  rcvIndicator = false;
}
//==============================================================================
/* Function that executes whenever data is received from master.
   Both devices are capable of functioning as either master or slave.
   This function is registered as an event, see setup(). */

void receiveEvent(void) {

  int i = 0;
  // Loop through this and read all available data, copy into data[].
  while (0 < Wire.available())  {
    data[i] = Wire.read();
    i++;
    // If the message exceeds 32 bytes break out of while loop.
    if (i == 31) {
      data[i] = '\0';
      break;
    }
  }
  // Set true to break waitForWire while loop.
  rcvIndicator = true;
}
//==============================================================================
/* Function that executes whenever data is ready for radio transmission. */

void sendRadioMessage(void) {
  manager.sendtoWait(data, 32, CONTROLLER_ADDRESS);
}
//==============================================================================
void setup(void) {
  // Reset error LED.
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
    digitalWrite(ERROR_LED_PIN, LOW);
  }

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {}
#endif

  // Radio initialization.
  if (!manager.init()) {
    fatalBlink();
  }

  /* Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
     Sf = 128chips/symbol, CRC on The default transmitter power is 13dBm,
     using PA_BOOST. If you are using RFM95/96/97/98 modules which uses
     the PA_BOOST transmitter pin, then you can set transmitter powers
     from 5 to 23 dBm: */

  // Set radio frequency.
  driver.setFrequency(RF95_FREQ);
  // Set radio transmission power.
  driver.setTxPower(20, false);
  // Channel Activity Detection.
  driver.setCADTimeout(100);
  // Retries for sendtoWait().
  manager.setRetries(10);

  // SDA and SCL must be connected to pull up resistors!
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  // Join i2c bus
  Wire.begin(RADIO_I2C_ADDRESS);

  // Register event (ISR)
  Wire.onReceive(receiveEvent);
}
//==============================================================================
void loop(void) {

  // Receive but do not ack.
  if (manager.recvfrom(buf, &len, &from)) {

    /* Sort message, received messages could be 1,2, or 3. This could be
       replaced with a switch statement to increase program efficiency. */

    //------------------------------------------------------------------------------
    if ((*buf - '0') == 1) {

      // Copy Received Signal Strength Intensity to data[].
      itoa(driver.lastRssi(), data, 10);

      // Wait for your turn...
      startMillis = millis();
      while (millis() - startMillis <= (32*waitToSend)) {}
      
      // Last RSSI value.
      sendRadioMessage();

      //------------------------------------------------------------------------------
    } else if ((*buf - '0') == 2) {

      // Message 1.
      strcpy(data, "1");

      // Wait for your turn...
      startMillis = millis();
      while (millis() - startMillis <= waitToSend) {}

      // "Received".
      sendRadioMessage();

      // Wait for all devices to send "Received", wait a little longer so 
      // the controller can display, then send i2c logging message.
      startMillis = millis();
      while ((millis() - startMillis) <= (waitToLog - waitToSend)) {}

      // Send command 2 over i2c.
      Wire.beginTransmission(SD_I2C_ADDRESS);
      Wire.write("2");
      Wire.endTransmission();

      //------------------------------------------------------------------------------
    } else if ((*buf - '0') == 3) {

      // Send command 3 over i2c.
      Wire.beginTransmission(SD_I2C_ADDRESS);
      Wire.write("3");
      Wire.endTransmission();

      // Wait for i2c message.
      waitForWire(30000);

      // Throw away reading
      analogRead(VBATPIN);

      // Read voltage into an array of uint8_t.
      itoa(analogRead(VBATPIN), rBat, 10);
      // Make sure rBat is null terminated.
      rBat[3] = '\0';

      // Make sure data is null terminated.
      data[3] = '\0';
      // Append our voltage to data[].
      strcat(data, rBat);

      // Message 1.
      sendRadioMessage();
    }

    //------------------------------------------------------------------------------
  } else if (rcvIndicator == true) {
    
    // Startup or command 2 messages.
    sendRadioMessage();
    rcvIndicator = false;
  }
}
//==============================================================================
