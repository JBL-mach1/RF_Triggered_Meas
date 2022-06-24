/*
  Controller
  Modified: rf95_reliable_datagram_client.ino from RadioHead Lib
  OLED contribution: OLED_featherwing.ino from SSD1306 Lib

  Written to fulfill needs of Graduate Research
  James Lesser
  March 1, 2021
*/
//------------------------------------------------------------------------------
// Include
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <string.h>
//------------------------------------------------------------------------------
// Uncomment to enable Serial commands.
// #define DEBUG

// Error LED.
const int8_t ERROR_LED_PIN = 13;

// Timekeeper.
unsigned long timeOut;
unsigned long varTime = 3000;

// Task 2 variables.
byte logArray[15] = {0};
bool taskTwo = false;
bool devVar = false;
bool rcvMessVar = false;
bool dispOnce = false;
bool logMessVar = false;
bool logSuccVar = false;
bool csvSuccVar = false;

// Task 3 variables.
int intVAR;
float floatVAR;

//------------------------------------------------------------------------------
// OLED Configuration

Adafruit_SSD1306 display(128, 32, &Wire, 13);
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

//------------------------------------------------------------------------------
// Radio Configuration

// Character arrays for sending and receiving messages.
uint8_t data[2];   // Array to hold outgoing messages.
uint8_t buf[32];   // Array to record received messages.

// Variables.
uint8_t len = sizeof(buf);
uint8_t from;

// Used to set SPI registers for each device (Radio & OLED).
#define SPI_HAS_TRANSACTION

// Feather32u4 RFM9x LoRa pin definitions.
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Define address
#define CLIENT_ADDRESS 0 // This address

// Define frequency, must match RX's frequency!
#define RF95_FREQ 905.0

// Singleton instance of the radio driver.
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt.
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

//==============================================================================
// Halt normal operation, blink LED on pin 13.
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
// Display which devices have received command 2.
void rcvMess() {

  // Devices received command.
  for (int i = 0; i <= 7; i++) {
    if (logArray[i] != 0) {
      display.print("A");
      display.print(logArray[i]);
      display.print(" ");
    }
  }
  display.println();
  display.print(F("received."));
  display.display();

  // Clear
  display.setCursor(0, 0);
  display.clearDisplay();

  // Next function
  rcvMessVar = false;
  logMessVar = true;
}
//==============================================================================
// Display which devices have/have not started logging & display countdown.
void logMess() {

  // Devices ready to log.
  for (int i = 0; i <= 7; i++) {
    if (logArray[i] != 0) {
      display.print("A");
      display.print(logArray[i]);
      display.print(" ");
    }
  }
  display.println();
  display.print(F("ready to log."));
  display.display();

  // Display for 5s
  timeOut = millis();
  while ((millis() - timeOut) <= 5000) {}

  // Countdown.
  for (int i = 10; i > 0; i--) {
    dispOnce = false;
    timeOut = millis();
    while ((millis() - timeOut) <= 1000) {
      if (dispOnce == false) {
        display.setCursor(0, 0);
        display.clearDisplay();
        display.print(F("Logging in: "));
        display.print(i);
        display.display();
        dispOnce = true;
      }
    }
  }
  display.setCursor(0, 0);
  display.clearDisplay();
  display.println("Logging.");
  display.display();

  // Clear
  display.setCursor(0, 0);
  display.clearDisplay();

  // Next function
  logMessVar = false;
  logSuccVar = true;
}
//==============================================================================
// Display which devices have/have not saved binary data.
void logSucc() {

  // Which devices were successful.
  for (int i = 0; i <= 7; i++) {
    if (logArray[i] != 0 && logArray[i + 7] == '1') {
      display.print("A");
      display.print(logArray[i]);
      devVar = true;
      display.print(" ");
    }
  }
  if (devVar == true) {
    devVar = false;
    display.println();
    display.print(F("logged successfully!"));
    display.println(); display.println();
    display.println("Saving to CSV...");
    display.display();
  }

  // Display for 7s
  timeOut = millis();
  while ((millis() - timeOut) <= 7000) {}
  display.setCursor(0, 0);
  display.clearDisplay();

  // Which were not.
  for (int i = 0; i <= 7; i++) {
    if (logArray[i] != 0 && logArray[i + 7] != '1') {
      display.print("A");
      display.print(logArray[i]);
      devVar = true;
      display.print(" ");
    }
  }
  if (devVar == true) {
    devVar = false;
    display.println();
    display.print(F("failed to log."));
    display.display();
  }

  // Clear
  display.setCursor(0, 0);
  display.clearDisplay();

  // Next fun.
  varTime = 15000;
  logSuccVar = false;
  csvSuccVar = true;
}
//==============================================================================
// Display which devices have/have not saved CSV data.
void csvSucc() {

  // Which devices were successful.
  for (int i = 0; i <= 7; i++) {
    if (logArray[i] != 0 && logArray[i + 7] == '1') {
      display.print("A");
      display.print(logArray[i]);
      devVar = true;
      display.print(" ");
    }
  }
  if (devVar == true) {
    devVar = false;
    display.println();
    display.println(F("saved CSV data."));
    display.display();
  }

  // Display for 7s
  timeOut = millis();
  while ((millis() - timeOut) <= 7000) {}
  display.setCursor(0, 0);
  display.clearDisplay();

  // Which were not.
  for (int i = 0; i <= 7; i++) {
    if (logArray[i] != 0 && logArray[i + 7] != '1') {
      display.print("A");
      display.print(logArray[i]);
      devVar = true;
      display.print(" ");
    }
  }
  if (devVar == true) {
    devVar = false;
    display.println();
    display.println(F("failed to save CSV."));
  }

    display.println();
    display.println("End of sequence.");
    display.display();

  // End of task 2 sequence.
  display.setCursor(0, 0);
  display.clearDisplay();
  csvSuccVar = false;
}
//==============================================================================
void setup() {

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {}
#endif

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
#ifdef DEBUG
    Serial.println(F("SSD1306 allocation failed"));
#endif
    fatalBlink();
  }

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(5000);

  // When the button is pressed, BUTTON_A is LOW.
  // When the button is not pressed, BUTTON_A is HIGH.
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // Text display
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Clear
  display.setCursor(0, 0);
  display.clearDisplay();
  display.display();

  // Radio Initialization
  if (!manager.init()) {
    display.println(F("Radio init. failed"));
    display.display();
    delay(5000);
  }

  // Clear
  display.setCursor(0, 0);
  display.clearDisplay();
  display.display();

  // Set frequency
  driver.setFrequency(RF95_FREQ);
  // Set power
  driver.setTxPower(20, false);
  // Mitigate message collision
  driver.setCADTimeout(500);
  // Retries for sendtoWait()
  manager.setRetries(10);

  // Opener
  display.println(F("Wireless Data Logger"));
  display.println(F("User Enters A,B, or C"));
  display.display();
  delay(5000);
}
//==============================================================================
void loop() {

  // Clear
  display.setCursor(0, 0);
  display.clearDisplay();
  display.display();

  // Command options
  display.println(F("A: RSSI"));
  display.println(F("B: Log Data"));
  display.println(F("C: Battery Check"));
  display.display();

  // Need this so that holding down a button to break the lower loop
  // does not read as the next command.
  delay(1000);

  // Wait for a button push or a radio message.
  while (true) {
    // If button is pushed break while loop.
    if (!digitalRead(BUTTON_A) || !digitalRead(BUTTON_B) || !digitalRead(BUTTON_C)) {
      break;
    }

    else if (manager.available()) {
      // Clear.
      display.setCursor(0, 0);
      display.clearDisplay();

      // Display startup messages.
      if (manager.recvfromAck(buf, &len, &from)) {
        display.print(F("Accel ")); display.print(from); display.println(F(": "));
        display.println((char*) buf);
        display.display();

        // Current time.
        timeOut = millis();
      }
    }

    // If 5 seconds elapses after last message re-display command options.
    else if ((millis() - timeOut) >= 10000) {
      // Clear.
      display.setCursor(0, 0);
      display.clearDisplay();

      // Command options.
      display.println(F("A: RSSI"));
      display.println(F("B: Log Data"));
      display.println(F("C: Battery Check"));
      display.display();
    }
  }

  // Clear.
  display.setCursor(0, 0);
  display.clearDisplay();
  display.display();

  // Display the command choice.
  if (!digitalRead(BUTTON_A)) {
    display.println(F("Sending Command: 'A'"));
    *data = '1';
  }
  if (!digitalRead(BUTTON_B)) {
    display.println(F("Sending Command: 'B'"));
    *data = '2';
  }
  if (!digitalRead(BUTTON_C)) {
    display.println(F("Sending Command: 'C'"));
    *data = '3';
  }
  display.display();
  delay(3000);

  // Clear.
  display.setCursor(0, 0);
  display.clearDisplay();
  display.display();

  // Send message to all nodes, do not wait for an ack.
  // Evaluates false unless driver fails to send packet.
  if (!manager.sendto(data, sizeof(data), RH_BROADCAST_ADDRESS)) {
    display.println(F("Send Failed"));
    display.display();
    delay(5000);
    return;
  }

  // Take a look at the time.
  timeOut = millis();

  // Wait for a response message.
  display.println(F("Wait for a response"));
  display.display();

  // Wait until we have a message.
  while (!manager.available()) {
    if (millis() - timeOut >= 30000) {
      // Clear.
      display.setCursor(0, 0);
      display.clearDisplay();
      display.display();
      display.println(F("No Response, are"));
      display.println(F("accelerometers on?"));
      display.display();
      delay(5000);
      return;
    }
  }

  // Reset task two variables
  rcvMessVar = true;
  taskTwo = false;
  varTime = 3000;

  //------------------------------------------------------------------------------
  /* Keep listening, pressing any button returns to the top of the loop.
     Pressing a button before all message have been received will result in
     inaccurate messages going forward. The radio driver will receive messages
     and store them in the buffer. */

  while (true) {
    // Clear.
    display.setCursor(0, 0);
    display.clearDisplay();

    // Button pressed?
    if (!digitalRead(BUTTON_A) || !digitalRead(BUTTON_B) || !digitalRead(BUTTON_C)) {
      logMessVar = false; logSuccVar = false; csvSuccVar = false;
      return;
    }
    //------------------------------------------------------------------------------

    // If three seconds has elapsed, we should have all the messages.
    else if (taskTwo == true && (millis() - timeOut) >= varTime) {

      // Set rcvMessVar false & logMessVar true in rcvMessVar function.
      if (rcvMessVar == true) {
        rcvMess();
        memset(logArray + 7, 0, 7);
      } else if (logMessVar == true) {
        logMess();
        memset(logArray + 7, 0, 7);
      } else if (logSuccVar == true) {
        logSucc();
        memset(logArray + 7, 0, 7);
      } else if (csvSuccVar == true) {
        csvSucc();
        memset(logArray, 0, 15);
      }
      taskTwo = false;
    }
    //------------------------------------------------------------------------------

    // Message received and acknowledged?
    // Evaluates true if message was received by driver & ack was returned to sender.
    else if (manager.recvfromAck(buf, &len, &from)) {

      // Task 1 response:
      if ((*data - '0') == 1) {
        // Display from:
        display.print(F("Accel ")); display.print(from); display.print(F(": "));
        // Received signal strength indicator.
        display.print(F("RSSI = "));
        display.println((char*) buf);
        display.print(F("Our RSSI = "));
        itoa(driver.lastRssi(), buf, 10);
        display.println((char*) buf);
        display.display();
      }

      //------------------------------------------------------------------------------

      // Task 2 response:
      else if ((*data - '0') == 2) {

        // Copy from and buf.
        logArray[from] = from;
        logArray[from + 7] = buf[0];

        // Used to call recvMess, logMessVar, logSuccVar, and csvSucc.
        taskTwo = true;
        // Wait 3 seconds for all devices to respond.
        timeOut = millis();
      }

      //------------------------------------------------------------------------------

      // Task 3 response: Display battery voltages.
      else {
        // Display from:
        display.print(F("Accel ")); display.print(from); display.print(F(": "));

        // Break up buf into separate arrays containing datalogger and radio ADC value.
        uint8_t str1[4];
        uint8_t str2[4];
        uint8_t i = 0;

        // Grab the first three, copy into str1 and add null terminator.
        while (i < 3) {
          str1[i] = buf[i];
          str1[i + 1] = '\0';
          i++;
        }

        // Grab the next three, copy into str2.
        while (i < 6) {
          str2[i - 3] = buf[i];
          str2[i - 2] = '\0';
          i++;
        }

        // Datalogger voltage
        intVAR = atoi(str1);
        // Cast to a float to preserve decimals
        floatVAR = (float) intVAR;
        floatVAR *= 0.0064453125;    // *= (2*3.3/1024)
        display.println(); display.print(F("Datalogger: ")); display.print(floatVAR);
        display.println(F(" V"));

        // Radio voltage
        intVAR = atoi(str2);
        floatVAR = (float) intVAR;
        floatVAR *= 0.0064453125;    // *= (2*3.3/1024)
        display.print(F("Radio: ")); display.print(floatVAR); display.println(F(" V"));

        // Controller voltage
        // Button A also uses A9(voltage divider), temporarily disable pullup resistor to read.
        pinMode(BUTTON_A, INPUT);
        digitalWrite(BUTTON_A, LOW);
        // Reduce "jitter" in ADC readings associated with these pins changing state.
        delay(1000);
        floatVAR = analogRead(BUTTON_A);
        pinMode(BUTTON_A, INPUT_PULLUP);
        floatVAR *= 0.0064453125;    // *= (2*3.3/1024)
        display.print(F("Our Battery: ")); display.print(floatVAR); display.println(F(" V"));
        display.display();
      }
    }
  }
}
//==============================================================================
