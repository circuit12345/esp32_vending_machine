#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Arduino.h>
unsigned long previousMillis = 0;
long interval = 10000;
const int acceptor = 22;
const int coinPin = 33;
const int resetButtonPin = 21;           // GPIO pin for the reset button
volatile int coinCount = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 2000;
Servo myServo;                          // Servo object
bool rotateFlag = false;                // Flag to control servo rotation
unsigned long startTime;

// Serial communication for DWIN display
const byte rxPin = 19;
const byte txPin = 18;
HardwareSerial dwin(1);
unsigned char Buffer[9];
int product_qty;

// NVS preferences object for storing stock count
Preferences preferences;
int stock = 60;                          // Initial stock value


// DWIN command arrays
unsigned char loading[8]     = {0x5a, 0xa5, 0x05, 0x82, 0x35, 0x00, 0x00, 0x00};
unsigned char success[8]     = {0x5a, 0xa5, 0x05, 0x82, 0x35, 0x00, 0x00, 0x01};
unsigned char failed[8]      = {0x5a, 0xa5, 0x05, 0x82, 0x35, 0x00, 0x00, 0x02};
unsigned char resetq[8]      = {0x5a, 0xa5, 0x05, 0x82, 0x58, 0x00, 0x00, 0x00};
uint8_t readCommand[]        = {0x5A, 0xA5, 0x04, 0x83, 0x58, 0x00, 0x01};
byte switchPageCommand[]     = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};
byte switchPageCommand2[]    = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01};

// Interrupt service routine to count coins
void IRAM_ATTR countCoins() {
    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > debounceDelay) {
        coinCount++;
        lastDebounceTime = currentTime;
    }
}


// Function to get and reset the coin count
int getAndResetCoinCount() {
    int count = coinCount;               // Store the current count
    return count;                        // Return the stored count
}

void setup() {
  
    Serial.begin(115200);
    myServo.attach(32);                  // Attach the servo to GPIO 32
    pinMode(acceptor, OUTPUT);           // Set acceptor pin as output
    digitalWrite(acceptor, HIGH);
    delay(2000);
    digitalWrite(acceptor, LOW);
    delay(2000);// Initialize acceptor as HIGH (disabled)
    digitalWrite(acceptor, HIGH);
    dwin.begin(115200, SERIAL_8N1, rxPin, txPin);
    delay(1000);
    pinMode(coinPin, INPUT_PULLUP);      // Initialize coin pin as input with pullup
    attachInterrupt(digitalPinToInterrupt(coinPin), countCoins, FALLING); // Attach interrupt
    pinMode(resetButtonPin, INPUT);  // Initialize reset button pin
    // Initialize preferences with namespace "stock_app"
    preferences.begin("stock_app", false);
    // Load stock value from non-volatile storage or initialize if not found
    stock = preferences.getInt("stock", 60);
        // Check if reset button is pressed at startup
    if (digitalRead(resetButtonPin) == LOW) {
        resetStock();                    // Reset stock if button is pressed during power on
    }
    Serial.print("Initial Stock: ");
    Serial.println(stock);
    pageswitch();                        // Initialize DWIN display pages
}
// Main loop function
void loop() {
    realy_control();                     // Control relay based on coin count and selection
    Status();                            // Check and update the status based on inputs
}

// Function to control the relay and product dispensing
void realy_control() {
    int count = getAndResetCoinCount();  // Get and reset coin count

    dwin.write(readCommand, sizeof(readCommand)); // Send read command to DWIN display
    if (dwin.available()) {
        for (int i = 0; i <= 8; i++) {
            delay(10);
            Buffer[i] = dwin.read();     // Read response into buffer
        }
        Serial.println(Buffer[4], HEX);
        if (Buffer[0] == 0X5A) {         // Check for valid response
            switch (Buffer[4]) {         // Process response based on buffer value
                case 0x58:               // For light control
                    if (Buffer[8] == 0) {
                        product_qty = 0;
                        digitalWrite(acceptor, HIGH);
                        coinCount = 0;
                    } else if (Buffer[8] == 1) {
                        product_qty = 1;
                        digitalWrite(acceptor, LOW);
                        delay(5);
                    } else if (Buffer[8] == 2) {
                        product_qty = 2;
                        digitalWrite(acceptor, LOW);
                    }
                    break;
            }
        }
    }
    Serial.println(product_qty);
    Serial.println(stock);
    dwin.write(readCommand, sizeof(readCommand)); // Send another read command
    delay(5);
}
// Function to dispense one product unit
void dispense() {
    if (stock > 0) {
        rotateFlag = true;               // Set rotate flag to true
        startTime = millis();
        for (int angle = 0; angle <= 90; angle += 1) {
            myServo.write(angle);        // Rotate the servo to dispense product
            delay(15);
        }
        if (rotateFlag && (millis() - startTime >= 10000)) {
            rotateFlag = false;
        }
        stock--;                         // Decrease stock count
        preferences.putInt("stock", stock);  // Save updated stock value to NVS
        Serial.print("Stock after dispense: ");
        Serial.println(stock);
        reset();                         // Reset the system
    } else {
        Serial.println("Out of stock!");
        dwin.write(failed, 8);           // Send a failure message if out of stock
    }
}

// Function to dispense two product units
void dispense2() {
    if (stock >= 2) {
        rotateFlag = true;               // Set rotate flag to true
        startTime = millis();
        for (int angle = 0; angle <= 90; angle += 1) {
            myServo.write(angle);        // Rotate the servo to dispense first product
            delay(15);
        }
        for (int angle = 0; angle <= 90; angle += 1) {
            myServo.write(angle);        // Rotate the servo to dispense second product
            delay(15);
        }
        if (rotateFlag && (millis() - startTime >= 10000)) {
            rotateFlag = false;
        }
        stock -= 2;                      // Decrease stock count by 2
        preferences.putInt("stock", stock);  // Save updated stock value to NVS
        Serial.print("Stock after dispense2: ");
        Serial.println(stock);
        reset();                         // Reset the system
    } else {
        Serial.println("Not enough stock!");
        dwin.write(failed, 8);           // Send a failure message if not enough stock
    }
}

// Function to check and update the system status
void Status() {
    int count = getAndResetCoinCount();  // Get and reset coin count
    Serial.println(count);
    int statusCode = product_qty * 10 + count;  // Calculate status code

    switch (statusCode) {
        case 12:
            dwin.write(success, 8);      // Send success message
            dispense();                  // Dispense one product unit
            coinCount = 0;
            product_qty = 0;
            pageswitch();                // Switch pages on DWIN display
            break;
        case 23:
            dwin.write(success, 8);      // Send success message
            dispense2();                 // Dispense two product units
            coinCount = 0;
            product_qty = 0;
            pageswitch();                // Switch pages on DWIN display
            break;
        default:
            dwin.write(loading, 8);      // Send loading message
            break;
    }
    delay(10);
}
// Function to reset the system
void reset() {
    dwin.write(resetq, 8);               // Send reset command to DWIN display
    digitalWrite(acceptor, HIGH); 
    //fire();// Disable the acceptor
    delay(1000);
}
// Function to switch pages on DWIN display
void pageswitch() {
    dwin.write(switchPageCommand, sizeof(switchPageCommand)); // Switch to first page
    delay(2000);
    dwin.write(switchPageCommand2, sizeof(switchPageCommand2)); // Switch to second page
    delay(2000);
}

// Function to reset stock to initial value
void resetStock() {
    stock = 60;                          // Reset stock to initial value
    preferences.putInt("stock", stock);  // Save the reset stock value to NVS
    Serial.println("Stock has been reset to 60");
}
