#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal.h>
#include <string.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ============================================================
// LCD WIRING
// ============================================================
//
// LCD pin 1  -> Arduino GND
// LCD pin 2  -> Arduino 5V
// LCD pin 3  -> Centre leg of 10k contrast potentiometer
// LCD pin 4  -> Arduino D10
// LCD pin 5  -> Arduino GND
// LCD pin 6  -> Arduino D11
// LCD pin 7  -> Leave disconnected
// LCD pin 8  -> Leave disconnected
// LCD pin 9  -> Leave disconnected
// LCD pin 10 -> Leave disconnected
// LCD pin 11 -> Arduino A0
// LCD pin 12 -> Arduino A1
// LCD pin 13 -> Arduino A2
// LCD pin 14 -> Arduino A3
// LCD pin 15 -> Arduino 5V through 220-ohm resistor
// LCD pin 16 -> Arduino GND

LiquidCrystal lcd(10, 11, A0, A1, A2, A3);

// ============================================================
// BUZZER WIRING
// ============================================================
//
// Active buzzer positive/signal -> Arduino D12
// Active buzzer negative        -> Arduino GND
//
// Jetson command:
//
// BIP\n
//
// Each BIP command adds one beep to the queue.

const uint8_t BUZZER_PIN = 12;

const unsigned long BEEP_ON_TIME_MS = 100;
const unsigned long BEEP_GAP_TIME_MS = 100;

bool buzzerIsOn = false;
bool buzzerInGap = false;

unsigned long buzzerStateStartTime = 0;

// Maximum number of queued beeps.
const uint8_t MAX_QUEUED_BEEPS = 20;
uint8_t queuedBeeps = 0;


// ============================================================
// JETSON SERIAL COMMAND PROTOCOL
// ============================================================
//
// LCD command:
//
// LCD:text to display\n
//
// Maximum LCD text length: 32 characters.
//
// Characters 1-16 appear on LCD row 1.
// Characters 17-32 appear on LCD row 2.
//
// Example:
//
// LCD:RTK FIX         ACCURACY 5CM\n
//
//
// Buzzer command:
//
// BIP\n
//
//
// Both commands use the same USB serial connection as the existing
// Arduino-to-Jetson DATA packets.
//
// The outgoing DATA packet format remains exactly unchanged.

const uint8_t MAX_COMMAND_LENGTH = 40;

char commandBuffer[MAX_COMMAND_LENGTH + 1];
uint8_t commandBufferIndex = 0;
bool commandOverflow = false;


// ============================================================
// LCD STATE
// ============================================================

const uint8_t LCD_TEXT_LENGTH = 32;

// Time of the last valid LCD packet received from the Jetson.
unsigned long lastGPSPacketTime = 0;

// Received LCD text remains visible for this duration.
const unsigned long GPS_PACKET_TIMEOUT_MS = 1500;

// Fallback LCD refresh rate.
const unsigned long LCD_FALLBACK_REFRESH_MS = 500;

unsigned long lastFallbackLCDRefresh = 0;

bool gpsPacketActive = false;


// ============================================================
// ENCODERS
// ============================================================

// Encoder order:
// 0 = Front Right
// 1 = Front Left
// 2 = Rear Right
// 3 = Rear Left

// Arduino Uno encoder connections:
//
// Front Right:
// Encoder A -> Arduino D2
// Encoder B -> Arduino D3
//
// Front Left:
// Encoder A -> Arduino D4
// Encoder B -> Arduino D5
//
// Rear Right:
// Encoder A -> Arduino D6
// Encoder B -> Arduino D7
//
// Rear Left:
// Encoder A -> Arduino D8
// Encoder B -> Arduino D9
//
// BNO055:
//
// SDA -> Arduino A4
// SCL -> Arduino A5

const uint8_t pinA[4] = {2, 4, 6, 8};
const uint8_t pinB[4] = {3, 5, 7, 9};

volatile long encoderCount[4] = {0, 0, 0, 0};
long lastEncoderCount[4] = {0, 0, 0, 0};

volatile uint8_t previousState[4] = {0, 0, 0, 0};

volatile uint8_t previousPortD = 0;
volatile uint8_t previousPortB = 0;

unsigned long lastEncoderPrint = 0;

// Latest values displayed on the LCD fallback screen.
float latestHeading = 0.0;
long latestDelta[4] = {0, 0, 0, 0};

// Full x4 quadrature decoding table.
const int8_t quadratureTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};


inline void updateEncoderFromState(
    uint8_t encoderIndex,
    uint8_t currentA,
    uint8_t currentB
) {
    uint8_t currentState = (currentA << 1) | currentB;

    uint8_t transition =
        (previousState[encoderIndex] << 2) | currentState;

    encoderCount[encoderIndex] += quadratureTable[transition];

    previousState[encoderIndex] = currentState;
}


// Handles Arduino pins D2-D7.
ISR(PCINT2_vect) {
    uint8_t currentPortD = PIND;
    uint8_t changedPins = currentPortD ^ previousPortD;

    // Front Right: D2 and D3
    if (changedPins & (_BV(PD2) | _BV(PD3))) {
        uint8_t currentA =
            (currentPortD & _BV(PD2)) ? 1 : 0;

        uint8_t currentB =
            (currentPortD & _BV(PD3)) ? 1 : 0;

        updateEncoderFromState(
            0,
            currentA,
            currentB
        );
    }

    // Front Left: D4 and D5
    if (changedPins & (_BV(PD4) | _BV(PD5))) {
        uint8_t currentA =
            (currentPortD & _BV(PD4)) ? 1 : 0;

        uint8_t currentB =
            (currentPortD & _BV(PD5)) ? 1 : 0;

        updateEncoderFromState(
            1,
            currentA,
            currentB
        );
    }

    // Rear Right: D6 and D7
    if (changedPins & (_BV(PD6) | _BV(PD7))) {
        uint8_t currentA =
            (currentPortD & _BV(PD6)) ? 1 : 0;

        uint8_t currentB =
            (currentPortD & _BV(PD7)) ? 1 : 0;

        updateEncoderFromState(
            2,
            currentA,
            currentB
        );
    }

    previousPortD = currentPortD;
}


// Handles Arduino pins D8-D9.
ISR(PCINT0_vect) {
    uint8_t currentPortB = PINB;
    uint8_t changedPins = currentPortB ^ previousPortB;

    // Rear Left: D8 and D9
    if (changedPins & (_BV(PB0) | _BV(PB1))) {
        uint8_t currentA =
            (currentPortB & _BV(PB0)) ? 1 : 0;

        uint8_t currentB =
            (currentPortB & _BV(PB1)) ? 1 : 0;

        updateEncoderFromState(
            3,
            currentA,
            currentB
        );
    }

    previousPortB = currentPortB;
}


// ============================================================
// LCD FUNCTIONS
// ============================================================

// Print exactly 16 characters on one LCD row.
// Unused positions are filled with spaces.
void printLCDRow(
    uint8_t row,
    const char *text
) {
    lcd.setCursor(0, row);

    uint8_t characterIndex = 0;

    while (
        characterIndex < 16 &&
        text[characterIndex] != '\0'
    ) {
        lcd.write(text[characterIndex]);

        characterIndex++;
    }

    while (characterIndex < 16) {
        lcd.write(' ');

        characterIndex++;
    }
}


// Display up to 32 characters received from the Jetson.
//
// Characters 0-15 are displayed on row 1.
// Characters 16-31 are displayed on row 2.
void displayJetsonLCDText(const char *text) {
    char firstRow[17];
    char secondRow[17];

    for (uint8_t index = 0; index < 16; index++) {
        firstRow[index] = ' ';
        secondRow[index] = ' ';
    }

    firstRow[16] = '\0';
    secondRow[16] = '\0';

    for (
        uint8_t index = 0;
        index < 16 && text[index] != '\0';
        index++
    ) {
        firstRow[index] = text[index];
    }

    for (
        uint8_t index = 0;
        index < 16 && text[index + 16] != '\0';
        index++
    ) {
        secondRow[index] = text[index + 16];
    }

    printLCDRow(0, firstRow);
    printLCDRow(1, secondRow);
}


// Convert an encoder value to a two-digit magnitude.
//
// Examples:
//
// 0   -> 00
// 5   -> 05
// -8  -> 08
// 32  -> 32
// 120 -> 99
uint8_t encoderValueForLCD(long value) {
    if (value < 0) {
        value = -value;
    }

    if (value > 99) {
        value = 99;
    }

    return (uint8_t)value;
}


// Display fallback information:
//
// NO GPS CPS:360
// E100E200E300E400
void displayFallbackStatus() {
    char firstRow[17];
    char secondRow[17];

    int headingInteger =
        (int)(latestHeading + 0.5);

    if (headingInteger >= 360) {
        headingInteger = 0;
    }

    if (headingInteger < 0) {
        headingInteger = 0;
    }

    uint8_t encoder1 =
        encoderValueForLCD(latestDelta[0]);

    uint8_t encoder2 =
        encoderValueForLCD(latestDelta[1]);

    uint8_t encoder3 =
        encoderValueForLCD(latestDelta[2]);

    uint8_t encoder4 =
        encoderValueForLCD(latestDelta[3]);

    snprintf(
        firstRow,
        sizeof(firstRow),
        "NO GPS CPS:%03d",
        headingInteger
    );

    snprintf(
        secondRow,
        sizeof(secondRow),
        "E1%02uE2%02uE3%02uE4%02u",
        encoder1,
        encoder2,
        encoder3,
        encoder4
    );

    printLCDRow(0, firstRow);
    printLCDRow(1, secondRow);
}


// ============================================================
// NON-BLOCKING BUZZER
// ============================================================

// Add one beep to the queue.
void queueBeep() {
    if (queuedBeeps < MAX_QUEUED_BEEPS) {
        queuedBeeps++;
    }
}


// Update the buzzer without using delay().
void updateBuzzer() {
    unsigned long currentTime = millis();

    // Start a new beep.
    if (
        !buzzerIsOn &&
        !buzzerInGap &&
        queuedBeeps > 0
    ) {
        queuedBeeps--;

        digitalWrite(BUZZER_PIN, HIGH);

        buzzerIsOn = true;
        buzzerStateStartTime = currentTime;

        return;
    }

    // Finish the current beep.
    if (
        buzzerIsOn &&
        currentTime - buzzerStateStartTime >=
            BEEP_ON_TIME_MS
    ) {
        digitalWrite(BUZZER_PIN, LOW);

        buzzerIsOn = false;
        buzzerInGap = true;
        buzzerStateStartTime = currentTime;

        return;
    }

    // Finish the gap between two beeps.
    if (
        buzzerInGap &&
        currentTime - buzzerStateStartTime >=
            BEEP_GAP_TIME_MS
    ) {
        buzzerInGap = false;
    }
}


// ============================================================
// SERIAL COMMAND HANDLING
// ============================================================

void processJetsonCommand(const char *command) {
    // Buzzer command.
    if (strcmp(command, "BIP") == 0) {
        queueBeep();

        return;
    }

    // LCD command.
    if (strncmp(command, "LCD:", 4) == 0) {
        const char *lcdText = command + 4;

        displayJetsonLCDText(lcdText);

        lastGPSPacketTime = millis();
        gpsPacketActive = true;

        return;
    }

    // Unknown commands are ignored.
}


// Read incoming serial commands without blocking.
//
// No delay(), readString(), parseInt() or waiting loop is used.
void receiveJetsonCommands() {
    while (Serial.available() > 0) {
        char receivedCharacter =
            (char)Serial.read();

        // Ignore carriage return.
        if (receivedCharacter == '\r') {
            continue;
        }

        // Newline completes the command.
        if (receivedCharacter == '\n') {
            if (
                !commandOverflow &&
                commandBufferIndex > 0
            ) {
                commandBuffer[commandBufferIndex] =
                    '\0';

                processJetsonCommand(commandBuffer);
            }

            commandBufferIndex = 0;
            commandOverflow = false;

            continue;
        }

        // Store characters while there is room.
        if (!commandOverflow) {
            if (
                commandBufferIndex <
                MAX_COMMAND_LENGTH
            ) {
                commandBuffer[commandBufferIndex] =
                    receivedCharacter;

                commandBufferIndex++;
            } else {
                // Discard the rest of an oversized command
                // until its newline arrives.
                commandOverflow = true;
            }
        }
    }
}


// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(9600);

    Serial.println("Starting Arduino...");

    // LCD setup.
    lcd.begin(16, 2);

    printLCDRow(0, "NO GPS");
    printLCDRow(1, "Starting...");

    // Buzzer setup.
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // BNO055 setup.
    //
    // BNO055 SDA -> Arduino A4
    // BNO055 SCL -> Arduino A5
    if (!bno.begin()) {
        Serial.println(
            "[ERROR] BNO055 not detected. Check wiring!"
        );

        printLCDRow(0, "BNO055 ERROR");
        printLCDRow(1, "CHECK WIRING");

        while (1) {
        }
    }

    bno.setExtCrystalUse(true);
    bno.setMode(OPERATION_MODE_IMUPLUS);

    // Encoder pin setup.
    for (uint8_t index = 0; index < 4; index++) {
        pinMode(pinA[index], INPUT_PULLUP);
        pinMode(pinB[index], INPUT_PULLUP);
    }

    // Read initial port states.
    previousPortD = PIND;
    previousPortB = PINB;

    // Initialise Front Right quadrature state.
    previousState[0] =
        (
            (
                (previousPortD & _BV(PD2))
                    ? 1
                    : 0
            ) << 1
        ) |
        (
            (previousPortD & _BV(PD3))
                ? 1
                : 0
        );

    // Initialise Front Left quadrature state.
    previousState[1] =
        (
            (
                (previousPortD & _BV(PD4))
                    ? 1
                    : 0
            ) << 1
        ) |
        (
            (previousPortD & _BV(PD5))
                ? 1
                : 0
        );

    // Initialise Rear Right quadrature state.
    previousState[2] =
        (
            (
                (previousPortD & _BV(PD6))
                    ? 1
                    : 0
            ) << 1
        ) |
        (
            (previousPortD & _BV(PD7))
                ? 1
                : 0
        );

    // Initialise Rear Left quadrature state.
    previousState[3] =
        (
            (
                (previousPortB & _BV(PB0))
                    ? 1
                    : 0
            ) << 1
        ) |
        (
            (previousPortB & _BV(PB1))
                ? 1
                : 0
        );

    noInterrupts();

    // Enable pin-change interrupt group for D0-D7.
    PCICR |= _BV(PCIE2);

    // Enable encoder interrupts on D2-D7.
    PCMSK2 |= _BV(PCINT18); // D2
    PCMSK2 |= _BV(PCINT19); // D3
    PCMSK2 |= _BV(PCINT20); // D4
    PCMSK2 |= _BV(PCINT21); // D5
    PCMSK2 |= _BV(PCINT22); // D6
    PCMSK2 |= _BV(PCINT23); // D7

    // Enable pin-change interrupt group for D8-D13.
    PCICR |= _BV(PCIE0);

    // Enable encoder interrupts only on D8 and D9.
    PCMSK0 |= _BV(PCINT0); // D8
    PCMSK0 |= _BV(PCINT1); // D9

    interrupts();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
    // Read incoming Jetson commands.
    // This function never waits.
    receiveJetsonCommands();

    // Update the buzzer without blocking.
    updateBuzzer();

    if (millis() - lastEncoderPrint >= 50) {
        sensors_event_t event;

        bno.getEvent(&event);

        float heading = event.orientation.x;

        long delta[4];

        noInterrupts();

        for (uint8_t index = 0; index < 4; index++) {
            delta[index] =
                encoderCount[index] -
                lastEncoderCount[index];

            lastEncoderCount[index] =
                encoderCount[index];
        }

        interrupts();

        // Invert right-side encoders.
        delta[0] *= -1;
        delta[2] *= -1;

        // Save latest values for the fallback LCD screen.
        latestHeading = heading;

        for (uint8_t index = 0; index < 4; index++) {
            latestDelta[index] = delta[index];
        }

        // ====================================================
        // EXISTING SERIAL OUTPUT FORMAT — UNCHANGED
        // ====================================================

        Serial.print("DATA:");
        Serial.print("HDG:");
        Serial.print(heading, 1);

        Serial.print(":ENC_FR_DELTA:");
        Serial.print(delta[0]);

        Serial.print(":ENC_FL_DELTA:");
        Serial.print(delta[1]);

        Serial.print(":ENC_RR_DELTA:");
        Serial.print(delta[2]);

        Serial.print(":ENC_RL_DELTA:");
        Serial.print(delta[3]);

        Serial.println();

        lastEncoderPrint = millis();
    }

    // Return to the fallback display if no new LCD packet
    // has arrived for 1.5 seconds.
    if (
        gpsPacketActive &&
        millis() - lastGPSPacketTime >=
            GPS_PACKET_TIMEOUT_MS
    ) {
        gpsPacketActive = false;

        lastFallbackLCDRefresh = 0;
    }

    // Refresh the fallback LCD without blocking.
    if (
        !gpsPacketActive &&
        millis() - lastFallbackLCDRefresh >=
            LCD_FALLBACK_REFRESH_MS
    ) {
        displayFallbackStatus();

        lastFallbackLCDRefresh = millis();
    }

    // Check again for serial commands.
    receiveJetsonCommands();

    // Continue servicing the buzzer.
    updateBuzzer();
}
