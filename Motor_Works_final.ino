/*******************************************************
 *  fish-flume-motor-control-tach-v1.0.ino
 *
 *  Purpose:
 *    Controls DC motor speed for the Sandbox fish flume 
 *    using a DRV8871 driver and reads real-time motor 
 *    RPM from the Pololu 5165 gearmotor’s encoder.
 *
 *  Features:
 *    - Potentiometer for speed control
 *    - PWM motor driving via DRV8871
 *    - Direction control (fixed direction for flume use)
 *    - Tachometer reading using encoder channel A ISR
 *    - Live RPM display on 128x32 OLED
 *    - Smoothing + deadband to stabilize noisy pot readings
 *
 *  Hardware:
 *    - Pololu 5165 motor (100:1 HP, 6V) w/ encoder
 *    - DRV8871 motor driver
 *    - Adafruit SSD1306 OLED (128x32)
 *    - Arduino Nano / similar
 *
 *  Author:
 *    Original student version by Levi Lilienthal
 *    Updated + tachometer integration by Riley Meehan
 *
 *******************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// === Display setup ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// =====================================================
//                  PIN DEFINITIONS
// =====================================================
const int potPin      = A0;    // Potentiometer → motor speed command
const int motorPWM    = 5;     // DRV8871 IN1 (PWM)
const int motorDir    = 9;     // DRV8871 IN2 (direction; HIGH = forward)
const int encoderPinA = 12;    // Encoder channel A (interrupt)
const int encoderPinB = 13;    // Encoder channel B

// =====================================================
//           MOTOR + ENCODER CONFIGURATION
// =====================================================
// Pololu 5165: ~100.37:1 gearbox, 12 CPR motor encoder.
// Using a simple rising-edge A count w/ B for direction.
// → 3 counts/motor-shaft rev → ~301 counts/output-shaft rev.
const float COUNTS_PER_REV = 301.0;

// No-load speed ~310 RPM @ 6V; pot will map 0–1023 → 0–310 RPM
const int MAX_RPM       = 310;
const int POT_THRESHOLD = 10;  // Minimum pot value to “wake up” motor

// =====================================================
//               POT SMOOTHING / DEADBAND
// =====================================================
float smoothedPot       = 0;
const float SMOOTHING   = 0.05;  // 5% new, 95% previous
const int DEADBAND      = 3;     // Ignore tiny jitter
int lastPotValue        = 0;

// =====================================================
//             ENCODER COUNTING (ISR + STATE)
// =====================================================
volatile long encoderCount = 0;  // Updated in ISR

unsigned long lastSampleTime = 0;
long lastSampleCount = 0;
float measuredRPM = 0;

// =====================================================
//                 ENCODER INTERRUPT
// =====================================================
// Called on rising edge of channel A.
// Uses channel B to determine rotation direction.
void encoderA_ISR() {
  int b = digitalRead(encoderPinB);

  // Quadrature decoding: if B is HIGH, count up; if LOW, count down.
  if (b == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// =====================================================
//                       SETUP
// =====================================================
void setup() {
  // --- Initialize OLED ---
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Motor Control Ready");
  display.display();
  delay(800);

  // --- Motor output pins ---
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

  // Fixed direction (flip HIGH/LOW if physically reversed)
  digitalWrite(motorDir, HIGH);

  // --- Encoder pins + interrupt ---
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderA_ISR, RISING);

  lastSampleTime = millis();
}

// =====================================================
//                         LOOP
// =====================================================
void loop() {

  // -----------------------------------------------------
  // 1. Read + smooth potentiometer input
  // -----------------------------------------------------
  int rawPot = analogRead(potPin);

  // Exponential smoothing: fast but stable
  smoothedPot = (SMOOTHING * rawPot) + ((1.0 - SMOOTHING) * smoothedPot);

  // Deadband: ignore tiny changes that flicker PWM
  if (abs((int)smoothedPot - lastPotValue) < DEADBAND) {
    smoothedPot = lastPotValue;
  }
  lastPotValue = (int)smoothedPot;


  // -----------------------------------------------------
  // 2. Convert pot → PWM + commanded RPM
  // -----------------------------------------------------
  int pwmValue = 0;
  int cmdRPM   = 0;

  if (smoothedPot > POT_THRESHOLD) {
    pwmValue = map((int)smoothedPot, 0, 1023, 0, 255);
    cmdRPM   = map((int)smoothedPot, 0, 1023, 0, MAX_RPM);
  } else {
    pwmValue = 0;
    cmdRPM   = 0;
  }

  analogWrite(motorPWM, pwmValue);


  // -----------------------------------------------------
  // 3. Encoder → measured RPM (sampled every 100 ms)
  // -----------------------------------------------------
  unsigned long now = millis();
  const unsigned long SAMPLE_INTERVAL = 100;

  if (now - lastSampleTime >= SAMPLE_INTERVAL) {

    long currentCount;

    // Safely grab encoderCount (atomic)
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    long deltaCounts = currentCount - lastSampleCount;
    lastSampleCount  = currentCount;

    float deltaTimeMin = (now - lastSampleTime) / 60000.0; // ms → minutes
    lastSampleTime = now;

    if (deltaTimeMin > 0) {
      measuredRPM = (deltaCounts / COUNTS_PER_REV) / deltaTimeMin;
      measuredRPM = fabs(measuredRPM);  // Show magnitude only
    }
  }


  // -----------------------------------------------------
  // 4. OLED Display Output
  // -----------------------------------------------------
  display.clearDisplay();

  // Row 0: Pot + PWM
  display.setCursor(0, 0);
  display.print("Pot:");
  display.print((int)smoothedPot);

  display.setCursor(64, 0);
  display.print("PWM:");
  display.print(pwmValue);

  // Row 1: Commanded RPM
  display.setCursor(0, 10);
  display.print("Cmd:");
  display.print(cmdRPM);
  display.print(" RPM");

  // Row 2: Measured RPM
  display.setCursor(0, 20);
  display.print("Meas:");
  display.print((int)measuredRPM);
  display.print(" RPM");

  // Bottom: Bar graph showing PWM demand
  int barWidth = map(pwmValue, 0, 255, 0, SCREEN_WIDTH);
  display.fillRect(0, 28, barWidth, 4, SSD1306_WHITE);

  display.display();

  delay(20);  // Mild throttling so display stays smooth
}