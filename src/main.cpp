
// the device will have 3 adc inputs 2 for the 2 cell lipo battery and 1 incoming pwm signal (on off for the leds), 3 outputs for controlling the led color (RGB)
// if the battery voltage is below a certain threshold the leds will blink red regardless of the pwm signal
// if the pwm signal is changing the leds will change color accordingly
// if the pwm signal is off the leds will be off
// lipo1Pin - 26
// lipo2Pin - 27
// pwmPin - 28
// redPin - 5
// greenPin - 6
// bluePin - 7

#include <Arduino.h>

#define LIPO1_PIN A1
#define LIPO2_PIN A2
#define PWM_PIN 6
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
#define INTERNAL_LED 13

#define VOLTAGE_THRESHOLD 3.5
#define VOLTAGE_OFFSET 0.35

#define SAMPLE_DELAY 100

void setup()
{
  Serial.begin(115200);
  pinMode(LIPO1_PIN, INPUT);
  pinMode(LIPO2_PIN, INPUT);
  pinMode(PWM_PIN, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(INTERNAL_LED, OUTPUT);
}
bool blinkState = false;

void setPin(int pin, int value)
{
  if (value == 0)
  {
    digitalWrite(pin, LOW);
  }
  else
  {
    digitalWrite(pin, HIGH);
  }
}

void loop()
{
  float lipo1Voltage = analogRead(LIPO1_PIN) * (5.0 / 1023.0) - VOLTAGE_OFFSET;
  float lipo2Voltage = (analogRead(LIPO2_PIN) * (5.0 / 1023.0) - VOLTAGE_OFFSET) * 2.0 - lipo1Voltage;
  int pwmSignal = pulseIn(PWM_PIN, HIGH, 50000UL) - 1110;
  Serial.println("S1: " + String(lipo1Voltage) + " S2: " + String(lipo2Voltage) + " PWM: " + String(pwmSignal));
  if (lipo1Voltage < VOLTAGE_THRESHOLD || lipo2Voltage < VOLTAGE_THRESHOLD)
  {
    // Serial.println("Battery voltage is low");
    // Serial.println("Cell 1 voltage: " + String(lipo1Voltage));
    // Serial.println("Cell 2 voltage: " + String(lipo2Voltage));
    if (blinkState)
    {
      setPin(RED_PIN, HIGH);
      setPin(GREEN_PIN, LOW);
      setPin(BLUE_PIN, LOW);
      setPin(INTERNAL_LED, HIGH);
    }
    else
    {
      setPin(RED_PIN, LOW);
      setPin(GREEN_PIN, LOW);
      setPin(BLUE_PIN, LOW);
      setPin(INTERNAL_LED, LOW);
    }
    blinkState = !blinkState;
  }
  else if (pwmSignal < 700)
  {
    // Serial.println("PWM signal is off");
    setPin(RED_PIN, LOW);
    setPin(GREEN_PIN, LOW);
    setPin(BLUE_PIN, LOW);
  }
  else if (pwmSignal > 0 && pwmSignal < 10)
  {
    Serial.println("PWM signal is low");
    setPin(RED_PIN, HIGH);
    setPin(GREEN_PIN, LOW);
    setPin(BLUE_PIN, LOW);
  }
  else if (pwmSignal >= 760 && pwmSignal < 780)
  {
    Serial.println("PWM signal is medium low");
    setPin(RED_PIN, HIGH);
    setPin(GREEN_PIN, HIGH);
    setPin(BLUE_PIN, LOW);
  }
  else if (pwmSignal >= 781 && pwmSignal < 800)
  {
    Serial.println("PWM signal is medium");
    setPin(RED_PIN, LOW);
    setPin(GREEN_PIN, HIGH);
    setPin(BLUE_PIN, LOW);
  }
  else if (pwmSignal >= 801 && pwmSignal < 820)
  {
    Serial.println("PWM signal is medium high");
    setPin(RED_PIN, LOW);
    setPin(GREEN_PIN, HIGH);
    setPin(BLUE_PIN, HIGH);
  }
  else if (pwmSignal >= 821 && pwmSignal < 840)
  {
    Serial.println("PWM signal is high");
    setPin(RED_PIN, LOW);
    setPin(GREEN_PIN, LOW);
    setPin(BLUE_PIN, HIGH);
  }
  else if (pwmSignal >= 841 && pwmSignal < 860)
  {
    Serial.println("PWM signal is very high");
    setPin(RED_PIN, HIGH);
    setPin(GREEN_PIN, LOW);
    setPin(BLUE_PIN, HIGH);
  }
  else if (pwmSignal >= 861 && pwmSignal < 900)
  {
    Serial.println("PWM signal is very very high");
    setPin(RED_PIN, HIGH);
    setPin(GREEN_PIN, HIGH);
    setPin(BLUE_PIN, HIGH);
  }
  else
  {
    float lipoVoltage = (lipo1Voltage + lipo2Voltage);
    Serial.println("Battery voltage is normal");
    Serial.println("Cell 1 voltage: " + String(lipo1Voltage));
    Serial.println("Cell 2 voltage: " + String(lipo2Voltage));
    Serial.println("Total voltage: " + String(lipoVoltage));
    if (lipoVoltage > 8.0)
    {
      setPin(RED_PIN, LOW);
      setPin(GREEN_PIN, HIGH);
      setPin(BLUE_PIN, LOW);
    }
    else if (lipoVoltage > 7.6 && lipoVoltage < 8.0)
    {
      setPin(RED_PIN, HIGH);
      setPin(GREEN_PIN, HIGH);
      setPin(BLUE_PIN, LOW);
    }
    else if (lipoVoltage > 7.3 && lipoVoltage < 7.6)
    {
      setPin(RED_PIN, HIGH);
      setPin(GREEN_PIN, LOW);
      setPin(BLUE_PIN, LOW);
    }
    else if (lipoVoltage < 7.3)
    {
      if (blinkState)
      {
        setPin(RED_PIN, HIGH);
        setPin(GREEN_PIN, LOW);
        setPin(BLUE_PIN, LOW);
        setPin(INTERNAL_LED, HIGH);
      }
      else
      {
        setPin(RED_PIN, LOW);
        setPin(GREEN_PIN, LOW);
        setPin(BLUE_PIN, LOW);
        setPin(INTERNAL_LED, LOW);
      }
      blinkState = !blinkState;
    }
  }
  delay(SAMPLE_DELAY);
}

// void loop()
// {
//   float lipo1Voltage = analogRead(LIPO1_PIN) * (5.0 / 1023.0) - VOLTAGE_OFFSET;
//   float lipo2Voltage = analogRead(LIPO2_PIN) * (5.0 / 1023.0) - VOLTAGE_OFFSET;
//   Serial.println("Cell 1 voltage: " + String(lipo1Voltage));
//   Serial.println("Cell 2 voltage: " + String(lipo2Voltage));
//   Serial.println("**************************");
//   delay(100);
// }