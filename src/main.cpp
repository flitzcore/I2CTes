#include <Arduino.h>
#include "uRTCLib.h"
// sudo chmod -R 777 /dev/ttyACM0
// sudo chmod -R 777 /dev/bus/usb/
/*
// uRTCLib rtc;
uRTCLib rtc(0x68);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup()
{
  Serial.begin(9600);
  delay(3000); // wait for console opening

  URTCLIB_WIRE.begin();

  // Comment out below line once you set the date & time.
  // Following line sets the RTC with an explicit date & time
  // for example to set January 13 2022 at 12:56 you would call:
  rtc.set(0, 53, 10, 5, 8, 12, 22);
  // rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year)
  // set day of week (1=Sunday, 7=Saturday)
}

void loop()
{
  rtc.refresh();

  Serial.print("Current Date & Time: ");
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());

  Serial.print(" (");
  Serial.print(daysOfTheWeek[rtc.dayOfWeek() - 1]);
  Serial.print(") ");

  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.println(rtc.second());

  delay(1000);
}

*/
uRTCLib rtc(0x68);
const int potPin = PA3;
const int LEDBuiltIn = PC13;
const int key = PA0;
const int opsiPeriode[3] = {1000, 5000, 10000};
short int count = 0;
short int buttonState;
bool set = false;
int ledState = LOW;
short int potValue;
float vPot;
unsigned long lastMilisTime;
const char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

void setup()
{
  // rtc.set(0, 31, 12, 6, 2, 12, 22);
  //  Led internal
  pinMode(LEDBuiltIn, OUTPUT);
  // Untuk potensio
  pinMode(potPin, INPUT);

  pinMode(key, INPUT_PULLUP);
  Serial.begin(115200);
  delay(3000); // wait for console opening

  URTCLIB_WIRE.begin();

  // Comment out below line once you set the date & time.
  // Following line sets the RTC with an explicit date & time
  // for example to set January 13 2022 at 12:56 you would call:

  // rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year)
  // set day of week (1=Sunday, 7=Saturday)
}

void loop()
{
  rtc.refresh();
  potValue = analogRead(potPin);
  buttonState = digitalRead(key);
  if (buttonState == HIGH && !set)
  {
    count++;
    set = true;
  }
  if (buttonState == LOW)
    set = false;

  if (count > 2)
    count = 0;

  if (millis() - lastMilisTime > opsiPeriode[count])
  {
    lastMilisTime = millis();
    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
    vPot = (float)(potValue / 1023.00) * 5.00;
    Serial.print("Waktu Pengukuran: ");
    Serial.print(rtc.day());
    Serial.print('/');
    Serial.print(rtc.month());
    Serial.print('/');
    Serial.print(rtc.year());

    Serial.print(" (");
    Serial.print(daysOfTheWeek[rtc.dayOfWeek() - 1]);
    Serial.print(") ");

    Serial.print(rtc.hour());
    Serial.print(':');
    Serial.print(rtc.minute());
    Serial.print(':');
    Serial.print(rtc.second());
    Serial.print("   , Tegangan Analog: ");
    Serial.print(vPot, 2);
    Serial.println(" Volt");
  }
}
//
//
///
///
//
//
//
//
//
//
//
//
/*
#include <Arduino.h>

const int LEDBuiltIn = PC13;
const int potPin = PA2;
const int LEDEksternalPin = PA1;
const int key = PA0;
float ledFade = 0;
unsigned long lastFadeTime;
const long fadeTime = 20;
float dimSpeed = 10;
int buttonState;
bool fadeup;
int potValue;
double vPot;
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LEDBuiltIn, OUTPUT);
  // Untuk potensio
  pinMode(potPin, INPUT);
  // led eksternal
  pinMode(LEDEksternalPin, OUTPUT);
  pinMode(key, INPUT_PULLUP);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop()
{
  buttonState = digitalRead(key);
  if (buttonState == HIGH)
    ledFade = 0;
  else
  {
    if (millis() - lastFadeTime > fadeTime)
    {
      if (fadeup)
      {
        analogWrite(LEDEksternalPin, ledFade);
        ledFade += dimSpeed;
        if (ledFade >= 255)
        {
          ledFade = 255;
          fadeup = false;
        }
      }
      else
      {
        analogWrite(LEDEksternalPin, ledFade);
        ledFade -= dimSpeed;
        if (ledFade <= 0)
        {
          ledFade = 0;
          fadeup = true;
        }
      }
      lastFadeTime = millis();
    }
  }

  potValue = analogRead(potPin);
  vPot = (double)(potValue / 1023.00) * 5.00;
  Serial.print("Tegangan Analog: ");
  Serial.print(vPot, 2);
  Serial.println(" volt");
  analogWrite(LEDEksternalPin, ledFade);
  if (vPot < 2)
    digitalWrite(LEDBuiltIn, HIGH);
  else
    digitalWrite(LEDBuiltIn, LOW);
}
*/
