/*
  This menu screen will display the INPUT VOLTAGE, OUTPUT VOLTAGE, FREQUENCY, CURRENT, KVA, KW, BATTERY VOLTAGE, BATTERY PERCENTAGE, UPS MODE - NORMAL OR BYPASS, BYPASS VOLTAGE, FREQUENCY,
  KVA, KW, BATTERY CHARGING CURRENT, DISHARGING CURRENT, VERSION

  Menu screen 1 will display:
  INPUTS:
  Mains phase voltages
  Mains Phase current
  Mains Frequency
  Mains power Factor
  Bypass Phase voltage and frequency

  Programmable Parameters:
  INPUT VOLTAGE - LOW THRESHHOLD AND HIGH THRESHHOLD
  OUTPUT VOLTAGE - LOW THRESHOLD AND HIGH THRESHOLD
  INPUT/OUTPUT FREQUENCY - LOW THRESHOLD AND HIGH THRESHOLD
  BATTERY TYPE
  BATTERY WH/AH
  BATTERY NUMBER
  BATTERY GROUP
  BOOST CHARGE
  MAX CHARGE CURRENT


*/

#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include "EmonLib.h"
#include "ACS712.h"
#include "RTClib.h"

LiquidCrystal_I2C lcd(0x3f, 20, 4);
RTC_DS3231 rtc;
EnergyMonitor mains;
EnergyMonitor bypass;
EnergyMonitor output;

String name = "Solar PV Controller";
String version = "V01_21";
//char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

//#define LCD_Cols 4

#define battvol A0
#define battTemp A1

// Menu page
#define right 3
#define left 4
#define up 5
#define down 6
#define enter 7
#define esc 8
#define menuButton 9
#define NUM_OF_SAMPLES 10

int sumOfSamples = 0;

boolean current_left = HIGH;
boolean last_left = HIGH;
boolean current_right = HIGH;
boolean last_right = HIGH;

boolean current_up = HIGH;
boolean last_up = HIGH;
boolean current_down = HIGH;
boolean last_down = HIGH;

boolean current_enter = HIGH;
boolean last_enter = HIGH;

boolean isInHomePage = true;

int numOfPages = 10;
int numOfNewPages;

uint32_t runHour = 0;
int battCapacity = 0;

// Parameters to input
int battWh = 0;
int Totbatt = 0;

bool batlow = false;

float mains_voltage;
float bypass_voltage;
float output_voltage;
float mains_current;
float output_current;
float mains_frequency;
float bypass_frequency;
float output_frequency;
float mains_realPower;
float mains_apparentPower;
float output_realPower;
float output_apparentPower;
float mains_powerFactor;
float output_powerFactor;
float dc_current;
float dc_voltage;

int page_counter;
int new_page_counter = 1;

float readACCurrentInput(int pin) {
  const float factor = 50;

  const float VMIN;
  const float VMAX;

  const float ADCV = 5.0;
  float voltage;
  float current;
  float sum = 0;

  long sampleTime = millis();
  int counter = 0;

  while (millis() - sampleTime > 500) {
    voltage = analogRead(pin) * ADCV / 1023.0;
    //    current =
  }

}

float readACCurrentOutput(int pin) {

}

float readDCVoltage(int pin) {
  float dcVoltage;
  int sample_count;
  float sumOfCounts = 0;

  while (sample_count < NUM_OF_SAMPLES) {
    sumOfCounts += analogRead(pin);
    sample_count++;
    delay(10);
  }
  sample_count = 0;
  sumOfCounts = 0;

  dcVoltage = (sumOfCounts / NUM_OF_SAMPLES * 5.0);
  return dcVoltage;
}

float readDCCurrent() {
  //  float I = sensor.getCurrentDC();
  //  return i;
}

float readBatteryTemp(int pin, float temperature) {

}

boolean isVoltageOkay(){
  
}

// LCD Home
String Home[] = {
  "Mains Input",
  "Bypass",
  "Output",
  "Frequency",
  "Battery",
  "Time Stamp",
  "Version",
};

boolean debounce_leftRight(boolean last, int pin) {
  boolean current = digitalRead(pin);

  if (last != current) {
    delay(5);
    current = digitalRead(pin);
  }
  return current;
}

boolean debounce_upDown(boolean last, int pin)
{
  boolean current = digitalRead(pin);
  if (last != current)
  {
    delay(5);
    current = digitalRead(pin);
  }
  return current;
}

int buttonUpDownPressed() {
  current_up = debounce_upDown(last_up, up);
  current_down = debounce_upDown(last_down, down);

  int page_counter = 1;

  if (last_up == HIGH and current_up == LOW) {
    lcd.clear();

    if (page_counter < numOfPages) {
      page_counter = page_counter + 1;
    }
    else {
      page_counter = numOfPages;
    }
  }
  last_up = current_up;

  if (last_down == HIGH and current_down == LOW) {
    lcd.clear();

    if (page_counter > 1) {
      page_counter = page_counter - 1;
    }
    else {
      page_counter = 1;
    }
  }

  last_down = current_down;

  return page_counter;
}

int buttonLeftRightPressed() {
  current_left = debounce_leftRight(last_left, left);
  current_right = debounce_leftRight(last_right, right);

  int page_counter = 1;

  if (last_left == HIGH and current_left == LOW) {
    lcd.clear();

    if (page_counter < numOfPages) {
      page_counter = page_counter + 1;
    }
    else {
      page_counter = numOfPages;
    }
  }
  last_left = current_left;

  if (last_right == HIGH and current_right == LOW) {
    lcd.clear();

    if (page_counter > 1) {
      page_counter = page_counter - 1;
    }
    else {
      page_counter = 1;
    }
  }
  last_right = current_right;

  return page_counter;
}

void LCD_head() {
  int arr;
  int len = sizeof(Home) / sizeof(Home[0]);
  if (last_left == HIGH and current_left == LOW) {
    current_left = last_left;
    if (arr <= len) {
      arr++;
      arr = len;
      lcd.setCursor(7, 0);
      lcd.print(Home[arr]);
    }
  }
}

void LCD_home() {
  lcd.setCursor(7, 1);
  lcd.print("WELCOME");
}

void LCD_mains_input() {
  LCD_head();

  mains_voltage = mains.Vrms;
  mains_current = mains.Irms;
  mains_realPower = mains.realPower;
  mains_apparentPower = mains.apparentPower;
  mains_frequency = 50;
  mains_powerFactor = mains.powerFactor;

  page_counter = buttonUpDownPressed();

  lcd.setCursor(1, 0);
  lcd.print("Mains(V): " + String(mains_voltage) + " V");
  lcd.setCursor(2, 0);
  lcd.print("Mains(A): " + String(mains_current) + " A");
  lcd.setCursor(3, 0);
  lcd.print("R.P(kW): " + String(mains_realPower) + " kW");
  //  Scroll down to power and bypass
  switch (page_counter) {
    case 1:
      lcd.setCursor(3, 0);
      lcd.print("A.P(kVA): " + String(mains_apparentPower) + " kVA");
      break;

    case 2:
      lcd.setCursor(3, 0);
      lcd.print("Frequency(Hz): " + String(mains_frequency) + " Hz");

    case 3:
      lcd.setCursor(3, 0);
      lcd.print("Power Factor: " + String(mains_powerFactor));
  }
}

void LCD_DC_voltage() {
  LCD_head();

  dc_voltage = readDCVoltage(5);
  dc_current = readDCCurrent();

  lcd.setCursor(1, 0);
  lcd.print("Battery(V): " + String(dc_voltage) + " V");
  lcd.setCursor(2, 0);
  lcd.print("Battery(A): " + String(dc_current) + " A");
}

void LCD_bypass_reading() {
  LCD_head();

  bypass_voltage = bypass.Vrms;
  bypass_frequency = 50;

  lcd.setCursor(1, 0);
  lcd.print("Bypass(V): " + String(bypass_voltage) + " V");
  lcd.setCursor(2, 0);
  lcd.print("Frequency(Hz): " + String(bypass_frequency) + " A");
}

void LCD_output() {
  LCD_head();

  output_voltage = output.Vrms;
  output_current = output.Irms;
  output_realPower = output.realPower;
  output_apparentPower = output.apparentPower;
  output_powerFactor = output.powerFactor;

  page_counter = buttonUpDownPressed();

  lcd.setCursor(1, 0);
  lcd.print("Output(V): " + String(output_voltage) + " V");
  lcd.setCursor(2, 0);
  lcd.print("Output(A): " + String(output_current) + " A");
  lcd.setCursor(3, 0);
  lcd.print("R.P(kW): " + String(output_realPower) + " kW");
  switch (page_counter) {
    case 1:
      lcd.setCursor(3, 0);
      lcd.print("A.P(kVA): " + String(output_apparentPower) + " kVA");
      break;

    case 2:
      lcd.setCursor(3, 0);
      lcd.print("Power Factor: " + String(output_powerFactor));
      break;
  }
}

void LCD_battery() {
  LCD_head();

  lcd.setCursor(1, 0);
  lcd.print("Battery Voltage");
  lcd.setCursor(2, 0);
  lcd.print("Battery Current");
}

void LCD_version() {
  LCD_head();
}

void LCD_record() {
  LCD_head();
  lcd.setCursor(1, 1);
  lcd.print(name);
  lcd.setCursor(7, 2);
  // Time stamp
}

//End of function

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(esc, INPUT_PULLUP);
  pinMode(enter, INPUT_PULLUP);
  pinMode(menuButton, INPUT_PULLUP);

  mains.voltage(2, 234.26, 1.7);
  mains.current(6, 111.1);

  bypass.voltage(3, 234.26, 1.7);

  output.voltage(4, 234.26, 1.7);
  output.current(8, 111.1);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.clear();
}

void loop() {
  Serial.println(mains_voltage);
  delay(500);
  Serial.println(bypass_voltage);
  delay(500);
  Serial.println(output_voltage);
  delay(500);

  page_counter = buttonLeftRightPressed();

  switch (page_counter) {
    case 1:
      LCD_home();
      break;

    case 2:
      LCD_mains_input();
      break;

    case 3:
      LCD_bypass_reading();
      break;

    case 4:
      LCD_DC_voltage();
      break;

    case 5:
      LCD_output();
      break;

    case 6:
      LCD_battery();
      break;

    case 7:
      LCD_record();
      break;

    case 8:
      LCD_version();
      break;
  }
}
