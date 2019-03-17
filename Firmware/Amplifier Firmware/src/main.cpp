#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

#include "Settings.h"
// pin definitions & mapping
#define AUDIO_DETECT 2
#define POWER_BTN 3
#define AMP_MUTE 4
#define AMP_STANDBY 5
#define AMP_POWER 6
#define SPEAKER_PROTECT 7
#define ENC_A 8
#define ENC_B 9
#define POWER_LED 10
#define FAN_CTRL 11
#define PIN_GAIN0 12
#define PIN_GAIN1 13
#define AMP_DIAG A0
#define T_SENSE A1
#define ENC_BTN A2

// MCP46X1 register addresses
// bits 4-8 in the command byte
#define MCP46X1_VWA (0x00 << 4)
#define MCP46X1_VWB (0x01 << 4)
#define MCP46X1_NVWA (0x02 << 4)
#define MCP46X1_NVWB (0x03 << 4)
#define MCP46X1_TCON (0x04 << 4)
#define MCP46X1_STATUS (0x05 << 4)

// MCP46X1 commands (read / write data)
// bit 2 - 3 in command byte
#define MCP46X1_WRITE_COMMAND (0b00 << 2)
#define MCP46X1_READ_COMMAND (0b11 << 2)

//MCP9701 constants from datasheet
#define MCP9701_OFFSET 0.4f    // Vout @ 0°C
#define MCP9701_COEFF 0.0195f // V / °C
#define ARDUINO_VREF 3.3f     //vref volts
#define ARDUINO_ADC_RES 1024  //resolution of the internal ADC

// constants used for fan control
#define FAN_START_TEMP 40 // temperature where the fan will start spinning
#define FAN_MAX_TEMP 80 // temperature where the fan reaches max-speed
#define FAN_MIN_SPEED 64 // minimum PWM duty cycle for lowest fan speed
#define FAN_MAX_SPEED 256 // full-on duty cycle value

// neccecary functiun prototypes
void setGain(Setting *level);
void setPot(Setting *level);
void loadPot(Setting *level);
void persistPot(Setting *level);
void loadSetting(Setting *level);
void persistSetting(Setting *level);

/**
 * @brief bitmap data for custom bargraph characters
 */
const uint8_t barChars[8][8] = {
    {0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111}, // empty bar char
    {0b11111, 0b00000, 0b10000, 0b10000, 0b10000, 0b10000, 0b00000, 0b11111}, // single line bar char
    {0b11111, 0b00000, 0b11000, 0b11000, 0b11000, 0b11000, 0b00000, 0b11111}, // 2 line bar char
    {0b11111, 0b00000, 0b11100, 0b11100, 0b11100, 0b11100, 0b00000, 0b11111}, // 3 line bar char
    {0b11111, 0b00000, 0b11110, 0b11110, 0b11110, 0b11110, 0b00000, 0b11111}, // 4 line bar char
    {0b11111, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000, 0b11111}, // full line bar char
    {0b11111, 0b10000, 0b10111, 0b10111, 0b10111, 0b10111, 0b10000, 0b11111}, // start character
    {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000}, // end character
};

#define SETTING_MAIN 0    // index of the main volume setting
#define SETTING_STANDBY 7 // index of the standby time setting
#define SETTING_MUTE 8    // index of the mnute time setting
#define NUM_SETTINGS 9    // number of different settings
const String nameMain = "Master Volume";
const String nameMainMuted = "(Muted)";
Setting *settings[NUM_SETTINGS] = {
    new ValueSetting(nameMain, "dB", 0x2D, 0, 255, 0, 255, &loadPot, &setPot, &persistPot),
    new ValueSetting("EQ: Bass", "dB", 0x2C, 0, 128, 0, 64, &loadPot, &setPot, &persistPot),
    new ValueSetting("EQ: Mids", "dB", 0x2B, 0, 128, 0, 64, &loadPot, &setPot, &persistPot),
    new ValueSetting("EQ: Treble", "dB", 0x2A, 0, 128, 0, 64, &loadPot, &setPot, &persistPot),
    new ValueSetting("Mixer: CH A Att", "dB", 0x28, 0, 128, 0, 128, &loadPot, &setPot, &persistPot),
    new ValueSetting("Mixer: CH B Att", "dB", 0x29, 0, 128, 0, 128, &loadPot, &setPot, &persistPot),
    new ChoiceSetting<float>("Gain Setting", "dB", 0x00, (float[]){25.6f, 31.6f, 35.6f, 37.6f}, 0, 4, &loadSetting, &setGain, &persistSetting),
    new ValueSetting("Standby Timer", "sec", 0x02, 0, 999, 300, 0, &loadSetting, [](Setting *setting) {}, &persistSetting),
    new ValueSetting("Mute Timer", "sec", 0x04, 0, 999, 60, 0, &loadSetting, [](Setting *setting) {}, &persistSetting),
};

uint8_t currentMenu = SETTING_MAIN;
bool menuActive = true;

LiquidCrystal_I2C lcd(0x3F, 16, 2); // 0x27 is the I2C bus address for an unmodified backpack
ClickEncoder encoder(ENC_A, ENC_B, ENC_BTN, 4);

volatile int32_t muteTimer;
volatile int32_t standbyTimer;

#define MENU_RESET_TIME 10000 //time with no user input to reset to main menu
volatile int32_t menuTimer;

#define POWER_STANDBY 1
#define POWER_ON 2
#define POWER_MUTED 3
/**
 * @brief the power state the amp currently is in
 */
uint8_t powerState = POWER_STANDBY;
/**
 * @brief the state the amp should be in (checked in the loop and adjusted accordingly)
 * Volatile so it can be changed in an ISR
 */
volatile uint8_t targetPowerState = POWER_STANDBY;

/**
 * @brief ms to wait during power on for the PSU to stabilize
 */
const uint16_t powerOnDelay = 3000;

/**
 * @brief Timer One ISR
 * 
 * Called every 1ms to service the rotary encoder
 */
void timerIsr()
{
  encoder.service();
}

/**
 * @brief Set the Gain pins of the TDA7498 to the current level
 * 
 * @param setting the ChoiceSetting object containing the selected value
 */
void setGain(Setting *setting)
{
  ChoiceSetting<float> *choiceSetting = (ChoiceSetting<float> *)setting;
  Serial.print("Setting Gain to ");
  Serial.print(choiceSetting->choices[setting->value]);
  Serial.println(setting->unit);
  switch (setting->value)
  {
  case 0: // 25.6dB
    digitalWrite(PIN_GAIN0, LOW);
    digitalWrite(PIN_GAIN1, LOW);
    break;
  case 1: // 31.6dB
    digitalWrite(PIN_GAIN0, LOW);
    digitalWrite(PIN_GAIN1, HIGH);
    break;
  case 2: // 35.6dB
    digitalWrite(PIN_GAIN0, HIGH);
    digitalWrite(PIN_GAIN1, LOW);
    break;
  case 3: // 37.6dB
    digitalWrite(PIN_GAIN0, HIGH);
    digitalWrite(PIN_GAIN1, HIGH);
    break;
  }
}

/**
 * @brief persist a setting to the internal EEPROM
 * 
 * @param setting The setting object whose value is saved to the specified address
 */
void persistSetting(Setting *setting)
{
  Serial.print("Persisting Setting ");
  Serial.print(setting->name);
  Serial.print(" to 0x");
  Serial.println(String(setting->value, HEX));
  EEPROM.put(setting->address, setting->value);
}

/**
 * @brief load a setting from the internal EEPROM
 * 
 * @param setting the Setting object whose value should be loaded from its EEPROM address
 */
void loadSetting(Setting *setting)
{
  uint16_t value;
  EEPROM.get(setting->address, value);
  setting->set(value);
}

/**
 * @brief Set the wiper position of both wipers of a MCP46X1
 * 
 * @param setting the ValueSetting object for the pot. address is the I2C address of the POT and value is the wiper position
 */
void setPot(Setting *setting)
{
  Serial.print("Setting Pot ");
  Serial.print(setting->name);
  Serial.print(" to 0x");
  Serial.println(String(setting->value, HEX));
  // copy the value to the pot's ram (volatile register)
  //Wiper A
  uint8_t commandByte = setting->address;
  Wire.beginTransmission(setting->address);
  Wire.write(MCP46X1_VWA | MCP46X1_WRITE_COMMAND);
  Wire.write(setting->value);
  Wire.endTransmission();
delay(100);
  // Wiper B
  Wire.beginTransmission(setting->address);
  Wire.write(MCP46X1_VWB | MCP46X1_WRITE_COMMAND);
  Wire.write(setting->value);
  Wire.endTransmission();
}

/**
 * @brief save the wiper position of a MCP46X1 to its EEPROM
 * 
 * @param setting ValueSetting object containing the address and value of the pot
 */
void persistPot(Setting *setting)
{
  Serial.print("Persisting Pot ");
  Serial.print(setting->name);
  Serial.print(" to 0x");
  Serial.println(String(setting->value, HEX));

  // copy the value to the pot's EEPROM (non-volatile register)
  //Wiper A
  Wire.beginTransmission(setting->address);
  Wire.write(MCP46X1_NVWA | MCP46X1_WRITE_COMMAND);
  Wire.write(setting->value);
  Wire.endTransmission();

  // Wiper B
  Wire.beginTransmission(setting->address);
  Wire.write(MCP46X1_NVWB | MCP46X1_WRITE_COMMAND);
  Wire.write(setting->value);
  Wire.endTransmission();
}

/**
 * @brief load the persistent wiper position of a MCP46X1
 * 
 * This only loads the first wiper register and saves the value into *value* of 
 * the passed settings object. This works because both wipers are always set to the same tap
 * 
 * @param setting 
 */
void loadPot(Setting *setting)
{
  Wire.beginTransmission(setting->address);
  Wire.write(MCP46X1_NVWA | MCP46X1_READ_COMMAND);
  Wire.requestFrom(setting->address, (uint8_t)2);
  setting->set((((uint16_t)Wire.read()) << 8) | Wire.read());
}

/**
 * @brief load all setting values from non-volatile memory
 */
void loadSettings()
{
  for (Setting *setting : settings)
  {

    Serial.print("Initializing value for ");
    Serial.print(setting->name);
    setting->load();
    Serial.print(" to value ");
    Serial.println(setting->value);
  }
}

void resetTimers()
{
  muteTimer = (uint32_t)settings[SETTING_MUTE]->value * 1000;
  standbyTimer = (uint32_t)settings[SETTING_STANDBY]->value * 1000;
}

void powerOn()
{
  Serial.println("Powering On...");
  powerState = POWER_ON;

  //always start in the main screen after power on
  currentMenu = SETTING_MAIN;

  settings[SETTING_MAIN]->name = nameMain;

  //switch on LCD backlight
  lcd.clear();
  lcd.backlight();
    lcd.setCursor(2, 0);
    lcd.print("Powering on");

  uint32_t processStart = millis();

  //ensure speaker protect state
  digitalWrite(SPEAKER_PROTECT, LOW);
  digitalWrite(AMP_MUTE, LOW);
  digitalWrite(AMP_STANDBY, LOW);

  //switch on amp power
  digitalWrite(AMP_POWER, HIGH);

  uint8_t ledBrightness = 0;
  bool pulseDirection = true;
  uint8_t lastSecondsLeft = 0;
  while (millis() - processStart <= powerOnDelay)
  {
    uint8_t secondsLeft = ((powerOnDelay - (millis() - processStart)) / 1000) + 1;
    if (secondsLeft != lastSecondsLeft)
    {
      lcd.setCursor(2, 1);
      lcd.print("ready in ");
      lcd.print(secondsLeft);
      lcd.print(" s");
      lastSecondsLeft = secondsLeft;
    }
    //pulse the power led during power on
    analogWrite(POWER_LED, ledBrightness);
    if (pulseDirection)
    {
      ledBrightness++;
    }
    else
    {
      ledBrightness--;
    }

    //change the pulsing direction if limits are reached
    if (ledBrightness == 0 || ledBrightness == 255)
    {
      pulseDirection = !pulseDirection;
      delay(50);
    }
    delay(2);
  }

  //switch on the amp and connect the speakers
  digitalWrite(AMP_STANDBY, HIGH);
  digitalWrite(AMP_MUTE, HIGH);
  digitalWrite(SPEAKER_PROTECT, HIGH);

  //switch the power LED to solid ON
  while (ledBrightness < 255)
  {
    ledBrightness++;
    analogWrite(POWER_LED, ledBrightness);
    delay(2);
  }
  digitalWrite(POWER_LED, HIGH);

  // override any changes to the target state that happened during power on
  targetPowerState = POWER_ON;
}

void powerOff()
{
  Serial.println("Powering Off...");
  powerState = POWER_STANDBY;

  // power off the amp and disconnect the speakers
  digitalWrite(SPEAKER_PROTECT, LOW);
  digitalWrite(AMP_MUTE, LOW);
  digitalWrite(AMP_STANDBY, LOW);

  //make sure the fan is off
  digitalWrite(FAN_CTRL, LOW);

  // print a message on the display
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Goodbye");
  //fade out the power LED
  for (uint8_t brightness = 0xFF; brightness > 0; brightness--)
  {
    analogWrite(POWER_LED, brightness);
    delay(5);
  }
  digitalWrite(POWER_LED, LOW);

  //clear the display and switch off backlight for standby
  lcd.clear();
  lcd.noBacklight();

  //switch off the power supply
  digitalWrite(AMP_POWER, LOW);

  // make sure the target power state is set to the current state
  // this effectivley debounces the power button since it overrides any
  // changes that happened during power off
  targetPowerState = POWER_STANDBY;
}

void muteAmp()
{
  Serial.println("Muting Amp");

  settings[SETTING_MAIN]->name = nameMainMuted;

  digitalWrite(AMP_MUTE, LOW);

  powerState = POWER_MUTED;
  targetPowerState = POWER_MUTED;
}

void unmuteAmp()
{
  Serial.println("Unmuting Amp");

  settings[SETTING_MAIN]->name = nameMain;

  digitalWrite(AMP_MUTE, HIGH);

  powerState = POWER_ON;
  targetPowerState = POWER_ON;
}

void audioDetectISR()
{
  if (powerState != POWER_ON)
  {
    targetPowerState = POWER_ON;
  }

  resetTimers();
}

void powerButtonISR()
{
  // disable interrupt on first ISR call to debounce button
  detachInterrupt(digitalPinToInterrupt(POWER_BTN));

  if (powerState != POWER_ON)
  {
    resetTimers();
    targetPowerState = POWER_ON;
    menuTimer = MENU_RESET_TIME;
  }
  else
  {
    muteTimer = 0;
    standbyTimer = 0;
    targetPowerState = POWER_STANDBY;
    menuTimer = MENU_RESET_TIME;
  }

  // reenable interrupts
  attachInterrupt(digitalPinToInterrupt(POWER_BTN), powerButtonISR, FALLING);
}

float getHeatsinkTemp()
{
  float currentTemperature = (analogRead(T_SENSE) * ARDUINO_VREF) / ARDUINO_ADC_RES;
  currentTemperature -= MCP9701_OFFSET;
  currentTemperature /= MCP9701_COEFF;
  return currentTemperature;
}

void setup()
{
  // setup and start the hardware UART
  Serial.begin(9600);
  Serial.println("Starting up...");

  
  // setup GPIO data direction for all pins
  pinMode(AUDIO_DETECT, INPUT);
  pinMode(POWER_BTN, INPUT_PULLUP);
  pinMode(AMP_MUTE, OUTPUT);
  pinMode(AMP_STANDBY, OUTPUT);
  pinMode(AMP_POWER, OUTPUT);
  pinMode(SPEAKER_PROTECT, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(FAN_CTRL, OUTPUT);
  pinMode(PIN_GAIN0, OUTPUT);
  pinMode(PIN_GAIN1, OUTPUT);
  pinMode(AMP_DIAG, INPUT);
  pinMode(T_SENSE, INPUT);
  pinMode(POWER_LED, OUTPUT);

  //Start the timer 1 to service the click encoder every 1ms
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  //load all settings from non-volatile memory
  loadSettings();

  //reset all timers
  resetTimers();

  // initialize the lcd and load the custom bar chart chars into display memory
  lcd.init();
  delay(10);
  for (uint8_t i = 0; i < 8; i++)
  {
    lcd.createChar(i, (uint8_t *)barChars[i]);
  }

  // setup the power button and audio detect interrupt
  attachInterrupt(digitalPinToInterrupt(POWER_BTN), powerButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUDIO_DETECT), audioDetectISR, CHANGE);

  targetPowerState = POWER_ON;
  
}

void loop()
{
  
  //decrement the timers by the time passed since the last loop
  static uint32_t lastLoop = millis();
  uint32_t timePassed = millis() - lastLoop;
  lastLoop = millis();
  if (menuTimer > 0)
  {
    menuTimer -= timePassed;
    lcd.backlight();
  }
  else
  {
    //restore the active setting to the persisted value (if it's not the master volume)
    if (currentMenu != SETTING_MAIN)
    {
      settings[currentMenu]->load();
      lcd.clear();
    }
    currentMenu = SETTING_MAIN;
    menuActive = true;
    lcd.noBacklight();
  }

  if (muteTimer > 0)
  {
    muteTimer -= timePassed;
    targetPowerState = POWER_ON;
  }
  else
  {
    targetPowerState = POWER_MUTED;
  }

  if (standbyTimer > 0)
  {
    standbyTimer -= timePassed;
  }
  else
  {
    targetPowerState = POWER_STANDBY;
  }

  //always reset the timers while a audio signal is present
  if (digitalRead(AUDIO_DETECT))
  {
    resetTimers();
    targetPowerState = POWER_ON;
  }

  if (targetPowerState != powerState)
  {
    if (targetPowerState == POWER_STANDBY)
    {
      powerOff();
    }
    else if (targetPowerState == POWER_MUTED)
    {
      muteAmp();
    }
    else if (targetPowerState == POWER_ON)
    {
      if (powerState == POWER_MUTED)
      {
        unmuteAmp();
      }
      else
      {
        powerOn();
      }
    }
  }

  if (powerState != POWER_STANDBY)
  {
    settings[currentMenu]->draw(lcd, menuActive);

    // handle input
    // encoder button:
    ClickEncoder::Button btnState = encoder.getButton();
    if (btnState == ClickEncoder::Clicked)
    {
      //persist the setting before leaving the active state
      if (menuActive)
      {
        settings[currentMenu]->persist();
      }
      //switch menu active state between setting value adjust and setting selection
      menuActive = !menuActive;

      resetTimers();
      menuTimer = MENU_RESET_TIME;
    }

    //handle encoder rotary action
    int16_t value = encoder.getValue();
    if (value != 0)
    {
      if (menuActive)
      {
        settings[currentMenu]->increment(value);
      }
      else
      {
        lcd.clear();
        if (value > 0)
        {
          currentMenu += 1;
          currentMenu %= NUM_SETTINGS;
        }
        else
        {
          currentMenu -= 1;
          //handle wrap around
          if (currentMenu == 0xFF)
          {
            currentMenu = NUM_SETTINGS - 1;
          }
        }
      }

      resetTimers();
      menuTimer = MENU_RESET_TIME;
    }

    // handle the fan controller depending on the current heatsink temerature

    // debounce the serial report of temp and fan setting
    // a lower value increases the report interval (reports more often)
    static uint8_t tempVerbosity = 127;
    static uint8_t tempVerbosityCounter = 0;
    float currentTemperature = getHeatsinkTemp();
    long fanSpeed = map(currentTemperature, FAN_START_TEMP, FAN_MAX_TEMP, FAN_MIN_SPEED, FAN_MAX_SPEED);
    // constrain the maximum speed to full PWM value
    fanSpeed = min(fanSpeed, FAN_MAX_SPEED);
    // check if we're 
    if(fanSpeed < FAN_MIN_SPEED){
      fanSpeed = 0;
    }

    if(tempVerbosityCounter % tempVerbosity == 0){
      Serial.print("Current heatsink temperature: ");
      Serial.print(String(currentTemperature, 1));
      Serial.print(" °C. Fan is at speed: ");
      Serial.println(fanSpeed);
    }
    tempVerbosityCounter++;
    analogWrite(FAN_CTRL, fanSpeed);
  }
}
