#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include "Settings.h"

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

Setting *settings[8] = {
    new ValueSetting("Master Volume", "dB", 0x00, 0, 128, 0, 128),
    new ValueSetting("EQ: Bass", "dB", 0x00, 0, 128, 0, 64),
    new ValueSetting("EQ: Mids", "dB", 0x00, 0, 128, 0, 64),
    new ValueSetting("EQ: Treble", "dB", 0x00, 0, 128, 0, 64),
    new ValueSetting("Mixer: CH A Att", "dB", 0x00, 0, 128, 0, 128),
    new ValueSetting("Mixer: CH B Att", "dB", 0x00, 0, 128, 0, 128),
    new ValueSetting("Sleep Timer", "sec", 0x00, 0, 999, 300, 0),
    new ChoiceSetting<float>("Gain Setting", "dB", 0x00, (float[]){25.6f, 31.6f, 35.6f, 37.6f}, 0, 4)};

LiquidCrystal_I2C lcd(0x3F, 16, 2); // 0x27 is the I2C bus address for an unmodified backpack

void setup()
{
  // initialize the lcd and load the custom bar chart chars into display memory
  lcd.init();
  delay(100);
  for (uint8_t i = 0; i < 8; i++)
  {
    lcd.createChar(i, (uint8_t *)barChars[i]);
  }

  lcd.backlight();
}

void loop()
{
  for (uint8_t i = 0; i < 128; i++)
  {
    settings[7]->draw(lcd, i % 2);
    settings[7]->set(i%4);
    delay(100);
  }
}