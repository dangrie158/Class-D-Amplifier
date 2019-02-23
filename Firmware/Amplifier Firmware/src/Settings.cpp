#include "Settings.h"
/**
 * @brief Draw a simple horizontal bargraph on the specified position
 * 
 * @param x horizontal character where the graph should start
 * @param y line where the graph should be drawn in
 * @param width number of characters the graph is wide
 * @param percent the fill state of the graph
 */
void drawBargraph(LiquidCrystal_I2C &lcd, uint8_t x, uint8_t y, uint8_t width, uint8_t percent)
{
    // move to the first character
    lcd.setCursor(x, y);
    // number of character elemets (start and end character are fixed)
    uint8_t numElements = width - 2;

    uint8_t numSegments = numElements * 5;
    // number of horizontal "on" lines
    uint8_t filledSegments = ((uint32_t)numSegments * (uint32_t)percent) / 100;

    // number of "off" lines
    uint8_t emptySegments = numSegments - filledSegments;
    // print the start character
    lcd.print(char(6));
    // print all full segments
    while (filledSegments >= 5)
    {
        lcd.print(char(5));
        filledSegments -= 5;
    }

    if(filledSegments > 0){
    // print the last segment
        lcd.print(char(filledSegments));
    }

    // print the empty segments
    while (emptySegments >= 5)
    {
        lcd.print(char(0));
        emptySegments -= 5;
    }

    // print the end character
    lcd.print(char(7));
}

void Setting::increment(int8_t steps)
{
    this->set(this->value + steps);
}

void Setting::decrement(int8_t steps)
{
    this->set(this->value - steps);
}

void Setting::set(uint16_t value)
{
    if (value >= this->minValue && value <= this->maxValue)
    {
        this->value = value;
        this->setCallback(this);
    }
}

void ValueSetting::draw(LiquidCrystal_I2C &lcd, bool active)
{
    lcd.setCursor(0, 0);
    // show an indicator if the setting is inactive (cycle through settings)
    if (!active)
    {
        lcd.print(char(0x7E));
    }
    else
    {
        lcd.print(char(0x20));
    }
    lcd.print(this->name);
    lcd.setCursor(0, 1);
    // show whether the setting is currently active (edit setting value)
    if (active)
    {
        lcd.print(char(0x7E));
    }
    else
    {
        lcd.print(char(0x20));
    }

    // create the value string
    String valueText = "";
    // calculate dB value
    if (this->unit.equals("dB"))
    {
        float dbValue = dB(this->value, this->zeroPoint);

        // pad the text to a fixed length
        if (abs(dbValue) < 10)
        {
            valueText.concat(" ");
        }

        // if the db value can go negative, append another char as spacer for the minus sign
        if (this->minValue < this->zeroPoint && dbValue >= 0)
        {
            valueText.concat("+");
        }

        valueText.concat(String(dbValue, 1));
    }

    //display the value as decimal
    else
    {
        if (abs(this->value) < 10)
        {
            valueText.concat(" ");
            if (abs(this->value) < 100)
            {
                valueText.concat(" ");
            }
        }

        valueText.concat(String(this->value, DEC));
    }

    valueText.concat(this->unit);

    // draw the bargraph (if requested)
    uint8_t graphWidth = 16 - 1 - valueText.length();
    uint8_t percent = map(this->value, this->minValue, this->maxValue, 0, 100);
    drawBargraph(lcd, 1, 1, graphWidth, percent);

    // draw the value
    lcd.print(valueText);
}

template <typename T>
void ChoiceSetting<T>::draw(LiquidCrystal_I2C &lcd, bool active)
{
    lcd.setCursor(0, 0);
    // show an indicator if the setting is inactive (cycle through settings)
    if (!active)
    {
        lcd.print(char(0x7E));
    }
    else
    {
        lcd.print(char(0x20));
    }
    lcd.print(this->name);
    lcd.setCursor(0, 1);
    // show whether the setting is currently active (edit setting value)
    if (active)
    {
        lcd.print(char(0x7E));
    }
    else
    {
        lcd.print(char(0x20));
    }

    // create the value string
    String valueText = String(this->choices[this->value], 1) + this->unit;

    // draw the value
    lcd.print(valueText);
}