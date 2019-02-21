#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

void drawBargraph(uint8_t x, uint8_t y, uint8_t width, uint8_t percent);

class Setting
{
  public:
    Setting(String name, String unit, uint8_t address);
    virtual ~Setting();

    virtual void draw(LiquidCrystal_I2C &lcd, bool active) = 0;
    virtual void increment(uint8_t steps = 1) = 0;
    virtual void decrement(uint8_t steps = 1) = 0;
    virtual void set(uint16_t value) = 0;

    String name;
    String unit;

    uint8_t addressEEPROM;
};

class ValueSetting : public Setting
{
  public:
    ValueSetting(String name, String unit, uint8_t address, uint16_t min, uint16_t max, uint16_t value, uint16_t zeroPoint) : Setting(name, unit, address), minValue(min), maxValue(max), value(value), zeroPoint(zeroPoint)
    {
    }
    virtual ~ValueSetting(){};

    virtual void draw(LiquidCrystal_I2C &lcd, bool active) override;
    virtual void increment(uint8_t steps = 1) override;
    virtual void decrement(uint8_t steps = 1) override;
    virtual void set(uint16_t value) override;

    uint16_t minValue;
    uint16_t maxValue;
    uint16_t value;
    uint16_t zeroPoint;

  protected:
    /**
 * @brief calculate a decbel power ratio
 * 
 * @param out the current power output
 * @param reference the reference power
 * @return float the ratio in dB
 */
    static float dB(uint8_t out, uint8_t reference)
    {
        return 10 * log10((float)out / (float)reference);
    }

    /**
 * @brief add to db values
 * 
 * @param db1 first summand
 * @param db2 second summand
 * @return float the added ratios
 */
    static float addDb(float db1, float db2)
    {
        return 10 * log10(pow(10, db1) + pow(10, db2));
    }
};

template <typename T>
class ChoiceSetting : public Setting
{
  public:
    ChoiceSetting(String name, String unit, uint8_t address, T *choices, uint8_t choice, uint8_t numChoices) : Setting(name, unit, address), choices(choices), choice(choice), numChoices(numChoices)
    {
    }
    virtual ~ChoiceSetting(){};

    virtual void draw(LiquidCrystal_I2C &lcd, bool active) override;
    virtual void increment(uint8_t steps = 1) override;
    virtual void decrement(uint8_t steps = 1) override;
    virtual void set(uint16_t value) override;

    T *choices;
    uint8_t choice;

  private:
    uint8_t numChoices;
};

template class ChoiceSetting<float>;