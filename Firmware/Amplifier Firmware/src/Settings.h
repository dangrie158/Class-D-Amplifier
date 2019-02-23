#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class Setting;
typedef void (*SettingCallback)(Setting*);

void drawBargraph(uint8_t x, uint8_t y, uint8_t width, uint8_t percent);

class Setting
{
  public:
    Setting(String name, String unit, uint8_t address, uint16_t min, uint16_t max, uint16_t value, SettingCallback loader, SettingCallback onChange, SettingCallback onPersist) : name(name), unit(unit), address(address), minValue(min), maxValue(max), value(value), loadFunction(loader), setCallback(onChange), persistCallback(onPersist) {}
    virtual ~Setting() {}

    virtual void draw(LiquidCrystal_I2C &lcd, bool active) = 0;
    virtual void increment(int8_t steps = 1);
    virtual void decrement(int8_t steps = 1);
    virtual void set(uint16_t value);

    void persist(){
      this->persistCallback(this);
    }

    void load(){
      this->loadFunction(this);
    }

    String name;
    String unit;
    uint8_t address;

    uint16_t minValue;
    uint16_t maxValue;
    uint16_t value;

private:
SettingCallback loadFunction;
SettingCallback setCallback;
SettingCallback persistCallback;
    
};

class ValueSetting : public Setting
{
  public:
    ValueSetting(String name, String unit, uint8_t address, uint16_t min, uint16_t max, uint16_t value, uint16_t zeroPoint,  SettingCallback loader, SettingCallback onChange, SettingCallback onPersist) : Setting(name, unit, address, min, max, value, loader, onChange, onPersist), zeroPoint(zeroPoint)
    {
    }
    virtual ~ValueSetting(){};

    virtual void draw(LiquidCrystal_I2C &lcd, bool active) override;

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
    ChoiceSetting(String name, String unit, uint8_t address, T *choices, uint8_t choice, uint8_t numChoices, SettingCallback loader, SettingCallback onChange, SettingCallback onPersist) : Setting(name, unit, address, 0, numChoices - 1, choice, loader, onChange, onPersist), choices(choices)
    {
    }
    virtual ~ChoiceSetting(){};

    virtual void draw(LiquidCrystal_I2C &lcd, bool active) override;

    T *choices;
};

template class ChoiceSetting<float>;