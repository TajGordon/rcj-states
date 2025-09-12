#include "Wire.h"

class SteelBarToF
{
  private:
    uint8_t address;
    TwoWire* wire;
    int _read(bool wait)
    {
      if (!wire) return 0;
      while(1)
      {
        wire->beginTransmission(address);
        wire->write(0x10);
        wire->endTransmission();
        if (wire->requestFrom(address, 5))
        {
          uint8_t sequence = wire->read();
          int distance = (wire->read() | ((int)wire->read() << 8) | ((int)wire->read() << 16) | ((int)wire->read() << 24));
          bool changed = sequence != lastSequence;
          lastSequence = sequence;
          if (!wait || changed) {
            return distance;
          }
        }
        delayMicroseconds(10);
      }
      return 0;
    };
  public:
    /** Counter that is incremented whenever a new measurement is received from a request (eg. `nextDistance()` or `currentDistance()`). */
    uint8_t lastSequence;
    SteelBarToF() : wire(NULL), address(0x50), lastSequence(0) {};
    SteelBarToF(uint8_t _address, TwoWire* _wire) : address(_address), wire(_wire), lastSequence(0) {};
    /** Waits for the next sensor measurement. Returns a distance in millimetres. */
    int nextMeasurement() { return _read(true); };
    /** Reads the current distance measurement, which may or may not have updated since last read. Returns a distance in millimetres. */
    int currentMeasurement() { return _read(false); };
};



SteelBarToF sensor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();

  sensor = SteelBarToF(0x50, &Wire);
}

void loop() {
  // put your main code here, to run repeatedly:
  int distance = sensor.nextMeasurement();
  Serial.printf("Distance: %d mm\n", distance);
  delay(100);
}
