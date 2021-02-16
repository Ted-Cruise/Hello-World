#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

#include <Arduino.h>

class MovingAverage {
  public: 
    MovingAverage(byte numReadings);
    
    void update(float reading);

    float getMean();
    float getStDev();

    byte getNumRecordedReadings();

  private:
    float * _readings;
    byte _numReadings;
    byte _numRecordedReadings;
    byte _readingIndex;
};

#endif